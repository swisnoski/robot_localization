#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TOFINISH: define additional helper functions if needed



class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        # TOFINISH: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)


    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)


    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """

        # THIS IS THE MAIN CODE. LET"S WALK THROUGH IT 

        # first, if we don't have a scan to process (ie, if we haven't 
        # recieved a scan yet), don't do anything
        if self.scan_to_process is None:
            return
        
        # otherwise, we begin our processing. msg becomes the most recently published LaserScan
        msg = self.scan_to_process

        # then, we get the most recent odom pose based on the timestamp of the laserscan
        # we also get the difference in time between the new pose and the last updated pose 
        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        
        # check if we TFHelper could actually get the pose 
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            # check if our delta_t is negative (self.transform_helper messed up)
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                # so we just reset out self.scan_to_process and wait for a new one to be published 
                self.scan_to_process = None
            return
        
        # next, we convert our msg (which is LaserScan data) to polar coords IN THE ROBOT FRAME
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        # retuns a tuple of two numpy arrays
        # r is a list of distances 
        # theta is a list of corresponding angles in radians

        # we then print the list of distances and thetas 
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))

        # clear the current scan so that we can process the next one
        self.scan_to_process = None
        # notably, this thread is seperate from the publishers and subscribers,
        # so they will continue to update, but this thread won't rerun until the 
        # rest of the processing below is done 

        # set our new pose based on the get_matching_odom_pose timestamp 
        self.odom_pose = new_pose

        # convert our odom pose to the x,y,theta in the global frame and print it 
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))


        # then, we begin the main particle filter loop 
        # PARTICLE PROCESSING LOOP STARTS HERE 

        if not self.current_odom_xy_theta: 
            # this will only happen once, at the beginning of the loop
            self.current_odom_xy_theta = new_odom_xy_theta


        elif not self.particle_cloud: 
            # this will also only happen once (afaik) after this, we reinitialize the 
            # particle cloud in update_initial_pose when the initialpose topic is published to
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)

        # so since the last two function (in theory) will only run once, the four functions 
        # below are what actually constitute our particle filter
        
        # first, we check if we have moved far enough to update our filter. 
        # if we havent, we just publish the same particles with an updated timestamp 
        # (timestamp is given by msg.header)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            
            # since we've moved, we need to update the positions of the particles accordingly
            # we do this using the updated odom_pose compared with the old odom_pose 
            self.update_particles_with_odom()    

            # after we update the positions, we reweight the particles based on the new 
            # LaserScan. we take in the robot LaserScan (r, theta) to compare 
            self.update_particles_with_laser(r, theta)   

            # after we have our new weightings, we update the estimate of our robot's pose 
            # update robot's pose based on particles
            self.update_robot_pose()  

            # and now that we have our new pose, we resample our particle swarm 
            # resample particles to focus on areas of high density, based on weights 
            self.resample_particles()               

        # lastly, publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)


    def moved_far_enough_to_update(self, new_odom_xy_theta):
        '''
        ok so this basically just compares the old position to the new position and returns true or false 
        if the robot has moved farther than the distance or angular threshold. 
        '''
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TOFINISH: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin
        
        #Initializes empty variables to hold the particles x, y and theta for the next robot guess
        x=0
        y=0
        theta = 0

        #Goes through everything and weighs each particle relative position information
        for particle in self.particle_cloud:
            x+= particle.x*particle.w
            y+= particle.y*particle.w
            theta += particle.theta*particle.w

        #Converts the theta angle measurement from 2D frame to 3 quadernion
        quaternion = quaternion_from_euler(0,0, theta)


        #Should add some quadernon stuff here once we understand it more
        self.robot_pose = Pose(position=Point(x=x, y=y, z=0.0), 
                               orientation = Quaternion(x=quaternion[0], y = quaternion[1], z = quaternion[2], w = quaternion[3]))
        
        
        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")



    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        # ok so the first thing we do is set the new odom using the TFHelper class
        # by converting the self.odom_pose to x, y, and theta. 
        # the self.odom_pose is updated with get_matching_odom_pose every loop as 
        # long as there is a scan to process. get_matching_odom_pose is another THHelper function 
        # that returns a odom pose given a certain timestamp. the timestamp comes from 
        # self.scan_to_process, which is just a subscription function to the LaserScan topic 

        # so basically, new_odom_xy_theta is just the newest x, y, theta values 
        # based on the odom_pose at the time of our most recently processed laser scan. 

        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        old_odom_xy_theta = self.current_odom_xy_theta


        # compute the change in x,y,theta since our last update

        if self.current_odom_xy_theta:
            # now, if we have an existing odom, we calculate the difference in the 
            # last recorded odom and the newly provided odom 
            delta = (new_odom_xy_theta[0] - old_odom_xy_theta[0],
                     new_odom_xy_theta[1] - old_odom_xy_theta[1],
                     new_odom_xy_theta[2] - old_odom_xy_theta[2])

            # after calculating delta, we can update our current_odom
            self.current_odom_xy_theta = new_odom_xy_theta 

        else:
            # if we DON'T have an existing odom, we set the newly provided odom as the 
            # existing odom. this should never happen because we already check for this 
            # earlier in the loop. not sure why it's here 
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # okay so now that we've calculated the change in odom, we need to update the odometery of 
        # each of our particles. the calculated change in odom takes place in the global frame.
        # luckily, each of our particles are already calculated in the global frame, so it's as simple 
        # as adding the deltas to each of the points in our particle cloud

        for particle in self.particle_cloud:
            # particle is [x,y,theta], so is delta
            particle.x += delta[0]
            particle.y += delta[1]
            particle.theta += delta[2]





    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        # TODO: fill out the rest of the function
        


    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """
        # TOFINISH: implement this
        # Iterates through every particle in the particle cloud
        for particle in self.particle_cloud:
            # Initializes an error counter
            error =0
            # Couples the collection of laser scans into a list with r (lidar distance) and theta (angle), then begins iterating through them
            for single_laser in enumerate(zip(r,theta)):
                # Checks if the lidar distance r is finitie and therefore usable
                if math.isfinite(single_laser(0)):
                    # Translates angle of robot and laser into the map coordinate frame
                    angle = single_laser(1)+self.current_odom_xy_theta(2)
                    distance = single_laser(0)
                    # Maps the laser's scan's x and y coordinates over the particles position to check for obstacles
                    new_x = particle.x + distance*math.cos(angle)
                    new_y = particle.y + distance*math.sin(angle)
                    # Compares the particle's 'new' position to the nearest obstacle, with a higher error the further apart the two are
                    error +=  self.occupancy_field.get_closest_obstacle_distance(new_x, new_y)
            #Fitness function to evaluate weight based on this distance, closer to the obstacle each new particle was the higher the weigtht 
            particle.w = 1/(min(1000, error/(len(self.particle_cloud))**2)**2)
        pass


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)
        

    def initialize_particle_cloud(self, timestamp, xy_theta=None, num_particles=500):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used 
            num_particles: an int value that controls how many particles we generate in our particle 
                      cloud. Default value is 500."""
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = []
        # xy_theta is a list that hax [x, y, theta/yaw]
        # we want to initialize a list of particles in self.particle_cloud
        # each particle takes in an x, y, theta, and weight 
        # when we initalize, we just want to set x, y, and theta, weight will just be 1 
        center = (xy_theta[0], xy_theta[1])
        cov = np.array([[10, -10], [-10, 10]])
        rng = np.random.default_rng()

        gaussian_dist = rng.multivariate_normal(center, cov, size=num_particles)

        # Initialize particles around (x, y, theta)
        for x, y in gaussian_dist:
            self.particle_cloud.append(Particle(x, y, xy_theta[2], w=1.0))
                
        self.normalize_particles()
        self.update_robot_pose()
        #TOFINISH


    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # let's just do a basic normalization for now. will probably want to switch to 
        # a better normalization if we end up weighting our particles differently 

        # start by just summing the weights
        total_weight = 0
        for particle in self.particle_cloud:
            total_weight += particle.w

        # then divide each individual weight by the total weight so they sum to 1
        for particle in self.particle_cloud:
            particle.w /= total_weight

        #TOFINISH



    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)


    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
