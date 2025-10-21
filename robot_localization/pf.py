""" This is the full code for the robot localization project. In order to run this file 
properly, you will first need to start rviz under the specified configuration, as well as  
load the map. You can then play bag files to test your build. Please see README.md for 
further detail about this particle filter."""


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
from helper_functions import TFHelper, draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler


class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
           a la l theta: the yaw of the hypothesis relative to the map frame
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
        """
        We initialize our particle filter here with each of the attributes as described above. 
        """
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 500          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

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

        # we create an occupancy_field (map) and TFHelper for helper functions 
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # your particle cloud will go here
        self.particle_cloud = []
        self.current_odom_xy_theta = []

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)


    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform.
    
            Arguments: 
                - None 

            Returns:
                - None 
        """
        # We just need to check if we have an actual last timestamp 
        if self.last_scan_timestamp is None:
            return
    
        # If we do have a last_scan_timestamp, we add a small amount of buffer time and update our 
        # map and odom frames by timestamp 
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)


    def loop_wrapper(self):
        """ 
        This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2 

        Arguments: 
            - None 

        Returns:
            - None 
            """
        while True:
            self.run_loop()
            time.sleep(0.1)


    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing. While the comments do walk 
            through the most important parts of the code, here is a simple breakdown 
            of what this function actually does: 

            1.) We get an odom pose based on the most recent postdated timestamp. 
            2.) We check if TFHelper could get the actual pose. 
            3.) We convert our laser scan to distances and thetas, 
                and convert our odom pose to x, y, and theta. 
            4.) We check if our particle cloud is initialized.
            5.) We check if we have moved far enough in order to update our particles. 
            6.) If we have moved, we update the location of our particles. 
            7.) We then update the weights of the particle, and then our pose estimation. 
            8.) Lastly, we resample our particles based on our new weights and then 
                publish all our particles. 

            Arguments: 
                - None 

            Returns:
                - None 
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
        """ Compares the old position to the new position and returns true or false 
            if the robot has moved farther than the linear or angular distance thresholds. 

            Arguments: 
                - new_odom_xy_theta (list), a list containing the robot's x, y, and theta values in the global frame 
            
            Returns: 
                - boolean, depenedant on how far the robot has moved since we last updated our particles
        """
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry. 
            
            Arguments: 
                - None 

            Returns:
                - None 
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
        delta = (0,0,0)
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
            # existing odom. this should in theory never happen because we already check for this 
            # earlier in the loop. 
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # okay so now that we've calculated the change in odom, we need to update the odometery of 
        # each of our particles. the calculated change in odom takes place in the global frame.
        # luckily, each of our particles are already calculated in the global frame, so it's as simple 
        # as adding the deltas to each of the points in our particle cloud

        # we should also add some noise to these particles 255

        for particle in self.particle_cloud:
            particle.x += delta[0] 
            particle.y += delta[1] 
            particle.theta += delta[2]
        
        self.add_noise(self.particle_cloud)
        

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data. We do this by 
            comparing each particle to the lidar scan data. 

            Arguments: 
                - r (list): the distance readings to obstacles
                - theta (list): the angle relative to the robot frame 
                    for each corresponding  distance reading 

            Returns:
                - None 
        """

        # The particles should already be normalized, but normalize them just in case 
        self.normalize_particles()
        
        # Iterates through every particle in the particle cloud
        for particle in self.particle_cloud:

            # initialize the error and count variables
            # error is the difference between where the obsticles are to the robot 
            # compared to where they are to the particle 
            # count is the number of scans for each particle. 
            # We reward particles that have more data 
            error_sum = 0
            count = 0

            # so we iterate through each scan that is real 
            for r_val, theta_val in zip(r, theta):
                if math.isfinite(r_val):
                    # transpose the scan onto the particle 
                    angle = particle.theta + theta_val
                    new_x = particle.x + r_val * math.cos(angle)
                    new_y = particle.y + r_val * math.sin(angle)

                    # By transposing the scan, where the laser collides with an obstacle, 
                    # we should be ON an obstacle from the robot perspective. Following, 
                    # the closer we are to an obstacle from the transposed scan on the 
                    # point, the more likely the particle is close to the robot. By 
                    # transposing over each scan and summing the distance squared, we 
                    # can get a really good idea of closely the particle matches with the 
                    # robot scan. 
                    dist = self.occupancy_field.get_closest_obstacle_distance(new_x, new_y)
                    if np.isfinite(dist):
                        error_sum += dist**2
                        # we also add to the count for each scan 
                        count += 1

            # if we have at least one scan, we can continue. otherwise, we just set the weight 
            # to be very very small 
            if count > 0:
                # Gaussian likelihood — high when mean_error small
                particle.w = (math.exp(- (error_sum / count) / (0.5)))**2
            else:
                particle.w = 1e-6  # fallback for weird scans


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            We do this by scaling each particles position by it's normalized weight. 
            Notably, we calculate heading using sin and cos rather than just summing
            over theta in order to avoid over/under heading

            Arguments: 
                - None 

            Returns:
                - None 
        """
        # First, make sure that the particle weights are normalized. 
        self.normalize_particles()

        # just to get started we will fix the robot's pose to always be at the origin
        # Initializes empty variables to hold the particles x, y and theta for the next robot guess. 
        x=0
        y=0
        theta = 0

        # Goes through everything and weighs each particle relative position information. 
        for particle in self.particle_cloud:
            x += particle.x * particle.w
            y += particle.y * particle.w
            sin_sum += math.sin(particle.theta) * particle.w
            cos_sum += math.cos(particle.theta) * particle.w

        theta = math.atan2(sin_sum, cos_sum)

        # Converts the theta angle measurement from 2D frame to 3 quadernion. 
        quaternion = quaternion_from_euler(0,0, theta)

        # We create a pose with Pose and Quaternion objects. 
        self.robot_pose = Pose(position=Point(x=x, y=y, z=0.0), 
                               orientation = Quaternion(x=quaternion[0], y = quaternion[1], z = quaternion[2], w = quaternion[3]))
        
        
        # Lastly, we update the map to odom transform using the new estimated pose and odom data. 
        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")


    def resample_particles(self):
        """ Resample the particle cloud. We keep 60% of particles using the 
            draw_random_sample function, then we resample 10% of particles 
            based on a gaussian distribution around the kept particles, and the 
            resample the remaining particles (in this case, 30%) to be totally random
            using the random_particle function. 

            Arguments:
                - None

            Returns:
                - None
        """
        self.normalize_particles()

        #Find the total number of particles
        n_total = self.n_particles

        # Split particles into different groups for resampling - here 60% 
        # is kept, 10% is for gaussian distribution, 30% is random
        n_keep = int(0.6 * n_total)
        n_gaussian = int(0.1 * n_total)
        n_random = n_total - n_keep - n_gaussian

        #Creates a random number generator for use in the Gaussian
        rng = np.random.default_rng()
        #Creates an empty list fo particle weights
        weights = [p.w for p in self.particle_cloud]
        # Filters our n_keep as 60% of total particles as kept, 
        # being more likley to select particles with a higher weight
        kept_particles = draw_random_sample(self.particle_cloud, weights, n_keep)

        # Creates an empty list for gaussian particles than 
        # iterates through a loop n_guassian amount of times (30)
        gaussian_particles = []
        for _ in range(n_gaussian):
            # Picks one 'parent' particle randomly from the kept particles list, with 
            # higher weighted particles being more likely to be picked
            parent = rng.choice(kept_particles)
            #Randomly picks a new particle pose from a small radius around each particle
            x, y = rng.multivariate_normal([parent.x, parent.y], [[0.5, 0.0], [0.0, 0.5]])
            #Adds a bit of randomness to the new theta as well
            theta = parent.theta + rng.normal(-0.05, 0.05)
            # Add the particle we just generated to the gaussian list, 
            # setting it's weight to 1 - this doesn't really matter as 
            # it'll be immediatley updated in our loop
            gaussian_particles.append(Particle(x, y, theta, w=1.0))

        #Populates a list with the random poses
        random_particles = [self.random_particle() for _ in range(n_random)]
        #Combines all of the lists into a new particle cloud
        self.particle_cloud = kept_particles + gaussian_particles + random_particles
        #Adds some stochasitic randomness to every particle and normalizes all particles
        self.add_noise(self.particle_cloud)
        self.normalize_particles()


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)
        

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud. In a gaussian distribution around 
            the initial pose estimate (given by xy_theta). 

            Arguments:
                -  xy_theta (tuple): a tuple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used 
            Returns: 
                - None 
        """

        # Check if we have xy_theta 
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)

        # Make sure our particle cloud is empty and a list 
        self.particle_cloud = []

        # xy_theta is a tuple that hax (x, y, theta)
        # we want to initialize a list of particles in self.particle_cloud
        # each particle takes in an x, y, theta, and weight 
        # when we initalize, we just want to set x, y, and theta, weight can just be 1/n_particles

        # initialize a 2D gaussian distribution with a size of n_particles
        center = (xy_theta[0], xy_theta[1])
        cov = np.array([[3.0, 0.0], [0.0, 3.0]])
        rng = np.random.default_rng()
        gaussian_dist = rng.multivariate_normal(center, cov, size=self.n_particles)

        # Add all our particles in the 2D gaussian distribution, give them a 
        # random heading between 0 and 2pi 
        for x, y in gaussian_dist:
            self.particle_cloud.append(Particle(x, y, np.random.uniform(0, 2 * np.pi), w=1.0/self.n_particles))
                
        # Estimate the robot pose based on this original gaussian distribution.  
        self.update_robot_pose()


    def normalize_particles(self):
        """ Normalizes the weights of the particles to add up to 1. 
            First sums all of the weights to find the total weight, then 
            divides each individual weight by the total. 
        
        Arguments: 
            - None 
        Returns: 
            - None 
        """
        # let's just do a basic normalization for now. will probably want to switch to 
        # a better normalization if we end up weighting our particles differently 

        # start by just summing the weights
        total_weight = 0
        for particle in self.particle_cloud:
            total_weight += particle.w

        # then divide each individual weight by the total weight so they sum to 1
        for particle in self.particle_cloud:
            particle.w /= total_weight


    def publish_particles(self, timestamp):
        """ The publisher for our particle cloud. 

            Arguments: 
                - timestamp (builtin_interfaces.msg.Time), has two integers that give a time 
            Returns: 
                - None
        """
        # set the message as a particle cloud message object 
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        # we iterate through each particle and create the pose and weight based 
        #   on the x, y, theta, w of each particle 
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)


    def scan_received(self, msg):
        """ Callback function for the LaserScan subscriber. 
            Function that sets the LaserScan timestamp, or updates the scan_to_process
            if we finished processing our last scan. 

            Aguments: 
                - msg (sensor_msgs.msg.LaserScan): The received LaserScan message.
            Returns: 
                - None
        """
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg


    def add_noise(self, particle_list):
        """Add a bit of random noise to each particle.
        This updates the particle by slightly changing positions (x, y) 
        and orientation (theta) using noise.

        Arguments:
            - particle_list (list): The list of particles to modify.

        Returns:
            - None
        """

        # initialize an array of zeros the size of the particle list, 
        # then add random noise to each cell of the array
        samples = np.zeros((len(particle_list), 3))
        samples[:, :2] = np.random.normal(0, 0.05, (len(particle_list), 2))
        samples[:, 2]  = np.random.normal(0, 0.05, len(particle_list))

        # add noise to each of the particles 
        for i, particle in enumerate(particle_list):
            particle.x += samples[i, 0]
            particle.y += samples[i, 1]
            particle.theta += samples[i, 2]


    def random_particle(self):
        """ Generate a random particle anywhere within the bounds of the map.
            Also tests to make sure the new point is not inside an obstacle. 

            Arguments:
                - None

            Returns:
                - Particle: a new particle somewhere random on the map
        """
        # get the bounds of the map from the occupancy_field
        (x_bounds, y_bounds) = self.occupancy_field.get_obstacle_bounding_box()
        lower_x, upper_x = x_bounds
        lower_y, upper_y = y_bounds

        # then keep trying to generate a particle until we get one 
        # thats not inside an obstacle 
        while True:
            x = np.random.uniform(lower_x, upper_x)
            y = np.random.uniform(lower_y, upper_y)
            theta = np.random.uniform(0, 2 * np.pi)

            distance = self.occupancy_field.get_closest_obstacle_distance(x, y)
            if np.isfinite(distance):
                return Particle(x, y, theta)




def main(args=None):
    """
    Start up the particle filter node and spin until shutdown.
    """
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
