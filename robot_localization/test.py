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

    # TODO: define additional helper functions if needed





xy_theta = [0,0,0]
num_particles = 1000

particle_cloud = []

cov = np.array([[20, 0], [0, 20]])
center = (xy_theta[0], xy_theta[1])
rng = np.random.default_rng()



gaussian_dist = rng.multivariate_normal(center, cov, size=num_particles)

# Initialize particles around (x, y, theta)
for x, y in gaussian_dist:
    particle_cloud.append(Particle(x, y, xy_theta[2], w=1.0))

import matplotlib.pyplot as plt

plt.plot(gaussian_dist[:, 0], gaussian_dist[:, 1], '.', alpha=0.5)
plt.axis('equal')
plt.grid()
plt.show()


