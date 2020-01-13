#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    PoseArray,
    Pose,
    Point,
    Quaternion,
)
from nav_msgs.srv import GetMap
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import (
    euler_from_quaternion,
    rotation_matrix,
    quaternion_from_matrix,
)

# from random import gauss


import math
import time

import numpy as np

# import scipy
from numpy.random import random_sample

# from sklearn.neighbors import NearestNeighbors
# from sklearn.svm import OneClassSVM
# from occupancy_field import OccupancyField

from utils import convert_pose_to_xy_and_theta
from particle import Particle


class ParticleFilterHandler(object):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            n_particles: the number of particles in the filter
            pose_pub: a publisher for the pose of the robot to reset particles
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """

    def __init__(self):
        self.initialized = False

        self.base_frame = "base_link"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame

        self.n_particles = 300  # the number of particles to use
        self.p_lost = (
            .4
        )  # The probability given to the robot being "lost" at any given time
        self.outliers_to_keep = int(
            self.n_particles * self.p_lost * 0.5
        )  # The number of outliers to keep around

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        # self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        # publish the current particle cloud.  This enables viewing particles in rviz.
        # self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.particle_sub = rospy.Subscriber(
            "particlecloud", PoseArray, self.scoring_particles
        )
        self.pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1
        )

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []

        # Make a ros service call to the /static_map service to get a nav_msgs/OccupancyGrid map.
        # Then use OccupancyField to make the map object

        robotMap = rospy.ServiceProxy("/static_map", GetMap)().map
        # self.occupancy_field = OccupancyField(robotMap)
        # print "OccupancyField initialized", self.occupancy_field

        self.initialized = True

    def generate_uniformly_on_map_with_ros(self):

        self.service = rospy.ServiceProxy("global_localization", Empty)
        self.service()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)

    def reset_particle_cloud(self, pose):
        """ reset the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around. """
        self.pose_pub.publish(pose)

    def reset_particle_cloud_simple(
        self, x, y, theta, x_var=0.5, y_var=0.5, theta_var=math.pi / 6
    ):
        """Resets the particle cloud.
        
        Arguments:
            x {float} -- 
            y {float} -- 
            theta {float} -- 
            x_var {float} -- Variance on the x axis
            y_var {float} -- Variance on the y axis
            theta_var {float} -- Variance on the theta rotation (around the z axis)
        """
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = self.map_frame
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.position.z = 0
        (
            p.pose.pose.orientation.x,
            p.pose.pose.orientation.y,
            p.pose.pose.orientation.z,
            p.pose.pose.orientation.w,
        ) = tf.transformations.quaternion_from_euler(0, 0, theta)
        p.pose.covariance[6 * 0 + 0] = x_var
        p.pose.covariance[6 * 1 + 1] = y_var
        p.pose.covariance[6 * 5 + 5] = theta_var

        self.pose_pub.publish(p)

    def initialize_particle_cloud(self, xy_theta):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is ommitted, the odometry will be used """

        self.particle_cloud = []

        linear_variance = 0.5  # meters
        angular_variance = 4

        xs = np.random.normal(xy_theta[0], linear_variance, size=self.n_particles)
        ys = np.random.normal(xy_theta[1], linear_variance, size=self.n_particles)

        if xy_theta[2] == None:
            thetas = np.random.vonmises(
                xy_theta[2], angular_variance, size=self.n_particles
            )
        else:
            thetas = np.full((self.n_particles), xy_theta)

        self.particle_cloud = [
            Particle(x=xs[i], y=ys[i], theta=thetas[i])
            for i in xrange(self.n_particles)
        ]

        self.publish_particles()

    def scoring_particles(self, msg):
        """ Callback function to scoring the particles, mean, variance of weights of the particles. """
        poseArray = msg.poses
        # print poseArray
        rospy.logdebug("number of particles : %s", str(len(poseArray)))

    def publish_particles(self):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(
            PoseArray(
                header=Header(stamp=rospy.Time.now(), frame_id=self.map_frame),
                poses=particles_conv,
            )
        )


if __name__ == "__main__":
    rospy.loginfo("starting....")
    rospy.init_node("particle_filter_handler", anonymous=False)
    particle_filter_handler = ParticleFilterHandler()

    rospy.loginfo("generate particles uniformly...")

    particle_filter_handler.generate_uniformly_on_map_with_ros()

    rospy.sleep(3)

    rospy.loginfo("generate particles as position...")

    p = PoseWithCovarianceStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "map"
    p.pose.pose.position.x = -1.2
    p.pose.pose.position.y = -1.0
    p.pose.pose.position.z = 1.0
    (
        p.pose.pose.orientation.x,
        p.pose.pose.orientation.y,
        p.pose.pose.orientation.z,
        p.pose.pose.orientation.w,
    ) = tf.transformations.quaternion_from_euler(0, 0, 1.0)
    p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
    p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
    p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

    particle_filter_handler.reset_particle_cloud(p)

    rospy.sleep(1)
    rospy.spin()