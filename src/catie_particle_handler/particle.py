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


class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, turn_multiplier=1.45, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y
        # self.turn_multiplier = turn_multiplier

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        return Pose(
            position=Point(x=self.x, y=self.y, z=0),
            orientation=Quaternion(
                x=orientation_tuple[0],
                y=orientation_tuple[1],
                z=orientation_tuple[2],
                w=orientation_tuple[3],
            ),
        )

    def generate_uniformly_on_map(self, map):
        """
        Reinitialize the current particle to be at some random location and orientation on the provided map.
        Args:
            map (OccupancyGrid): The map to generate a random pose on
        Returns:
            self (Particle)
        """
        minx = map.info.origin.position.x
        miny = map.info.origin.position.y
        maxx = minx + map.info.width * map.info.resolution
        maxy = minx + map.info.height * map.info.resolution

        x = float(np.random.uniform(minx, maxx))
        y = float(np.random.uniform(miny, maxy))

        theta = float(np.random.uniform(-np.pi, np.pi))

        self.__init__(x, y, theta)
        return self

    # def lost_particles(self):
    #     """ lost_particles predicts which paricles are "lost" using unsupervised outlier detection.
    #         In this case, we choose to use Scikit Learn - OneClassSVM
    #     Args:
    #     Returns:
    #         inliers = particles that are not lost
    #         outlier = particles that are lost
    #     """
    #     # First format training data
    #     x = [p.x for p in self.particle_cloud]
    #     y = [p.y for p in self.particle_cloud]
    #     X_train = np.array(zip(x, y))

    #     # Next make unsupervised outlier detection model
    #     # We have chosen to use OneClassSVM
    #     # Lower nu to detect fewer outliers
    #     # Here, we use 1/2 of the lost probability : self.p_lost / 2.0
    #     clf = OneClassSVM(nu=.3, kernel="rbf", gamma=0.1)
    #     clf.fit(X_train)

    #     # Predict inliers and outliers
    #     y_pred_train = clf.predict(X_train)

    #     # Create inlier and outlier particle lists
    #     inliers = []
    #     outliers = []

    #     # Iterate through particles and predictions to populate lists
    #     for p, pred in zip(self.particle_cloud, y_pred_train):
    #         if pred == 1:
    #             inliers.append(p)
    #         elif pred == -1:
    #             outliers.append(p)

    #     return inliers, outliers

    # def resample_particles(self):
    #     """ Resample the particles according to the new particle weights.
    #         The weights stored with each particle should define the probability that a particular
    #         particle is selected in the resampling step.  You may want to make use of the given helper
    #         function draw_random_sample.
    #     """
    #     # TODO: Dynamically decide how many particles we need

    #     # make sure the distribution is normalized
    #     self.normalize_particles()

    #     # Calculate inlaying and exploring particle sets
    #     inliers, outliers = self.lost_particles()
    #     desired_outliers = int(self.n_particles * self.p_lost)
    #     desired_inliers = int(self.n_particles - desired_outliers)

    #     # Calculate the average turn_multiplier of the inliers
    #     mean_turn_multipler = np.mean([p.turn_multiplier for p in inliers])
    #     print "Estimated turn multiplier:", mean_turn_multipler

    #     # Recalculate inliers
    #     probabilities = [p.w for p in self.particle_cloud]
    #     new_inliers = self.draw_random_sample(self.particle_cloud, probabilities, desired_inliers)

    #     # Recalculate outliers
    #     # This keeps some number of outlying particles around unchanged, and spreads the rest randomly around the map.
    #     if desired_outliers > min(len(outliers), self.outliers_to_keep):
    #         outliers.sort(key=lambda p: p.w, reverse=True)

    #         num_to_make = desired_outliers - min(len(outliers), self.outliers_to_keep)

    #         new_outliers = outliers[:self.outliers_to_keep] + \
    #                        [Particle().generate_uniformly_on_map(self.occupancy_field.map) for _ in xrange(num_to_make)]
    #         for p in new_outliers:
    #             p.turn_multiplier = mean_turn_multipler
    #     else:
    #         new_outliers = outliers[:desired_outliers]

    #     # Set all of the weights back to the same value. Concentration of particles now reflects weight.
    #     new_particles = new_inliers + new_outliers

    #     for p in new_particles:
    #         p.w = 1.0
    #         p.turn_multiplier = np.random.normal(p.turn_multiplier, 0.1)
    #     self.normalize_particles()

    #     self.particle_cloud = new_particles

    # @staticmethod
    # def draw_random_sample(choices, probabilities, n):
    #     """ Return a random sample of n elements from the set choices with the specified probabilities
    #         Args:
    #             choices: the values to sample from represented as a list
    #             probabilities: the probability of selecting each element in choices represented as a list
    #             n: the number of samples
    #         Returns:
    #             samples (List): A list of n elements, deep-copied from choices
    #     """
    #     values = np.array(range(len(choices)))
    #     probs = np.array(probabilities)
    #     bins = np.add.accumulate(probs)
    #     inds = values[np.digitize(random_sample(n), bins)]
    #     samples = []
    #     for i in inds:
    #         samples.append(deepcopy(choices[int(i)]))
    #     return samples

