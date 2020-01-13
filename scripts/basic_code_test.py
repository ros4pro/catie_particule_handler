#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseWithCovariance,Pose,Point,Quaternion
from std_srvs.srv import Empty
import time
import tf
import math
# from joy.msg import Joy

class catie_Localization:

    def __init__(self): 

        # rospy.Subscriber('/joy', Joy, self.joy_cb)

        self.service = rospy.ServiceProxy('global_localization', Empty)
        self.setpose_pub = rospy.Publisher("initialpose",PoseWithCovarianceStamped,latch=True, queue_size=1)

        self.setted_pose = {'x':0,'y':0,'a':0}
        self.test_set_pose_flag = True
        self.test_set_pose_cnt = 3


    def gen_particles_uniform(self):
        self.service()


    # def reset_particle_1(self,x=None,y=None,theta=None,covariance=None):
    #     pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
    #     rospy.loginfo("Setting Pose")

    #     p   = PoseWithCovarianceStamped()
    #     msg = PoseWithCovariance()
    #     msg.pose = Pose(Point(1.8, -0.953, 0.000), Quaternion(0.000, 0.000, -0.0149, 0.9999))
    #     msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 
    #                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    #                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    #                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    #     p.pose = msg
    #     pub.publish(p)

    def reset_particle_2(self,covariance=0):
        # Define a set inital pose publisher.
        rospy.loginfo("start set pose...")
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.pose.position.x = self.setted_pose['x']
        p.pose.pose.position.y = self.setted_pose['y']
        p.pose.pose.position.z = self.setted_pose['a']
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, self.setted_pose['a'])
        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

        self.setpose_pub.publish(p)
         
    
    #def robot_is_kinapped(self):
        

# if __name__ == "__main__":

#     rospy.init_node('catie_Localization')   
#     rospy.loginfo('starting....')
#     catie_Localization = catie_Localization()    

#     rospy.loginfo("start test inital pose...")

#     while catie_Localization.test_set_pose_flag == True:
#         catie_Localization.reset_particle_2()
#         catie_Localization.test_set_pose_cnt -= 1
#         catie_Localization.setted_pose = {'x':3.2,'y':4,'a':0}
#         if catie_Localization.test_set_pose_cnt == 0:
#             catie_Localization.test_set_pose_flag = False
#     rospy.sleep(1)
#     rospy.spin()