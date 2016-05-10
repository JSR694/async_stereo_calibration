#!/usr/bin/env python

"""
Wrapper client class that communicates with the apriltags_ros package to
store tag information (images, pose, corners).
Based on code by Alex Zhu (alexzhu(at)seas.upenn.edu).
Adapted for general use by Jack Rasiel (jrasiel(at)haverford.edu).
"""

import roslib
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
import tf
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import *
from apriltags_ros.msg import AprilTagDetectionArray
#from utility import *
from datetime import datetime

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Point32,
    Quaternion,
)

class AprilTagClient(object):
    """
    Wrapper client class that communicates with the apriltags_ros package to 
    store tag information (images, pose, corners).
    """
    def __init__(self):
        self.cam2_image = None
        self.cam1_image = None

        self._bridge = CvBridge()
        self._num_saved=0

        # ROS Pubs and Subs
        rospy.Subscriber("/usb_cam1/image_raw",Image,self.__get_cam1_image)
        rospy.Subscriber("/usb_cam2/image_raw",Image,self.__get_cam2_image)
        self.listener = tf.TransformListener()
        cv2.namedWindow("Cam1",0)
        cv2.namedWindow("Cam2",0)
        
        self.save_dir = "/tmp/"

        tstamp = datetime.now().strftime("%Y-%m-%d-%y-%H-%M")
        img_dir = self.save_dir + "imgs_" + tstamp
        if not os.path.exists(img_dir):
            os.mkdir(img_dir)
        self.save_dir = img_dir


    def __get_cam1_image(self,image):
        """
        # Dummy function to save incoming images
        """
        try:
            cv_image=self._bridge.imgmsg_to_cv2(image,"bgr8")
            self.cam1_image=cv_image
        except CvBridgeError, e:
            print e

    def __get_cam2_image(self,image):
        """
        # Dummy function to save incoming images
        """
        try:
            cv_image=self._bridge.imgmsg_to_cv2(image,"bgr8")
            self.cam2_image=cv_image
        except CvBridgeError, e:
            print e

    def show_image(self):
        if self.cam1_image is None or self.cam2_image is None:
            return
        try:
            cv2.imshow("Cam1",self.cam1_image)
            cv2.imshow("Cam2",self.cam2_image)
            user_input=cv2.waitKey(50)
        except:
            user_input=-1
            print "failed to load image"
            return
            
        # Key code for 'ENTER'
        if user_input == 1048586:
            cv2.imwrite(self.save_dir + '/cam1_{0}.png'.format(self._num_saved), self.cam1_image)
            print "Successfully wrote " + self.save_dir + '/cam1_{0}.png'.format(self._num_saved) + "to file."
            cv2.imwrite(self.save_dir + '/cam2_{0}.png'.format(self._num_saved), self.cam2_image)
            print "Successfully wrote " + self.save_dir + '/cam2_{0}.png'.format(self._num_saved) + "to file."
            self._num_saved=self._num_saved+1	

def main(args):
    rospy.init_node('save_images')
    client = AprilTagClient()
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        client.show_image()
        r.sleep()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
