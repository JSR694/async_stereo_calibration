#!/usr/bin/env python

################################################################################
#  Displays (cv2.imshow) images from two image topics.
#  TODO why is cam2 (xtion) showing up in BOTH windows?
################################################################################

import roslib
import cv2
import rospy
import genpy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CalibImageShower(object):
    def __init__(self, cam1_topic_name, cam2_topic_name):
        self.cam1_topic_name = cam1_topic_name + "_cv_img"
        self.cam2_topic_name = cam2_topic_name + "_cv_img"

        self.cam1_img, self.cam2_img = None, None
            
        # OpenCV window init:
        self.win1_name = "Cam1 (topic %s )" % self.cam1_topic_name
        self.win2_name = "Cam2 (topic %s )" % self.cam2_topic_name
        cv2.namedWindow(self.win1_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.win2_name, cv2.WINDOW_NORMAL)

        # ROS stuff init:
        print "SUBSCRIBING TO:"
        print self.cam1_topic_name
        print self.cam2_topic_name
        self.cam1_sub = rospy.Subscriber(self.cam1_topic_name, 
                Image, self.update_cam1, queue_size=1)
        self.cam2_sub = rospy.Subscriber(self.cam2_topic_name, 
                Image, self.update_cam2, queue_size=1)
        # Publishes val of keypresses received by cv2.waitkey:
        self.keypress_pub = rospy.Publisher('img_keypress',Int32,
                queue_size = 5)
        
        self.bridge = CvBridge()
        self.main_loop()
        

    def update_cam1(self, img):
        self.cam1_img = self.bridge.imgmsg_to_cv2(img)

    def update_cam2(self, img):
        self.cam2_img = self.bridge.imgmsg_to_cv2(img)

    def main_loop(self):
        rospy.init_node('calib_imshower')
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if (self.cam1_img is None or self.cam2_img is None):
                continue
            cv2.imshow(self.win1_name, self.cam1_img)
            cv2.imshow(self.win2_name, self.cam2_img)
            keycode = cv2.waitKey(5)
            if keycode != -1:
                self.keypress_pub.publish(keycode)
            rate.sleep()

if __name__ == "__main__":
    showy = CalibImageShower("cam1_feed"+"_cv_img","cam2_feed"+"_cv_img")
