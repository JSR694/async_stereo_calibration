#!/usr/bin/env python

################################################################################
#  Captures keypress events and publishes them.
#  Starts publishing upon receiving a service request.
################################################################################

import roslib
import rospy
import genpy
import numpy as np
from cv2 import putText, imshow, waitKey, namedWindow, FONT_HERSHEY_SIMPLEX, WINDOW_NORMAL
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerResponse

class KeypressPublisher(object):
    def __init__(self, topic_name = "keypresses"):
        self.topic_name = topic_name 

        self.activated = False #until srv request received

        self.img = np.ones((500,500,3))
        self.img[:] = (250,150,100)
        s = "Press any key"
        putText(self.img, s,
                (0,self.img.shape[0]/3),
                FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 4)
        putText(self.img, s, 
                (0,self.img.shape[0]/3),
                FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 3)
        s = "to capture a"
        putText(self.img, s,
                (0,self.img.shape[0]/3 + 100),
                FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 4)
        putText(self.img, s, 
                (0,self.img.shape[0]/3 + 100),
                FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 3)
        s = "frame."
        putText(self.img, s,
                (0,self.img.shape[0]/3 + 200),
                FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 4)
        putText(self.img, s, 
                (0,self.img.shape[0]/3 + 200),
                FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 3)
            
        # OpenCV window init:
        self.win_name = "Camera Calibrator" 
        namedWindow(self.win_name, WINDOW_NORMAL)

        # ROS stuff init:
        rospy.init_node('keypress_publisher')
        self.s = rospy.Service("publish_keypresses", Trigger, self.activate)
        # Publishes val of keypresses received by cv2.waitkey:
        self.pub = rospy.Publisher(topic_name,Int32,
                queue_size = 5)
        print "Keypress publsiher constructor finished"
        self.main_loop()

    def activate(self, msg):
        self.activated = True
        return TriggerResponse(True, "hey there")

    def main_loop(self):
        print "Waiting for key presses."
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.activated:
                continue
            imshow(self.win_name, self.img)
            keycode = waitKey(5)
            if keycode != -1:
                self.pub.publish(keycode)
            rate.sleep()

if __name__ == "__main__":
    go = KeypressPublisher()
