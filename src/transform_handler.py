#!/usr/bin/env python

################################################################################
#
#   Publishes the static transform from bumblebee --> xtion.
#
################################################################################

import roslib
import rospy
import tf
from baxter_stereo_calibration.srv import SetStereoTF

class StereoTFPublisher(object):
    def __init__(self, cam1_frame_name, cam2_frame_name):
        # TF is FROM cam1 TO cam2:
        self.cam1_frame = cam1_frame_name
        self.cam2_frame = cam2_frame_name
        
        self.br = tf.TransformBroadcaster()
        self.s = rospy.Service('set_stereo_tf', SetStereoTF, self.update_tf)
        
        self.tf_R, self.tf_T = None, None

        self.main_loop()

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.tf_R is not None and self.tf_T is not None :
                br.sendTransform(self.tf_T, self.tf_R, rospy.Time.now(),
                        self.cam1_frame, self.cam2_frame)
            rate.sleep()

    def update_tf(self, req): 
        self.tf_T = req.tf.position
        self.tf_R = req.tf.orientation

if __name__ == "__main__":
    rospy.init_node("stereo_tf_publisher")
    pubby = StereoTFPublisher("bumblebee", "xtion")
