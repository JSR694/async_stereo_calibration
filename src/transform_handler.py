#!/usr/bin/env python

#TODO doesn't roslaunch (ros can't find this node).
#TODO load parameters.

################################################################################
#
#   Publishes the static transform from cam1 --> cam2.
#
################################################################################

import numpy as np
import roslib
import rospy
import genpy
import tf
from async_stereo_calibration.srv import SetStereoTF

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
                print "sending tf! ", self.tf_T, self.tf_R
                self.br.sendTransform(self.tf_T, self.tf_R, rospy.Time.now(),
                        self.cam1_frame, self.cam2_frame)
            rate.sleep()

    def update_tf(self, req): 
        # Must convert from msgs to tuples:
        self.tf_T = (req.tf.position.x, req.tf.position.y, req.tf.position.z)
        q = req.tf.orientation
        self.tf_R = (q.x, q.y, q.z, q.w)
        mate = tf.transformations.quaternion_matrix(self.tf_R)
        self.tf_T = -np.dot(mate.T[0:3,0:3], self.tf_T)
        self.tf_R = tf.transformations.quaternion_from_matrix(mate.T)
        print "Transform handler received new tf.  Publishing..."

if __name__ == "__main__":
    rospy.init_node("stereo_tf_publisher")
    pubby = StereoTFPublisher("xtion_link","bumblebee")
