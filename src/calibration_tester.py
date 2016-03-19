#!/usr/bin/env python

################################################################################
#
#   Starts apriltag detection
#   Calculates and publishes useful metrics on calibration:
#       e.g. error in calibration via euclid distance in apriltag location.
#
################################################################################

import roslib
import rospy
import tf

class StereoCalibTester(object):
    def __init__(self, cam1_frame_name, cam2_frame_name,
           cam1_topic_name, cam2_topic_name):
        # TF is FROM cam1 TO cam2:
        self.cam1_frame = cam1_frame_name
        self.cam2_frame = cam2_frame_name

        # ROS stuff init:
        self.bridge = CvBridge()
        # TODO Switch to subscribe to Apriltags pose,
        #  ALSO add subscribers for Apriltags detections imgs!
        self.cam1_sub = rospy.Subscriber(self.cam1_topic_name,
                Image, self.timestamp_register_cam1, queue_size=1)
        self.cam2_sub = rospy.Subscriber(self.cam2_topic_name,
                Image, self.timestamp_register_cam2, queue_size=1)
        # TODO callbacks for Apriltag pose subscribers.
        # Republish camera images as OpenCV images, to be displayed by 
        #   another node:
        self.cam1_pub_topic = self.cam1_topic_name + "_cv_img"
        self.cam2_pub_topic = self.cam2_topic_name + "_cv_img"
        self.cam1_pub = rospy.Publisher(self.cam1_pub_topic,Image,
                queue_size = 3)
        self.cam2_pub = rospy.Publisher(self.cam2_pub_topic,Image,
                queue_size = 3)
        # TODO Error metric publishers:
        # Listens for user keypresses:
        self.keypress_sub = rospy.Subscriber('img_keypress', Int32,
                self.handle_keypress, queue_size = 5)
 

        self.br = tf.TransformBroadcaster()
        self.s = rospy.Service('set_stereo_tf', SetStereoTF, self.update_tf)

        self.tf_R, self.tf_T = None, None

        self.main_loop()

    def handle_keypress(self):
        return

    def main_loop(self):
        # TODO publish error metrics between apriltag poses:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            br.sendTransform(self.tf_T, self.tf_R, rospy.Time.now(),
                    self.cam1_frame, self.cam2_frame)
            rate.sleep()

#TODO error metric for rotation distance:
'''
   -- Get quaternion transform between the poses.
   -- Calculate angle along shortest path:
     tfScalar s = tfSqrt(length2() * q.length2());
     tfAssert(s != tfScalar(0.0));
     if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
             return tfAcos(dot(-q) / s) * tfScalar(2.0);
     else 
             return tfAcos(dot(q) / s) * tfScalar(2.0);

^^SOURCE:http://docs.ros.org/jade/api/tf/html/c++/Quaternion_8h_source.html
'''

#TODO error metric for translation distance:  just Euclidean distance.
'''
    -- get tf between poses.
    -- calculate euclidean dist.
'''


if __name__ == "__main__":
    pubby = StereoTFPublisher("bumblebee", "xtion")
