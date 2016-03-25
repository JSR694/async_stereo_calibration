#!/usr/bin/env python

################################################################################
#
#   Starts apriltag detection.
#   Calculates and publishes useful metrics on calibration:
#       e.g. error in calibration via euclid distance in apriltag location.
#
################################################################################

from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Int32
import numpy as np
import roslib
import rospy
import tf


### Some useful geometric functions:
# Quaternion dot product:
def quat_dot(q1, q2):
    return np.dot(np.array((q1.x,q1.y,q1.z,q1.w)),
            np.array((q2.x,q2.y,q2.z,q2.w)))
# Just np acos but doesn't throw an error for |x|>1:
def acos(x):
    if x > 1: x = 1
    elif x < -1: x = -1
    return np.math.acos(x)

# Get axis angle form from quaternion:
def axis_angle_from_quat(q):
    angle = 2*acos(q.w)
    s = np.sqrt(1-q.w*q.w)
    if s < 0.001:
        return q.x, q.y, q.z, s
    else:
        return q.x/s, q.y/s, q.z/s, s

# Returns angle between two quaternions along the shortest path.
# Based on tf:Quaternion::angleShortestPath()
#   See docs.ros.org/jade/api/tf/html/c++/Quaternion_8h_source.html
def get_angle_shortest_path(q1, q2):
    # Get sqrt of product of squared lengths (dot(q,q)) of each quaternion:
    s = np.sqrt(quat_dot(q1,q1), quat_dot(q2,q2))
    if s == 0.0:
        # TODO raise fatal error if s == 0.
        print "BAD THING!  s = 0."
        raise Exception

    # Take care of long angle case (see en.wikipedia.org/wiki/Slerp):
    if quat_dot(q1,q2) < 0:
        return acos(quat_dot(-q2 / s)) * 2.0
    else:
        return acos(quat_dot(q2 / s)) * 2.0


class StereoCalibTester(object):
    def __init__(self, cam1_frame_name, cam2_frame_name,
           cam1_tag_topic_name, cam2_tag_topic_name):
        # TF is FROM cam1 TO cam2:
        self.cam1_frame = cam1_frame_name
        self.cam2_frame = cam2_frame_name
        self.cam1_tag_topic_name = cam1_tag_topic_name
        self.cam2_tag_topic_name = cam2_tag_topic_name
        # TODO set topic names based on params:
        self.trans_error_topic = "trans_error"
        self.rot_error_topic = "rot_error"

        # Pose msgs for latest tag detections:
        self.cam1_tag_pose = None
        self.cam2_tag_pose = None
        self.tag_reprojected = None

        # ROS stuff init:
        rospy.init_node("nodey") #TODO lol
        self.listener = tf.TransformListener()
        # Activate once the cam1 --> cam2 tf has been published.
        print "Waiting for tf between frames: " + self.cam1_frame, self.cam2_frame
        # TODO better way to make the timeout infinite?
        #self.listener.waitForTransform(self.cam1_frame,self.cam2_frame,
                #rospy.Time.now(), rospy.Duration(9*10^10))
        print "Found cam1 --> cam2 TF! Starting error metric publishing:"
        self.cam1_to_cam2 = self.listener.lookupTransform('bumblebee','xtion', rospy.Time(0))

        # Subscribers for Apriltag detection poses:
        self.cam1_tag_pose_sub = rospy.Subscriber(self.cam1_tag_topic_name,
                PoseArray, self.handle_cam1_tag_pose, queue_size=3)
        self.cam2_tag_pose_sub = rospy.Subscriber(self.cam2_tag_topic_name,
                PoseArray, self.handle_cam2_tag_pose, queue_size=3)
        self.trans_error_pub = rospy.Publisher(self.trans_error_topic, Int32,
                queue_size = 10)
        self.rot_error_pub = rospy.Publisher(self.rot_error_topic, Int32,
                queue_size = 10)

        self.main_loop()

    # Calculate and publish error metrics at 10hz:
    def main_loop(self):
        rate = rospy.Rate(10)
        print "hit main loop"
        while not rospy.is_shutdown():
            if (self.cam1_tag_pose is not None and
                    self.cam2_tag_pose is not None and
                    self.tag_reprojected is not None):
                print "Calculating error metric!"
                rot_error_msg = Int32(self.get_orientation_error())
                trans_error_msg = Int32(self.get_orientation_error())
                self.rot_error_pub.publish(rot_error_msg)
                self.trans_error_pub.publish(trans_error_msg)
            else:
                print "Waiting for initialization of all tag poses..."
                print "\t", self.cam1_tag_pose
                print "\t", self.cam2_tag_pose
                print "\t" , self.tag_reprojected
            rate.sleep()
                       
    # Convert ROS Quaternion msg to a 4-tuple (x,y,z,w):
    def quat_msg_to_tuple(self, q):
        return q.x,q.y,q.z,q.w

    # Callback for new tag pose.  Recalculate tag_projected.
    def handle_cam1_tag_pose(self, msg):
        print "GOT TAG IN CAM1!"
        if len(msg.poses) > 0:
            self.cam1_tag_pose = msg.poses[0]
            ### Recalculate tag_projected:
            # Transform position and orientation:
            q = tf.transformations.quaternion_multiply(
                    self.quat_msg_to_tuple(self.cam1_to_cam2.orientation),
                    self.quat_msg_to_tuple(self.cam1_tag_pose.orientation))
            t = Point(self.cam1_to_cam2.position.x + self.cam1_tag_pose.position.x,
                    self.cam1_to_cam2.position.y + self.cam1_tag_pose.position.y,
                    self.cam1_to_cam2.position.z + self.cam1_tag_pose.position.z)
            self.tag_reprojected = Pose(t, Quaterion(q[0],q[1],q[2],q[3]))

    def handle_cam2_tag_pose(self, msg):
        print "GOT TAG IN CAM2!"
        if len(msg.poses) > 0:
            self.cam2_tag_pose = msg.poses[0]

    # Returns metric for error between pose of the apriltag seen by cam1, 
    #   and pose of the same tag when seen by cam2 and transformed into 
    #   cam1's frame.
    def get_orientation_error(self, metric = 'AXIS_ANGLE'):
        if method == 'AXIS_ANGLE':
            # Convert to axis-angle and compute 2-norm:
            x1,y1,z1,s1 = axis_angle_from_quat(self.cam2_tag_pose.orientation)
            x2,y2,z2,s2 = axis_angle_from_quat(self.tag_reprojected.orientation)
            return np.linalg.norm(np.array([x1-x2, y1-y2, z1-z2, s1-s2]))
        elif method == 'ANGLE_SHORTEST_PATH':
            # Get angle along shortest path between quaternions:
            return get_angle_shortest_path(self.tag_reprojected.orientation, 
                    self.cam2_tag_pose.orientation)

    # Returns euclidean distance between two poses:
    def get_position_error(self):
        t1, t2 = self.cam2_tag_pose.Position, self.tag_reprojected.Position
        return np.linalg.norm(np.array([t1.x-t2.x,t1.y-t2.y,t1.x-t1.z]))

def start_calib_tester():
    # TODO load params and init calib tester object.
    cam1_frame_name = "bumblebee"
    cam2_frame_name = "xtion"
    cam1_topic_name = "/cam1/tag_detections_pose"
    cam2_topic_name = "/cam2/tag_detections_pose"
    StereoCalibTester(cam1_frame_name, cam2_frame_name,
           cam1_topic_name, cam2_topic_name)



if __name__ == "__main__":
    start_calib_tester()
