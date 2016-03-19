#!/usr/bin/env python

################################################################################
# Stereo calibration
# Collects (semi-)synced image pairs of a chessboard calibration target.
#
# Some functionality based on the ROS camera_calibration package K, D)
#  Specifically, downsampling-based checkerboard detection, [others?]
# To stop gathering frames and perform R,T estimation, press "q" while
#   in either camera feed window.
#
# TODO: implement flags for rotating new frames.
################################################################################

import roslib
import math
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
import genpy
import tf
from baxter_stereo_calibration.srv import *
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from distutils.version import LooseVersion

def pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return np.linalg.norm((p1[0]-p2[0],p1[1]-p2[1]))

class BaxterStereoCalibrator(object):
    def __init__(self, cam1_topic_name, cam2_topic_name, 
            cam1_K, cam1_D, cam2_K, cam2_D, 
            cam1_do_undistort = False, cam2_do_undistort = False,
            board_dim = (8,10), board_size = .02,
            load_calib_data = False, save_calib_data = True):
        self.load_calib_data = load_calib_data
        self.save_calib_data = save_calib_data
        self.cam1_K = cam1_K
        self.cam1_D = cam1_D
        self.cam2_K = cam2_K
        self.cam2_D = cam2_D
        self.cam1_do_undistort = cam1_do_undistort
        self.cam2_do_undistort = cam2_do_undistort

        ### CONSTANTS: 
        # Max allowed time gap between two camera frames in a correspondence:
        #self.MAX_CORRESPONDENCE_LAG = genpy.Duration.from_sec(.1)
        self.MAX_CORRESPONDENCE_LAG = .05
        # Min duration between successive correspondences:
        self.MIN_DELAY_BETWEEN_CORRESPONDENCES = genpy.Duration.from_sec(1)
        # Make sure n_cols > n_rows to agree with OpenCV CB detector output:
        self.n_board_cols = max(board_dim)
        self.n_board_rows = min(board_dim)
        # Length of one chessboard square, in meters.
        #  Default (2cm) is from Alex's chessboard:
        self.CHESSBOARD_SIZE = board_size
        self.size = None #img size.  A param for cv2.stereoCalib.
        # Generate chessboard corner loc.s, in chessboard frame:
        self.CHESSBOARD = self.mk_object_points(use_board_size = True)

        self.cam1_topic_name, self.cam2_topic_name = cam1_topic_name, cam2_topic_name

        ### WHAT WE WANNA FIND:
        self.R, self.T = None, None
        self.calibrated = False
        
        ### CHECKERBOARD DETECTION VAR.S:
        self.init_time = None # Timestamp offset.
        # List of image correspondences between cam1 and cam2:
        # Each item must be of the form:
        #   ((cam1_img, cam1_corners, cam1_tstamp),
        #    (cam2_img, cam2_corners, cam2_tstamp))
        self.stereo_correspondences = []
        self.last_correspondence_time = genpy.Time(secs=0)
        # If true, checks for chessboard in next cam2 frame:
        self.need_follower_calib_frame = False 
        self.processing_follower_frames = False
        # Time when last stereo correspondence was found:
        self.last_calib_frame_time = None
        # Leader's frame in which chessboard was found:
        self.leader_calib_frame_time = None
        self.leader_calib_frame_img = None
        self.leader_calib_frame_chessboard = None
        self.follower_frame_buffer = []
        
        # Var.s for dynamic load balancing:
        self.cam1_last_frame_time = None
        self.cam2_last_frame_time = None
        # ID (1 or 2) of the "Leader" for this round of correspondence-finding.
        #   That is, the camera which first found the chessboard.
        #   The "Follower" cam is the camera whose frames we then search for a
        #       correspondence.
        self.leader_id = 1
        
        # Service for publishing the calculated transform:
        self.publish_stereo_tf = rospy.ServiceProxy('set_stereo_tf', SetStereoTF)

        # Calibrate using recorded point correspondencies, or capture new ones:
        if self.load_calib_data:
            self.cal_fromcorners(load_data = True)
        else:
            # Init node pubs and subs for checkerboard detection: 
            self.bridge = CvBridge()
            self.cam1_sub = rospy.Subscriber(self.cam1_topic_name, 
                    Image, self.timestamp_register_cam1, queue_size=1)
            self.cam2_sub = rospy.Subscriber(self.cam2_topic_name, 
                    Image, self.timestamp_register_cam2, queue_size=1)
            # Republish camera images as OpenCV images, to be displayed by 
            #   another node:
            # TODO topic naming convention too cumbersome?
            self.cam1_pub_topic = self.cam1_topic_name + "_cv_img"
            self.cam2_pub_topic = self.cam2_topic_name + "_cv_img"
            print "PUBLISHING TO:"
            print self.cam1_pub_topic
            print self.cam2_pub_topic     
            self.cam1_pub = rospy.Publisher(self.cam1_pub_topic,Image,
                    queue_size = 3)
            self.cam2_pub = rospy.Publisher(self.cam2_pub_topic,Image,
                    queue_size = 3)
            # Listens for user keypresses:
            # TODO should this be a constructor param?
            self.keypress_sub = rospy.Subscriber('img_keypress', Int32,
                    self.handle_keypress, queue_size = 5)
            
            #Try to initiate calibration using correspondencies.
            rospy.on_shutdown(self.do_cal_fromcorners)

    def do_cal_fromcorners(self):
        print "SAVE?\t", self.save_calib_data
        #Try to initiate calibration using correspondencies.
        self.cal_fromcorners(save_data = self.save_calib_data, 
                load_data = False)

    def handle_keypress(self, msg):
        keycode = msg.data
        if keycode == 1048689:
            #Try to initiate calibration using correspondencies.
            self.cal_fromcorners(save_data = self.save_calib_data, 
                    load_data = False)

    def reset_correspondence_search(self):
        print "TOTAL CORRESPONDENCES FOUND:\t", str(len(self.stereo_correspondences))
        print
        self.need_follower_calib_frame = False 
        self.leader_calib_frame_time = None
        self.leader_calib_frame_img = None
        self.leader_calib_frame_chessboard = None
        self.follower_frame_buffer = []

    def store_stereo_correspondence(self, img, corners, timestamp, dt):
        print "Storing correspondence!  dt:\t%s" % str(dt)

        leader = (self.leader_calib_frame_img,
                self.leader_calib_frame_chessboard,
                self.leader_calib_frame_time)
        follower = (img,corners,timestamp)
        
        # Make sure tuple always of form (cam1, cam2):
        if self.leader_id == 1:
            self.stereo_correspondences.append( (leader, follower) )
        else:
            self.stereo_correspondences.append( (follower, leader) )
        self.last_correspondence_time = timestamp 
        self.reset_correspondence_search()
        
    def timestamp_register_cam1(self, img_msg):
        # Save timestamp for this frame, and pass img_msg along to the switch:
        self.cam1_last_frame_time = img_msg.header.stamp
        self.callback_switch(img_msg, 1)

    def timestamp_register_cam2(self, img_msg):
        self.cam2_last_frame_time = img_msg.header.stamp
        self.callback_switch(img_msg, 2)

    def callback_switch(self, img_msg, cam):
        # If one cam's feed is ahead of the other, assign it the more
        #   computationally complex callback, to slow it down.
        # Prevents camera feeds from becoming unusably out of sync:

        # Offset printed times from initial:
        if self.init_time is None:
            self.init_time = img_msg.header.stamp
        # Wait until both cams have released a frame:
        if (self.cam1_last_frame_time == None or 
                self.cam2_last_frame_time == None):
            return

        # Swap leader+follower if current leader is lagging behind follower,
        #   and system is not currently looking for a correspondence:
        if (not self.need_follower_calib_frame and
            ((self.leader_id==1 and 
                self.cam1_last_frame_time < self.cam2_last_frame_time) or
            (self.leader_id==2 and 
                self.cam1_last_frame_time > self.cam2_last_frame_time))):
                self.leader_id = 2 if self.leader_id == 1 else 1
                #print "NEW LEADER: %s" % self.leader_id
        # Direct the image_msg to the appropriate callback:
        if cam == self.leader_id:
            self.callback_leader(img_msg, cam)
        else:
            self.callback_follower(img_msg, cam)

    def callback_leader(self,img_msg, cam):
        # The leader searches each frame received until it detects a chessboard.
        # It then saves that frame, and sets the need_follower_calib_frame flag,
        #  prompting the follower to search for a corresponding frame.

        # Ignore this frame if currently calibrating based on earlier leader frame,
        #   or not enough time passed since last correspondence:
        if (self.need_follower_calib_frame or
                img_msg.header.stamp - self.last_correspondence_time < \
                        self.MIN_DELAY_BETWEEN_CORRESPONDENCES):
            return

        # Convert img message to opencv img:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg)
            if cam == 2:
                cv_image = cv2.flip(cv_image, -1)
                if self.cam2_do_undistort:
                    cv_image = cv2.undistort(cv_image, self.cam2_K, self.cam2_D)

            if self.size is None and cam == 2:
                self.size = cv_image.shape
        except CvBridgeError as e:
            rospy.logfatal("CV_Bridge failed to convert message from %s!" 
                    % self.leader_topic_name)
            return
        # Check for chessboard calib pattern:
        scrib, corners, downsampled_corners, scales = self.downsample_and_detect(cv_image)
        # If chessboard found, save frame + chessboard data 
        #   and set flag for follower's handler to find a corresponding frame:
        if corners is not None:
            print "LEADER found chessboard!"
            print "Starting search for corresponding follower frame..."
            self.leader_calib_frame_img = cv_image
            self.leader_calib_frame_time = img_msg.header.stamp
            self.leader_calib_frame_chessboard = corners
            cv2.drawChessboardCorners(scrib,
                    (self.n_board_rows, self.n_board_cols), 
                    downsampled_corners, True)
            self.need_follower_calib_frame = True
        '''
        dt = img_msg.header.stamp-self.init_time
        dt = dt.to_sec()
        cv2.putText(cv_image, str(dt), 
                (0,cv_image.shape[0]), 
                cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 10)
        cv2.putText(cv_image, str(dt), 
                (0,cv_image.shape[0]),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (255,0,0), 3)
        win_name = self.win2_name if cam==2 else self.win1_name
        cv2.imshow(win_name,scrib)
            #Try to initiate calibration using correspondencies.
            self.cal_fromcorners()
        '''
        # Convert cv img to img msg and publish:
        if cam == 1:
            self.cam1_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, '8UC1'))
        else:
            self.cam2_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, '8UC1'))

    def callback_follower(self,img_msg, cam):
        # The follower waits for the need_follower_calib_frame is set, at which
        #  point it searches incoming frames for the chessboard.  It does this
        #  until incoming frames are timestamped AFTER the leader's calib frame,
        #  at which point the best correspondence is saved.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg)
            if cam == 2:
                cv_image = cv2.flip(cv_image, -1)
                if self.cam2_do_undistort:
                    cv_image = cv2.undistort(cv_image, self.cam2_K, self.cam2_D)

        except CvBridgeError as e:
            rospy.logfatal("CV_Bridge failed to convert message from %s!" 
                    % self.follower_topic_name)
            return
        # If leader has found chessboard, look for corresponding follower frame:
        if self.need_follower_calib_frame:
            # Store follower's frames until time difference from leader's calib
            #  frame is too great:
            # TODO nonetype error only appeared when testing live, when the
            #  prgrm had been running for >30secs without issue.  What's going on?
            if (img_msg.header.stamp is None or
                self.leader_calib_frame_time is None):
                    return
            dt = img_msg.header.stamp - self.leader_calib_frame_time
            dt = abs(dt.to_sec())
            # FUN FACT: taking the abs val of a Duration just flips its sign...
            # dt = abs(img_msg.header.stamp - self.leader_calib_frame_time)

            if dt <= self.MAX_CORRESPONDENCE_LAG:
                self.follower_frame_buffer.append(
                        (cv_image, img_msg.header.stamp, dt))
            # Find best match follower calib frame out of the buffer:
            else:
                print "\tDone collecting follower frames.  Collected %s" % \
                        str(len(self.follower_frame_buffer))
                print "\tChecking them for chessboards..."                 
                self.need_follower_calib_frame = False
                # Sort follower frames by time diff from leader:
                self.follower_frame_buffer.sort(key=lambda x: x[2])
                found_it = False
                count = 0
                for frame, timestamp, dt in self.follower_frame_buffer:
                    scrib, corners, downsampled_corners, scales = \
                            self.downsample_and_detect(cv_image)
                    if corners is not None:
                        found_it = True
                        self.store_stereo_correspondence(cv_image, 
                                corners, timestamp, dt)
                        break
                if not found_it:
                    print "\tNo calib frame found!  ABORTING SEARCH."
                    self.reset_correspondence_search()
        # Show frame:
        '''
        dt = img_msg.header.stamp-self.init_time
        dt = dt.to_sec()
        cv2.putText(cv_image, str(dt), 
                (0,cv_image.shape[0]),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 10)
        cv2.putText(cv_image, str(dt), 
                (0,cv_image.shape[0]),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (0,0,255), 3)
        '''
        if cam == 1:
            self.cam1_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, '8UC1'))
        else:
            self.cam2_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, '8UC1'))

    def downsample_and_detect(self, img):
        """
        Downsample the input image to approximately VGA resolution and detect the
        calibration target corners in the full-size image.

        Scales the corners back up to the correct size after detection.

        Returns (scrib, corners, downsampled_corners, (x_scale, y_scale)).
        """
        # Scale the input image down to ~VGA size
        height = img.shape[0]
        width = img.shape[1]
        scale = math.sqrt( (width*height) / (640.*480.) )
        if scale > 1.0:
            scrib = cv2.resize(img, (int(width / scale), int(height / scale)))
        else:
            scrib = img
        # Due to rounding, actual horizontal/vertical scaling may differ slightly
        x_scale = float(width) / scrib.shape[1]
        y_scale = float(height) / scrib.shape[0]

        # Detect checkerboard
        (ok, downsampled_corners) = self.get_chessboard_corners(scrib)
        # Scale corners back to full size image
        corners = None
        if ok:
            if scale > 1.0:
                # Refine up-scaled corners in the original full-res image
                # TODO Does this really make a difference in practice?
                corners_unrefined = downsampled_corners.copy()
                corners_unrefined[:, :, 0] *= x_scale
                corners_unrefined[:, :, 1] *= y_scale
                radius = int(math.ceil(scale))
                if len(img.shape) == 3 and img.shape[2] == 3:
                    mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                else:
                    mono = img
                cv2.cornerSubPix(mono, corners_unrefined, (radius,radius), (-1,-1),
                                              ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1 ))
                corners = corners_unrefined
            else:
                corners = downsampled_corners

        return (scrib, corners, downsampled_corners, (x_scale, y_scale))

    def get_chessboard_corners(self,img, refine = True, checkerboard_flags=0):
        """
        Use cvFindChessboardCorners to find corners of chessboard in image.
        """
        h = img.shape[0]
        w = img.shape[1]
        if len(img.shape) == 3 and img.shape[2] == 3:
            mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            mono = img
        (ok, corners) = cv2.findChessboardCorners(mono, (self.n_board_cols, self.n_board_rows), 
                flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | checkerboard_flags)
        if not ok:
            return (ok, corners)

        # If any corners are within BORDER pixels of the screen edge, reject the detection by setting ok to false
        # NOTE: This may cause problems with very low-resolution cameras, where 8 pixels is a non-negligible fraction
        # of the image size. See http://answers.ros.org/question/3155/how-can-i-calibrate-low-resolution-cameras
        BORDER = 8
        if not all([(BORDER < corners[i, 0, 0] < (w - BORDER)) and (BORDER < corners[i, 0, 1] < (h - BORDER)) for i in range(corners.shape[0])]):
            ok = False

        if refine and ok:
            # Use a radius of half the minimum distance between corners. This should be large enough to snap to the
            # correct corner, but not so large as to include a wrong corner in the search window.
            min_distance = float("inf")
            for row in range(self.n_board_rows):
                for col in range(self.n_board_cols - 1):
                    index = row*self.n_board_rows + col
                    min_distance = min(min_distance, pdist(corners[index, 0], corners[index + 1, 0]))
            for row in range(self.n_board_rows - 1):
                for col in range(self.n_board_cols):
                    index = row*self.n_board_rows + col
                    min_distance = min(min_distance, pdist(corners[index, 0], corners[index + self.n_board_cols, 0]))
            radius = int(math.ceil(min_distance * 0.5))
            cv2.cornerSubPix(mono, corners, (radius,radius), (-1,-1),
                                          ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1 ))

        return (ok, corners)

    def cal_fromcorners(self, save_data = True, load_data = False): 
        #                                , do_mono_calib = False):
        #if do_mono_calib:
        #    print
        #     TODO Perform monocular calibrations
        #    self.l.cal_fromcorners(cam1_corners)
        #    self.r.cal_fromcorners(cam2_corners)
        cam1_ipts, cam2_ipts, b = None, None, None

        if load_data:
            #TODO Loading and save dirs set via ros param.
            print "LOADING DATA"
            cam1_ipts = np.load("/tmp/cam1_points.npy")
            cam2_ipts = np.load("/tmp/cam2_points.npy")
            b = np.load("/tmp/board_points.npy")
            print "LOADED"
            #TODO print some data abt loaded stuff
        else:
            # each elem in stereo_corr.s: (cam1_frame, cam2_frame)
            # each cam#_frame: (img, corners, timestamp)
            cam1_ipts = np.array([ cam1[1] for cam1, cam2 in self.stereo_correspondences])
            cam2_ipts = np.array([ cam2[1] for cam1, cam2 in self.stereo_correspondences])
            try:
                xtion_y = self.stereo_correspondences[0][1][0].shape[0]
                xtion_x = self.stereo_correspondences[0][1][0].shape[1]
                for corners in cam2_ipts:
                    for i in range(corners.shape[0]):
                        corners[i,0,1] = xtion_y - corners[i,0,1]
                        corners[i,0,0] = xtion_x - corners[i,0,0]
            except IndexError:
                n = str(len(self.stereo_correspondences))
                print "NOT ENOUGH CORRESPONDENCES FOUND: %s" % n
                return

            #cv2.stereoCalib requires each image pair have its own set of board points...
            b = np.array([self.CHESSBOARD[0] for i in range(len(self.stereo_correspondences))])
            
            # Only save data if it wasn't loaded in the first place:
            if save_data:
                print "SAVING CAM1,2 POINTS AND BOARD POINTS TO /tmp/"
                numpy.save("/tmp/board_points",b)
                numpy.save("/tmp/cam1_points",cam1_ipts)
                numpy.save("/tmp/cam2_points",cam2_ipts)

        flags = cv2.CALIB_FIX_INTRINSIC

        self.T = numpy.zeros((3, 1), dtype=numpy.float64)
        self.R = numpy.eye(3, dtype=numpy.float64)
        self.cam2_D = np.zeros((5,1), np.float64)
        if LooseVersion(cv2.__version__).version[0] == 2:
            cv2.stereoCalibrate(b, cam1_ipts, cam2_ipts, 
                               self.size,
                               self.cam1_K, self.cam1_D,
                               self.cam2_K, self.cam2_D,
                               self.R,                            # R
                               self.T,                            # T
                               criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                               flags = flags)

        else:
            cv2.stereoCalibrate(self.CHESSBOARD, cam1_ipts, cam2_ipts,
                               self.cam1_K, self.cam1_D,
                               self.cam2_K, self.cam2_D,
                               self.size,
                               self.R,                            # R
                               self.T,                            # T
                               criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                               flags = flags)
        print "RESULTS (R, T):"
        print self.R, self.T
        homog = np.hstack((self.R, np.array([[0],[0],[0]])))
        homog = np.vstack((homog, np.array([0,0,0,1])))
        print "generating quaternion from:"
        print homog
        print
        print "AS A QUATERNION:"
        quat = tf.transformations.quaternion_from_matrix(homog)
        print quat

        # Send the calculated tf to the tf handler node:
        self.publish_stereo_tf(Pose(self.T, quat))
        #TODO need this?
        #self.set_alpha(0.0)


    def mk_object_points(self, use_board_size = False):
        opts = []
        num_pts = self.n_board_cols * self.n_board_rows
        opts_loc = numpy.zeros((num_pts, 1, 3), numpy.float32)
        for j in range(num_pts):
            opts_loc[j, 0, 0] = (j / self.n_board_cols)
            opts_loc[j, 0, 1] = (j % self.n_board_cols)
            opts_loc[j, 0, 2] = 0
            if use_board_size:
                opts_loc[j, 0, :] = opts_loc[j, 0, :] * self.CHESSBOARD_SIZE
        opts.append(opts_loc)
        return opts

# Fetch all parameters and initialize calibrator object:
def start_calibrator():
    rospy.init_node('stereo_calibrator')
    # Fetch each camera's parameters.  Convert lists to np arrays:
    cam1 = rospy.get_param('cam1') # a dict
    cam1['K'] = np.array(cam1['K'])
    cam1['D'] = np.array(cam1['D'])
    cam2 = rospy.get_param('cam2')
    cam2['K'] = np.array(cam2['K'])
    cam2['D'] = np.array(cam2['D'])

    # Fetch calibration parameters:
    board_rows = rospy.get_param('~board_rows')
    board_cols = rospy.get_param('~board_cols')
    board_size = rospy.get_param('~board_size')
    load_calib_data = rospy.get_param('~load_calib_data')
    save_calib_data = rospy.get_param('~save_calib_data')

    calibrator = BaxterStereoCalibrator("cam1_feed","cam2_feed", 
            cam1['K'], cam1['D'],cam2['K'], cam2['D'],
            board_dim = (board_rows, board_cols), board_size = board_size,
            load_calib_data = load_calib_data,
            save_calib_data = save_calib_data)
    rospy.spin()

if __name__ == "__main__":
    start_calibrator()

    '''
    rospy.init_node('stereo_calibrator')
    xtion_K = np.array([[525.0, 0.0, 319.5], 
                        [0.0, 525.0, 239.5], 
                        [0.0, 0.0, 1.0]])
    xtion_K = np.array([[ 552.24642152,0.0,325.71609566],
        [0.0,549.55314454,236.11290228],
        [0.0,0.0,1.0]])
    xtion_D = np.zeros((5,1), np.float64)
    d = [ 0.01178956, -0.01958604, -0.00250728, -0.0011414,  0.00411025]
    for i in range(len(xtion_D)):
        xtion_D[i][0] = d[i]

    # The bumblebee is already undistorted:
    bumblebee_K = np.array([[811.857711829874, 0.0, 515.7504920959473],
            [0.0, 811.857711829874, 403.7249565124512], 
            [0.0, 1.0, 0.0]])
    bumblebee_D = np.zeros((5,1), np.float64)

    calibrator = BaxterStereoCalibrator("/bumblebee/left/image_rect","/xtion/rgb/image_rect_mono", bumblebee_K, bumblebee_D,xtion_K, xtion_D, load_calib_data = False)
    rospy.spin()
    '''
