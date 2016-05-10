#!/usr/bin/env python

################################################################################
# Stereo calibration
# Collects (semi-)synced image pairs of a chessboard calibration target.
#
# Some functionality based on the ROS camera_calibration package K, D)
#  Specifically, downsampling-based checkerboard detection, [others?]
# To stop gathering frames and perform R,T estimation, 
#   close either camera feed window.
#
# TODO: implement flags for rotating new frames.
# TODO: replace prints with appropriate ros log calls.
################################################################################

import math
import numpy as np
import numpy.matlib
import yaml 
import cv2
import sys
import os # for directory creation when saving data.
import roslib
import rospy
import genpy
import tf
from datetime import datetime # for file timestamp
from distutils.version import LooseVersion
#from async_stereo_calibration.srv import *
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError

def pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return np.linalg.norm((p1[0]-p2[0],p1[1]-p2[1]))

class AsyncStereoCalibrator(object):
    def __init__(self, cam1_topic_name, cam2_topic_name, 
            cam1_K, cam1_D, cam2_K, cam2_D, 
            cam1_do_undistort = False, cam2_do_undistort = False,
            cam1_is_upside_down = False, cam2_is_upside_down = False,
            board_dim = (8,10), board_size = .02,
            cam1_do_mono_calib = False, cam2_do_mono_calib = False,
            load_calib_data = False, save_calib_data = True,
            save_calib_imgs = False, load_calib_imgs = False,
            save_calib_results = False, manual_frame_capture = True,
            load_data_dir = "/tmp/", save_data_dir = "/tmp/"):
        self.load_calib_data = load_calib_data
        self.save_calib_data = save_calib_data
        self.load_data_dir = load_data_dir  # do calib from image points
        
        self.save_data_dir = save_data_dir + \
                "calib_" + datetime.now().strftime("%Y-%m-%d-%y-%H-%M") + "/"
        if not os.path.exists(self.save_data_dir):
            print "Creating dir for saving data: %s" % self.save_data_dir
            os.makedirs(self.save_data_dir)
        self.save_calib_imgs = save_calib_imgs
        self.load_calib_imgs = load_calib_imgs # do calib from raw images (no points)
        self.manual_frame_capture = manual_frame_capture
        self.save_calib_results = save_calib_results
        self.cam1_K = cam1_K
        self.cam1_D = cam1_D
        self.cam2_K = cam2_K
        self.cam2_D = cam2_D
        self.cam1_do_undistort = cam1_do_undistort
        self.cam2_do_undistort = cam2_do_undistort
        self.cam1_do_mono_calib = cam1_do_mono_calib 
        self.cam2_do_mono_calib = cam2_do_mono_calib 
        self.cam1_is_upside_down = cam1_is_upside_down
        self.cam2_is_upside_down = cam2_is_upside_down
        if cam1_is_upside_down and cam2_is_upside_down:
            rospy.logerr("Cam1 and Cam2 are both flagged as being upside down.\n"\
                    "\tBoth cameras cannot be upside-down relative to each other.")
            return

        ### CONSTANTS: 
        # Max allowed time gap between two camera frames in a correspondence:
        #self.MAX_CORRESPONDENCE_LAG = genpy.Duration.from_sec(.1)
        self.MAX_CORRESPONDENCE_LAG = .05
        # Min duration between successive correspondences:
        self.MIN_DELAY_BETWEEN_CORRESPONDENCES = genpy.Duration.from_sec(1)
        # Length of one chessboard square, in meters.  Default .02
        self.CHESSBOARD_SIZE = board_size
        # Make sure n_cols > n_rows to agree with OpenCV CB detector output:
        self.N_BOARD_COLS = max(board_dim)
        self.N_BOARD_ROWS = min(board_dim)
        # Generate chessboard corner loc.s, in chessboard frame:
        self.CHESSBOARD = self.mk_object_points(use_board_size = True)
        # Timeout for user requesting frame capture:
        self.KEYPRESS_TIMEOUT = rospy.Duration(.25)

        self.cam1_size = None
        self.cam2_size = None
        

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
        # For manual frame capture:
        self.frame_requested = False
        self.frame_request_time = rospy.Duration(0)

        self.block_leader_callback = False # TODO shameful kludge
        
        # Var.s for dynamic load balancing:
        self.cam1_last_frame_time = None
        self.cam2_last_frame_time = None
        # ID (1 or 2) of the "Leader" for this round of correspondence-finding.
        #   That is, the camera which first found the chessboard.
        #   The "Follower" cam is the camera whose frames we then search for a
        #       correspondence.
        self.leader_id = 1
        
        # Service for publishing the calculated transform:
        #self.publish_stereo_tf = rospy.ServiceProxy('set_stereo_tf', SetStereoTF)

        # Calibrate using recorded point correspondencies, or capture new ones:
        if self.load_calib_data:
            self.cal_fromcorners()
        else:
            # Init node pubs and subs for checkerboard detection: 
            self.bridge = CvBridge()
            self.cam1_sub = rospy.Subscriber(self.cam1_topic_name, 
                    Image, self.timestamp_register_cam1, queue_size=1)
            self.cam2_sub = rospy.Subscriber(self.cam2_topic_name, 
                    Image, self.timestamp_register_cam2, queue_size=1)
            # For manual frame capture, activate keypress publishing node:
            self.keypress_sub = None
            if self.manual_frame_capture:
                print "Waiting for keypress publisher service...",
                rospy.wait_for_service("publish_keypresses")
                print " found it."
                self.activate_keypress_publisher = rospy.ServiceProxy('publish_keypresses', Trigger)
                self.activate_keypress_publisher()
                self.keypress_sub = rospy.Subscriber("keypresses", Int32,
                        self.keypress_callback, queue_size=1)

            # Calibrate on shutdown:
            rospy.on_shutdown(self.cal_fromcorners)
    
    def keypress_callback(self, keypress):
        #keypress = keypress.data
        print "Keypress detected.  Saving next frame."
        self.frame_requested = True
        self.frame_request_time = rospy.Time.now()

    # Reset state for image correspondence search.
    def reset_correspondence_search(self):
        self.frame_requested = False
        self.need_follower_calib_frame = False 
        self.leader_calib_frame_time = None
        self.leader_calib_frame_img = None
        self.leader_calib_frame_chessboard = None
        self.follower_frame_buffer = []

        self.frame_request_time = rospy.Duration(0)
        print "TOTAL CORRESPONDENCES FOUND:\t", str(len(self.stereo_correspondences))
        print


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

        if self.save_calib_imgs:
            # Save imgs:
            print "SAVING CAM1, 2 IMAGE in directory %s:" % self.save_data_dir
            cam1_filename= self.save_data_dir+ "cam1_%02d.png" % \
                    len(self.stereo_correspondences)-1
            cam2_filename= self.save_data_dir+ "cam2_%02d.png" % \
                    len(self.stereo_correspondences)-1
            flag = (cv2.IMWRITE_PNG_COMPRESSION, 0)
            if self.leader_id == 1:
                cv2.imwrite(cam1_filename,self.leader_calib_frame_img, flag)
                cv2.imwrite(cam2_filename,self.follower_calib_frame_img, flag)
            else:
                cv2.imwrite(cam2_filename,self.leader_calib_frame_img, flag)
                cv2.imwrite(cam1_filename,self.follower_calib_frame_img, flag)


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
    
    # TODO Kludge to make sure leader callback isn't run concurrently
    def callback_leader(self,img_msg, cam):
        if self.block_leader_callback:
            return
        else:
            self.block_leader_callback = True
            self.do_callback_leader(img_msg,cam)
            self.block_leader_callback = False

    def do_callback_leader(self, img_msg, cam):
        # The leader searches each frame received until it detects a chessboard.
        # It then saves that frame, and sets the need_follower_calib_frame flag,
        #  prompting the follower to search for a corresponding frame.

        # For manual capture mode: ignore frame if user hasn't requested 
        #   a frame, or if user request has timed out:
        if self.manual_frame_capture:
            if self.frame_requested:
                time_since_keypress = rospy.get_rostime() - self.frame_request_time
                if time_since_keypress > self.KEYPRESS_TIMEOUT:
                    self.frame_requested = False
                    self.frame_request_time = rospy.Duration(0)
                    print "Timed out before finding valid correspondence!"
                    return
            else:
                return

        # Ignore this frame is not enough time passed since last correspondence:
        too_soon = img_msg.header.stamp - self.last_correspondence_time < \
                self.MIN_DELAY_BETWEEN_CORRESPONDENCES
        if too_soon:
            print "Waiting %fs before looking for more frames." % (self.MIN_DELAY_BETWEEN_CORRESPONDENCES.to_sec() - (img_msg.header.stamp.to_sec() - self.last_correspondence_time.to_sec()))
            return
        # Ignore this frame if currently calibrating based on earlier leader frame
        elif self.need_follower_calib_frame:  
            return
        else:
            self.leader_check_for_chessboard(img_msg, cam)

    # Check for chessboard in current frame.
    #  If chessboard found, start search in other camera.
    def leader_check_for_chessboard(self, img_msg, cam):
        # Convert img message to opencv img:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg)
            if cam == 2: 
                # Store cam img dimensions:
                if self.cam2_size is None:
                    self.cam2_size = cv_image.shape[0:2]
            elif self.cam1_size is None:
                self.cam1_size = cv_image.shape[0:2]

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
            #cv2.drawChessboardCorners(scrib,
            #        (self.N_BOARD_ROWS, self.N_BOARD_COLS), 
            #        downsampled_corners, True)
            self.need_follower_calib_frame = True


    def callback_follower(self,img_msg, cam):
        # The follower waits for the need_follower_calib_frame is set, at which
        #  point it searches incoming frames for the chessboard.  It does this
        #  until incoming frames are timestamped AFTER the leader's calib frame,
        #  at which point the best correspondence is saved.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg)

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
        '''
        # Show frame:
        dt = img_msg.header.stamp-self.init_time
        dt = dt.to_sec()
        cv2.putText(cv_image, str(dt), 
                (0,cv_image.shape[0]),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (255,255,255), 10)
        cv2.putText(cv_image, str(dt), 
                (0,cv_image.shape[0]),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (0,0,255), 3)
        '''

    def downsample_and_detect(self, img):
        """
        Downsample the input image to approximately VGA resolution and detect the
        calibration target corners in the full-size image.

        Scales the corners back up to the correct size after detection.

        Returns (scrib, corners, downsampled_corners, (x_scale, y_scale)).

        This function based on code from the ROS camera calibration package.
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
        This function copied from the ROS camera calibration package.
        """
        h = img.shape[0]
        w = img.shape[1]
        if len(img.shape) == 3 and img.shape[2] == 3:
            mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            mono = img
        (ok, corners) = cv2.findChessboardCorners(mono, (self.N_BOARD_COLS, self.N_BOARD_ROWS), 
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
            for row in range(self.N_BOARD_ROWS):
                for col in range(self.N_BOARD_COLS - 1):
                    index = row*self.N_BOARD_ROWS + col
                    min_distance = min(min_distance, pdist(corners[index, 0], corners[index + 1, 0]))
            for row in range(self.N_BOARD_ROWS - 1):
                for col in range(self.N_BOARD_COLS):
                    index = row*self.N_BOARD_ROWS + col
                    min_distance = min(min_distance, pdist(corners[index, 0], corners[index + self.N_BOARD_COLS, 0]))
            radius = int(math.ceil(min_distance * 0.5))
            cv2.cornerSubPix(mono, corners, (radius,radius), (-1,-1),
                                          ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1 ))

        return (ok, corners)
    
    # RUN STEREO CALIBRATION (also mono, if specified)
    def cal_fromcorners(self): 
        '''This function based on code from the ROS camera calibration package.'''
        ret = self.get_calib_data()
        if ret is None:  # If too few correspondencies, just quit.
            return
        cam1_ipts, cam2_ipts, b = ret

        #TODO add debug flag for useful printouts like this:
        """
        print "Calibration data info:"
        print "  CAM1 data array shape:\t %s" % str(cam1_ipts.shape)
        print "  CAM2 data array shape:\t %s" % str(cam2_ipts.shape)
        print "  BOARD data array shape:\t %s" % str(b.shape)
        """

        if self.cam1_do_mono_calib or self.cam2_do_mono_calib:
            print "Doing mono calibration for specified cameras."
            self.mono_cal_fromcorners(cam1_ipts, cam2_ipts, b)

        print "Doing stereo calibration..."
        flags = cv2.CALIB_FIX_INTRINSIC # Intrinsics must be provided.

        self.T = numpy.zeros((3, 1), dtype=numpy.float64)
        self.R = numpy.eye(3, dtype=numpy.float64)
        #self.cam2_D = np.zeros((5,1), np.float64)
        if LooseVersion(cv2.__version__).version[0] == 2:
            cv2.stereoCalibrate(b, cam1_ipts, cam2_ipts, 
                    self.cam1_size,
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
        print "=======STEREO======"
        print "R = "
        print self.R, 
        print "\nT = "
        print self.T
        homog = np.hstack((self.R, np.array([[0],[0],[0]])))
        homog = np.vstack((homog, np.array([0,0,0,1])))
        print "\nR as a quaternion:"
        q = tf.transformations.quaternion_from_matrix(homog)
        print q
        # NOTE quat_from_mat() doesn't return a quaternion object!
        #   But the Pose msg requires one!  
        #   Fun times if ya like debugging cryptic errors.
        q_msg = Quaternion(q[0], q[1], q[2], q[3])
        t_msg = Point(self.T[0], self.T[1], self.T[2])


        if self.save_calib_results:
            filename = self.save_data_dir + "stereoParams.yaml"
            while os.path.isfile(filename):
                root, ext = os.path.splitext(filename)
                filename = root + "_new" + ext
            data = dict(
                    R = self.R.flatten().tolist(),
                    T = self.T.flatten().tolist(),
                    R_quaternion = q.flatten().tolist())
            with open(filename,'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            print "===Saved results to " + filename
        else:
            "Results not saved."

        print "~~~ Calibration finished! ~~~"
    
        '''
        # Send the calculated tf to the tf handler node:
        posemsg = Pose(t_msg, q_msg)
        try:
            self.publish_stereo_tf(posemsg)
        except rospy.service.ServiceException:
            print "Service request was successful!"
        print "Shutting down calibrator node %s" % rospy.get_name()
        sys.exit([0])
        '''

    def mono_cal_fromcorners(self, cam1_ipts, cam2_ipts, opts):
        #TODO get img size for both cams
        # Calib for cam1:
        if self.cam1_do_mono_calib:
            intrinsics = numpy.zeros((3, 3), numpy.float64)
            distortion = numpy.zeros((8, 1), numpy.float64) # rational polynomial
            #TODO rational distortion always returns zero vector...
            """
            dist_model = "rational"
            if cv2.CALIB_RATIONAL_MODEL:
                distortion = numpy.zeros((8, 1), numpy.float64) # rational polynomial
            else:
            """
            distortion = numpy.zeros((5, 1), numpy.float64) # plumb bob
            dist_model = "plumb_bob"
            # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
            intrinsics[0,0] = 1.0
            intrinsics[1,1] = 1.0
            cv2.calibrateCamera(
                       opts, cam1_ipts,
                       self.cam1_size, intrinsics,
                       distortion)
            self.cam1_K, self.cam1_D = intrinsics, distortion
            print "======CAM1====="
            print "INTRINSICS:"
            print intrinsics
            print "DISTORTION:"
            print distortion
            if self.save_calib_results:
                self.save_mono_calib_results(self.cam1_K, self.cam1_D, 
                        dist_model, self.cam1_size, "cam1")

        if self.cam2_do_mono_calib:
            # Calib for cam2:
            intrinsics = numpy.zeros((3, 3), numpy.float64)
            distortion = numpy.zeros((8, 1), numpy.float64) # rational polynomial
            """
            dist_model = "rational"
            if cv2.CALIB_RATIONAL_MODEL:
                distortion = numpy.zeros((8, 1), numpy.float64) # rational polynomial
            else:
            """
            distortion = numpy.zeros((5, 1), numpy.float64) # plumb bob
            dist_model = "plumb_bob"
            # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
            intrinsics[0,0] = 1.0
            intrinsics[1,1] = 1.0
            cv2.calibrateCamera(
                       opts, cam2_ipts,
                       self.cam2_size, intrinsics,
                       distortion)
            self.cam2_K, self.cam2_D = intrinsics, distortion
            print "======CAM2====="
            print "INTRINSICS:"
            print intrinsics
            print "DISTORTION:"
            print distortion
            if self.save_calib_results:
                self.save_mono_calib_results(self.cam2_K, self.cam2_D, 
                        dist_model, self.cam2_size, "cam2")

    def save_mono_calib_results(self, K, D, dist_model, image_size, cam_name):
        height, width = image_size
        P = np.vstack((K, np.array([0,1,0])))

        filename = self.save_data_dir + cam_name + "Params.yaml"
        while os.path.isfile(filename):
            root, ext = os.path.splitext(filename)
            filename = root + "_new" + ext
        data = dict(
                image_width = width,
                image_height = height,
                camera_name = cam_name,
                camera_matrix = dict(
                    rows = 3,
                    cols = 3,
                    data = K.flatten().tolist()
                    ),
                distortion_model = dist_model,
                distortion_coefficients = dict(
                    rows = 1,
                    cols = 5,
                    data = D.flatten().tolist()
                    ),
                rectification_matrix = dict(
                    rows = 3,
                    cols = 3,
                    data = np.eye(3).flatten().tolist()
                    ),
                projection_matrix = dict(
                    rows = 3,
                    cols = 4,
                    data = P.flatten().tolist()
                    )
                )
        with open(filename,'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print "===Saved results to " + filename

    # Load calib data from file or prepare it from newly captured frames.
    #   Return cam1_points, cam2_points, board_points
    def get_calib_data(self):
        cam1_ipts, cam2_ipts, b = None, None, None
        
        if self.load_calib_data: ### Load data from save_data_dir:
            # Load images and run chessboard detection:
            if self.load_calib_imgs:
                cam1_ipts, cam2_ipts, b = self.load_imgs_and_get_pts()
            # Load previously-detected chessboard points:
            else:
                try:
                    print "Attempting to load data from %s" % self.load_data_dir
                    print "  Looking for %s" % self.load_data_dir + "board_pts.npy"
                    b = np.load(self.load_data_dir + "board_pts.npy")
                    print "  Looking for %s" % self.load_data_dir + "cam1_pts.npy"
                    cam1_ipts = np.load(self.load_data_dir + "cam1_pts.npy")
                    print "  Looking for %s" % self.load_data_dir + "cam2_pts.npy"
                    cam2_ipts = np.load(self.load_data_dir + "cam2_pts.npy")
                    print "Loading successful!"
                except IOError:
                    print "FAILED TO FIND SAVED DATA!"
                    sys.exit(["Exited on load failure."])
        ### Create calib point arrays from captured correspondencies:
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
                rospy.logwarn("Insufficient number correspondences to do calibration: %s" % n)
                return 

            #cv2.stereoCalib requires each image pair have its own set of board points...
            b = np.array([self.CHESSBOARD[0] for i in range(len(self.stereo_correspondences))])

            ### Save new data, if specified:
            if self.save_calib_data:
                tstamp = datetime.now().strftime("%Y-%m-%d-%y-%H-%M")
                board_file = self.save_data_dir + "board_pts_" + tstamp
                cam1_file = self.save_data_dir + "cam1_pts_" + tstamp
                cam2_file = self.save_data_dir + "cam2_pts_" + tstamp
                print "SAVING CAM1,2 POINTS AND BOARD POINTS:"
                numpy.save(board_file,b)
                print "\tBOARD PTS: %s" % board_file
                numpy.save(cam1_file,cam1_ipts)
                print "\tCAM1 PTS: %s" % cam1_file
                numpy.save(cam2_file,cam2_ipts)
                print "\tCAM2 PTS: %s" % cam2_file
        
        # Correct ipts for any upside-down cameras:
        # Reverse order of point coords to ensure upside down ipts 
        #   have the correct checkerboard orientation.  
        #   (TODO less cryptic comment :P )
        if self.cam1_is_upside_down:
            print "Correcting ipts for upside-down camera 1."
            cam1_ipts = cam1_ipts[:,::-1,:,:]
        if self.cam2_is_upside_down:
            print "Correcting ipts for upside-down camera 2."
            cam2_ipts = cam2_ipts[:,::-1,:,:]

        return cam1_ipts, cam2_ipts, b

# Load imgs and perform checkerboard detection on them.
#  Return 3-tuple: left img pts, right img pts, and board pts.
    def load_imgs_and_get_pts(self):
        img_dir = self.load_data_dir
        print "Attempting to load images from %s:" % img_dir
        # Expects img_dir to contain file names of the form camX_#.(extension)
        img_filenames = filter(lambda s:".png" in s, os.listdir(img_dir))
        cam1_filenames = filter(lambda s: "cam1_" in s, img_filenames)
        cam2_filenames = filter(lambda s: "cam2_" in s, img_filenames)
        cam1_filenames.sort(key=lambda s: int(s[5:s.index('.png')]))
        cam2_filenames.sort(key=lambda s: int(s[5:s.index('.png')]))
        img_pairs_filenames = zip(cam1_filenames, cam2_filenames)
        self.img_ext = os.path.splitext(cam1_filenames[0])
        cam1_ipts_list, cam2_ipts_list = [], []

        for cam1_file, cam2_file in img_pairs_filenames:
            cam1_img = cv2.imread(img_dir + "/" + cam1_file, 0)
            if self.cam1_size == None:
                self.cam1_size = cam1_img.shape[0:2]
            cam2_img = cv2.imread(img_dir + "/" + cam2_file, 0)
            if self.cam2_size == None:
                self.cam2_size = cam2_img.shape[0:2]
            scrib, corners, downsampled_corners, scales = self.downsample_and_detect(cam1_img)
            if corners is not None:
                print "Found corners in cam1 img!  Checking cam2:"
                scrib2, corners2, downsampled_corners2, scales2 = self.downsample_and_detect(cam1_img)
                if corners2 is None:
                    print "Could not find corners in cam2 img."
                    continue
                print "Found corners in cam2 img!  ADDING PAIR TO LIST."
                cam1_ipts_list.append(corners)
                cam2_ipts_list.append(corners2)
            else:  # If corners not found in cam1
                continue
        cam1_ipts = np.array(cam1_ipts_list)
        cam2_ipts = np.array(cam2_ipts_list)
        b = np.array([self.CHESSBOARD[0] for i in range(len(cam1_ipts))])

        return cam1_ipts, cam2_ipts, b


    def mk_object_points(self, use_board_size = False):
        '''This function based on code from the ROS camera calibration package.'''
        opts = []
        num_pts = self.N_BOARD_COLS * self.N_BOARD_ROWS
        opts_loc = numpy.zeros((num_pts, 1, 3), numpy.float32)
        for j in range(num_pts):
            opts_loc[j, 0, 0] = (j / self.N_BOARD_COLS)
            opts_loc[j, 0, 1] = (j % self.N_BOARD_COLS)
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
    cam1_do_mono_calib = cam1['do_mono_calib']
    cam1_is_upside_down = cam1['is_upside_down']
    cam2 = rospy.get_param('cam2')
    cam2['K'] = np.array(cam2['K'])
    cam2['D'] = np.array(cam2['D'])
    cam2_is_upside_down = cam2['is_upside_down']
    cam2_do_mono_calib = cam2['do_mono_calib']

    # Fetch calibration parameters:
    board_rows = rospy.get_param('~board_rows')
    board_cols = rospy.get_param('~board_cols')
    board_size = rospy.get_param('~board_size')
    load_calib_data = rospy.get_param('~load_calib_data')
    save_calib_data = rospy.get_param('~save_calib_data')
    load_data_dir = rospy.get_param('~load_data_dir')
    save_data_dir = rospy.get_param('~save_data_dir')
    save_results = rospy.get_param('~save_calib_results')
    manual_frame_cap = rospy.get_param('~manual_frame_capture')
    save_imgs = rospy.get_param('~save_calib_imgs')
    load_imgs = rospy.get_param('~load_calib_imgs')


    calibrator = AsyncStereoCalibrator("cam1_feed","cam2_feed", 
            cam1['K'], cam1['D'],cam2['K'], cam2['D'],
            board_dim = (board_rows, board_cols), board_size = board_size,
            cam1_is_upside_down = cam1_is_upside_down,
            cam2_is_upside_down = cam2_is_upside_down,
            cam1_do_mono_calib = cam1_do_mono_calib,
            cam2_do_mono_calib = cam2_do_mono_calib,
            load_calib_data = load_calib_data,
            load_calib_imgs = load_imgs,
            save_calib_data = save_calib_data,
            save_calib_imgs = save_imgs,
            save_calib_results = save_results,
            manual_frame_capture = manual_frame_cap,
            load_data_dir = load_data_dir,
            save_data_dir = save_data_dir)
    rospy.spin()

if __name__ == "__main__":
    start_calibrator()
