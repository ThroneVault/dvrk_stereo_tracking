#!/usr/bin/env python

import sys, os, signal
from PyQt4 import QtGui, uic, QtCore

from PyQt4.QtCore import SIGNAL
import rospy
import cv2
import numpy as np
from image_geometry import StereoCameraModel
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from IPython import embed
import scipy.io
import time
import pickle
import os.path
import sys
import tf.transformations

NUM_ARM_POSES = 5

# TODO put arm poses here.

class AutomaticRegistration(QtGui.QMainWindow):
    def __init__(self):
        self.failed = True
        super(AutomaticRegistration, self).__init__()
        rospy.init_node('tool_tracker', anonymous=True)

        self.bridge = CvBridge()

        # Create camera model for calculating 3d position
        self.cam_model = StereoCameraModel()
        msgL = rospy.wait_for_message("/stereo/left/camera_info",CameraInfo,1);
        msgR = rospy.wait_for_message("/stereo/right/camera_info",CameraInfo,1);
        self.cam_model.fromCameraInfo(msgL,msgR)

        # Set up subscribers for camera images
        self.image_r_sub = rospy.Subscriber("/stereo/right/image_rect_color",Image,self.image_r_callback)
        self.imageL_sub = rospy.Subscriber("/stereo/left/image_rect_color",Image,self.imageL_callback)
        self.pose_sub = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, self.pose_callback)

        # Set up blank image for left camera to update
        self.imageL = np.zeros((1,1,3),np.uint8)

        # Setup GUI

        self.ui = uic.loadUi('tracking_window.ui', self)


        self.ui.hSpinBox.valueChanged.connect(self.hSpinBoxChanged)
        self.ui.sMinSpinBox.valueChanged.connect(self.sMinSpinBoxChanged)
        self.ui.vMinSpinBox.valueChanged.connect(self.vMinSpinBoxChanged)
        self.ui.vMaxSpinBox.valueChanged.connect(self.vMaxSpinBoxChanged)

        self.ui.hSlider.valueChanged.connect(self.hSliderChanged)
        self.ui.sMinSlider.valueChanged.connect(self.sMinSliderChanged)
        self.ui.vMinSlider.valueChanged.connect(self.vMinSliderChanged)
        self.ui.vMaxSlider.valueChanged.connect(self.vMaxSliderChanged)

        self.ui.startRecordingPushButton.clicked.connect(self.startRecordingCallback)
        self.ui.stopRecordingPushButton.clicked.connect(self.stopRecordingCallback)
        self.ui.exportPointsPushButton.clicked.connect(self.exportPointsCallback)
        self.ui.generateTransformPushButton.clicked.connect(self.generateTransformCallback)
        self.ui.saveTransformPushButton.clicked.connect(self.saveTransformCallback)
        self.ui.loadTransformPushButton.clicked.connect(self.loadTransformCallback)
        self.ui.validateTransformPushButton.clicked.connect(self.validateTransformCallback)
        self.ui.showSegmentedCheckBox.stateChanged.connect(self.showSegmentedCallback)
        self.ui.importPointsPushButton.clicked.connect(self.importPointsCallback)

        self.connect(self, SIGNAL("updateLeft"), self.updateLeftSlot)
        self.connect(self, SIGNAL("updateRight"), self.updateRightSlot)
        self.connect(self, SIGNAL("updateError"), self.updateErrorSlot)

        self.ui.show()

        self.ui.hSlider.setValue(0);
        self.ui.sMinSlider.setValue(80);
        self.ui.vMinSlider.setValue(80);
        self.ui.vMaxSlider.setValue(255);

        self.ui.hSpinBox.setValue(0);
        self.ui.sMinSpinBox.setValue(80);
        self.ui.vMinSpinBox.setValue(80);
        self.ui.vMaxSpinBox.setValue(255);

        self.recorded_kinematic_positions = np.empty((0,3))
        self.recorded_stereo_positions = np.empty((0,3))
        self.recorded_toolcenter_poses = np.empty((4,4))

        self.counter = 0
        self.arm_pose_counter = 0
        self.show_segmented = True 
        self.transform_loaded = False

        # Initialize bulb_position
        self.bulb_position = (0,0,0)
        self.toolcenter_pose = np.identity(4)

        # Open cached poses from file. These are the registration poses
        pickle.dump(self.pose_list, open( "pose_list.p", "wb"))

        self.bulb_position_correspondence_array = []


    def hSliderChanged(self, val):
        self.ui.hSpinBox.setValue(val)

    def sMinSliderChanged(self, val):
        self.ui.sMinSpinBox.setValue(val)

    def vMinSliderChanged(self, val):
        self.ui.vMinSpinBox.setValue(val)

    def vMaxSliderChanged(self, val):
        self.ui.vMaxSpinBox.setValue(val)

    def hSpinBoxChanged(self, val):
        self.ui.hSlider.setValue(val)

    def sMinSpinBoxChanged(self, val):
        self.ui.sMinSlider.setValue(val)

    def vMinSpinBoxChanged(self, val):
        self.ui.vMinSlider.setValue(val)

    def vMaxSpinBoxChanged(self, val):
        self.ui.vMaxSlider.setValue(val)

    def startRecordingCallback(self):
        self.ui.stopRecordingPushButton.setEnabled(True)
        self.ui.startRecordingPushButton.setEnabled(False)
        self.ui.importPointsPushButton.setEnabled(False)
        self.automatic_registration_routine()

    def stopRecordingCallback(self):
        self.ui.stopRecordingPushButton.setEnabled(False)
        self.ui.exportPointsPushButton.setEnabled(True)
        self.ui.generateTransformPushButton.setEnabled(True)

    def exportPointsCallback(self):

        arranged_recorded_toolcenter_poses = np.vsplit(self.recorded_toolcenter_poses,
                self.recorded_toolcenter_poses.shape[0]/4)

        scipy.io.savemat('recorded_data.mat',
                mdict={'recorded_stereo_positions':self.recorded_stereo_positions,
                    'recorded_kinematic_positions': self.recorded_kinematic_positions,
                    'recorded_toolcenter_poses': self.recorded_toolcenter_poses,
                    'arranged_recorded_toolcenter_poses': arranged_recorded_toolcenter_poses})


        print "file saved"

    def importPointsCallback(self):
        print "file loaded"
        inp = scipy.io.loadmat('recorded_data.mat')
        self.recorded_stereo_positions = inp['recorded_stereo_positions']
        self.recorded_kinematic_positions = inp['recorded_kinematic_positions']
        self.ui.generateTransformPushButton.setEnabled(True)
                

    def generateTransformCallback(self):
        self.ui.generateTransformPushButton.setEnabled(False)
        self.ui.saveTransformPushButton.setEnabled(True)

        self.R, self.t = self.horns_method(self.recorded_stereo_positions, self.recorded_kinematic_positions)

        print "R value is" , self.R
        print "t value is" , self.t

        self.ui.validateTransformPushButton.setEnabled(True)
        self.ui.loadTransformPushButton.setEnabled(False)
        self.transform_loaded = True

        
        avg_error = 0


        for i in range(0,len(self.recorded_kinematic_positions)):
            p_robot = self.recorded_kinematic_positions[i,:]
            tfp_camera = np.dot(self.R, p_robot) + self.t

            p_camera = self.recorded_stereo_positions[i,:]
            error = np.linalg.norm(tfp_camera - p_camera)
            avg_error += error

#        print tfp_camera.shape, p_camera.shape
        avg_error /= self.recorded_stereo_positions.shape[0]

        self.ui.avgRecordedErrorLineEdit.setText(str(avg_error * 1000))

    def saveTransformCallback(self):
        scipy.io.savemat('transform.mat',
            mdict={'R': self.R, 't': self.t})
        print "transform saved"

    def loadTransformCallback(self):
        inp = scipy.io.loadmat('transform.mat')
        self.R = inp['R']
        self.t = inp['t']
        print "transform loaded"
        self.ui.validateTransformPushButton.setEnabled(True)
        self.transform_loaded = True

    def showSegmentedCallback(self, state):
        if state:
            self.show_segmented = True
        else:
            self.show_segmented = False

    def mask(self,img):
        # Convert to HSV and mask colors

        h = self.ui.hSlider.value()
        sMin = self.ui.sMinSlider.value()
        vMin = self.ui.vMinSlider.value()
        vMax = self.ui.vMaxSlider.value()
        hueShift = 0
        if h < 20 or h > 160:
            hueShift = 60
        colorLower = ((h-5 + hueShift)%180, sMin, vMin)
        colorUpper = ((h+5 + hueShift)%180, 255, vMax)
        blurred = cv2.GaussianBlur(img, (31, 31), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        hsv[:,:,0] = (hsv[:,:,0] + hueShift)%180
        mask = cv2.inRange(hsv, colorLower, colorUpper )
        # Refine mask
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask


    def get_centroid(self,maskImage):
        # With help from http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(maskImage.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (M["m10"] / M["m00"], M["m01"] / M["m00"])
            # only proceed if the radius meets a minimum size
            if radius > 3:
                return center
        # Otherwise return nonsense
        return None

    def pose_callback(self, data):

        # Convert the DVRK pose to a 4x4 toolcenter_pose matrix to make it easy
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)

        # Create rotation matrix
        rot_matrix = tf.transformations.quaternion_matrix(quaternion)

        position = (
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z)

        # Populate toolcenter_pose
        self.toolcenter_pose = np.identity(4)

        self.toolcenter_pose[0:3, 3] = np.transpose([position[0],
            position[1], position[2]])
        self.toolcenter_pose[0:3, 0:3] = rot_matrix[0:3, 0:3]

        # TODO verify tooltip offset
        tooltip_transform = np.identity(4)
        tooltip_transform[2,3] = 0.013 # tool center is 13mm offset from the bulb

        bulb_pose = np.dot(self.toolcenter_pose, tooltip_transform)

        # return only the xyz position
        self.bulb_position = bulb_pose[0:3, 3]

    def updateLeftSlot(self):

        if self.show_segmented:
            cvImage = self.segmentedL
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_GRAY2RGB)
        else:
            cvImage = self.imageL
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB)

        height, width, channel = cvImage.shape

        byteValue = channel * width
        local_image = QtGui.QImage(cvImage, width, height, 
                byteValue, QtGui.QImage.Format_RGB888)

        pixMap = QtGui.QPixmap(local_image)
        pixMap = pixMap.scaled(320, 240, QtCore.Qt.KeepAspectRatio)
        pixMapItem = QtGui.QGraphicsPixmapItem(pixMap)

        scene = QtGui.QGraphicsScene(self)
        scene.addItem(pixMapItem)
        self.ui.leftGraphicsView.setScene(scene)

    def updateRightSlot(self):

        if self.show_segmented:
            cvImage = self.segmentedR
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_GRAY2RGB)
        else:
            cvImage = self.imageR
            cvImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB)

        height, width, channel = cvImage.shape

        byteValue = channel * width
        local_image = QtGui.QImage(cvImage, width, height, 
                byteValue, QtGui.QImage.Format_RGB888)

        pixMap = QtGui.QPixmap(local_image)
        pixMap = pixMap.scaled(320, 240, QtCore.Qt.KeepAspectRatio)
        pixMapItem = QtGui.QGraphicsPixmapItem(pixMap)

        scene = QtGui.QGraphicsScene(self)
        scene.addItem(pixMapItem)
        self.ui.rightGraphicsView.setScene(scene)

    def updateErrorSlot(self):
        # compute error TODO
        
        # p_robot = self.bulb_position
        # tfp_camera = np.dot(self.R, p_robot) + self.t

        # p_camera = self.current_stereo_position

        # # print tfp_camera.shape, p_camera.shape

        # error = np.linalg.norm(tfp_camera - p_camera)
        # self.ui.currentErrorLineEdit.setText(str(error * 1000))
        pass

    def validateTransformCallback(self):
        self.emit(SIGNAL("updateError"))

    def imageL_callback(self,data):
        try:
            self.imageL = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print "left not working"
            print e

        self.segmentedL = self.mask(self.imageL)

        self.emit(SIGNAL("updateLeft"))

        if self.transform_loaded:
            self.emit(SIGNAL("updateError"))


    def image_r_callback(self, data):
        try:
            self.imageR = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print "right not working"
            print e
        

        # store the toolcenter pose
        stored_toolcenter_pose = self.toolcenter_pose #todo verify sync

        self.segmentedR = self.mask(self.imageR)

        # If the transform has already been generated or loaded, draw it on screen for verification
        # if self.transform_loaded:
        
        self.emit(SIGNAL("updateRight"))

        # TEST ONLY TODO
        self.automatic_registration_routine()

    def visualize_transform(self):

        points3D = self.current_stereo_position
        #points3D = np.dot(self.R, points3D.T)
        #points3D = points3D.T + self.t
        point2D = self.cam_model.left.project3dToPixel(self.current_bulbcenter_position)
        point2D = (int(point2D[0]),int(point2D[1]))
        print point2D
        cv2.circle(self.imageL,point2D,5,(0,255,0),-1)
        
    def compute_bulb_position_by_stereo_triangulation(self):

        # First, do a check if the images are available. If unavailable, return false

        # Then, calculate the centroids.
        # If centroids are not along epipolar lines, return false
        # centerL = self.get_centroid(self.segmentedL)
        # centerR = self.get_centroid(self.segmentedR)
        
        # if(centerL != None and centerR != None):

        # append to some array the 3d position along with the pose

        pass

    def move_arm_to_poses(self):
        # read from table. if out of range. quit
        # Send a command to dvrk to move there.
        #Ensure you wait till it has done moved fully
        time.sleep(5)
        print "move_arm_to_poses called"
        self.arm_pose_counter += 1

    def automatic_registration_routine(self):

        # Make sure we haven't done this before
        if self.arm_pose_counter < NUM_ARM_POSES:

            # For every the cached poses
            for i in range(0, NUM_ARM_POSES):
                # Move arm to a cached pose
                self.move_arm_to_poses()

                # Compute the 3D position of the ball center and store it
                status = self.compute_bulb_position_by_stereo_triangulation()
                
                # if status = True, play good sound
                # else, play bad sound

            # TODO Now do Horn's method

        
    def horns_method(self, q, p):

        p_bar = np.mean(p, axis=0)
        q_bar = np.mean(q, axis=0) 

        # # % find data centroid and deviations from centroid

        q_mark = q - q_bar

        # # % find data centroid and deviations from centroid
        p_mark = p - p_bar
        # # % Apply weights

        N = np.dot(p_mark.T, q_mark) # taking points of q in matched order

        U, s, V = np.linalg.svd(N, full_matrices=True) # singular value decomposition

        R =  np.dot(V.T, U.T);
        t = q_bar.T - np.dot(R, p_bar.T)

        return R, t

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = AutomaticRegistration()
    sys.exit(app.exec_())
