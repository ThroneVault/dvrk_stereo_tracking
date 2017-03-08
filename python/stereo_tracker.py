#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from image_geometry import StereoCameraModel
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class tool_tracker:
    def __init__(self):
        self.bridge = CvBridge()
        # Create camera model for calculating 3d position
        self.cam_model = StereoCameraModel()
        msgL = rospy.wait_for_message("/stereo/left/camera_info",CameraInfo,1);
        msgR = rospy.wait_for_message("/stereo/right/camera_info",CameraInfo,1);
        self.cam_model.fromCameraInfo(msgL,msgR)
        # Set up subscribers for camera images
        self.image_r_sub = rospy.Subscriber("/stereo/right/image_rect_color",Image,self.image_r_callback)
        self.imageL_sub = rospy.Subscriber("/stereo/left/image_rect_color",Image,self.imageL_callback)
        # Set up blank image for left camera to update
        self.imageL = np.zeros((1,1,3),np.uint8)
        # Set up GUI
        cv2.namedWindow('image')
        cv2.createTrackbar('H','image',0,180,self.nothing_cb)
        cv2.createTrackbar('min S','image',80,255,self.nothing_cb)
        cv2.createTrackbar('min V','image',80,255,self.nothing_cb)
        cv2.createTrackbar('max V','image',255,255,self.nothing_cb)
        cv2.createTrackbar('masked','image',0,1,self.nothing_cb)

    def nothing_cb(self,data):
        pass

    def mask(self,img):
        # Convert to HSV and mask colors
        h = cv2.getTrackbarPos('H','image')
        sMin = cv2.getTrackbarPos('min S','image')
        vMin = cv2.getTrackbarPos('min V','image')
        vMax = cv2.getTrackbarPos('max V','image')
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

    def makeHSV(self,img):
        blurred = cv2.GaussianBlur(img, (21, 21), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        hsv[:,:,2] = 255
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

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
            if radius > 2:
                return center
        # Otherwise return nonsense
        return None

    def imageL_callback(self,data):
        try:
            self.imageL = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def image_r_callback(self,data):
        try:
            imageR = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # Process left image if it exists
        (rows,cols,channels) = self.imageL.shape

        if cols > 60 and rows > 60 :
            maskImageL = self.mask(self.imageL)
            centerL = self.get_centroid(maskImageL)

        # if it doesn't exist, don't do anything
        else:
            return

        (rows,cols,channels) = imageR.shape
        if cols > 60 and rows > 60 :
            maskImageR = self.mask(imageR)
            centerR = self.get_centroid(maskImageR)
        else:
            return

        if(centerL != None and centerR != None):
            pixel_shift = 1
            point3D = self.cam_model.projectPixelTo3d(centerL,centerL[0] - centerR[0])
            #print(centerL)
            print(point3D)

            cv2.circle(self.imageL, (int(centerL[0]), int(centerL[1])),
                       3,(0, 255, 0), -1, cv2.CV_AA)

            cv2.circle(imageR, (int(centerR[0]), int(centerR[1])),
                       3,(0, 255, 0), -1, cv2.CV_AA)

        if cv2.getTrackbarPos('masked','image') == 0:
            # Draw centroids
            doubleImage = np.zeros((rows,cols*2,channels),np.uint8)
            doubleImage[0:rows,0:cols] = self.imageL
            doubleImage[0:rows,cols:cols*2] = imageR
        else:
            doubleImage = np.zeros((rows,cols*2),np.uint8)
            doubleImage[0:rows,0:cols] = maskImageL
            doubleImage[0:rows,cols:cols*2] = maskImageR
        cv2.imshow("image", doubleImage)
        cv2.waitKey(3)

def drawCircleFloat(img,center,size):
    c = np.array(center)
    c_floor = np.floor(c)
    c_ceil = np.ceil(c)
    ratio = c - np.floor(c)
    inverse = 1 - ratio
    opacities = np.hstack((ratio,inverse)) * .5
    positions = np.array([[c_floor[0], c_ceil[1] ],
                          [c_ceil[0],  c_ceil[1] ],
                          [c_floor[0], c_floor[1]],
                          [c_ceil[0],  c_floor[1]]])
    circleMask = np.zeros()
    overlay = np.zeros((size+1,size+1))
    for i in [0,1]:
        for j in [0,1]:

            circle(img,positions[i],size,(0, 255, 0))




def main(args):
    rospy.init_node('tool_tracker', anonymous=True)
    ic = tool_tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)