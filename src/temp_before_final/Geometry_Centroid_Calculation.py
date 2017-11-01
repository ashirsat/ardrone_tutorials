#!/usr/bin/env python
import math
import sys
import os
import numpy as np
from PyQt4 import QtGui, QtCore
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ardrone_autonomy import *
# from ardrone_autonomy.srv import CamSelect
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from threading import Lock, Thread
from ARdrone_Joystick_control import XboxController
from std_msgs.msg import Int8
from ardrone_autonomy.msg import vector21

## Frame update constants
CV_GUI_UPDATE_PERIOD = 20  # ms
CV_CONNECTION_CHECK_PERIOD = 250  # ms


# TOGGLE_COUNT = 0 ## This corresposnds to the front camera


class Centroid_Calculator(QtGui.QMainWindow):
    """This class contains the functions for the image processing that is to be done from the ardrone video stream"""

    def __init__(self):
        super(Centroid_Calculator, self).__init__()
        self.initCVGUI()
        # XboxController()

    def initCVGUI(self):

        self.cv_thread_lock = Lock()
        self.cv_thread = Thread()
        self.centroid = vector21()
        ## Initialising the cvbridge module
        self.cvbridge = CvBridge()

        # Initialsing the Image
        self.hsv_img = None
        ## Initializing the subscriber
        self.hsv_subs = None

        self.flag = False
        ## Initialising the publisher
        self.hsv_pubs = None

        self.ToggleCount = 0  ## Corresponds to the front camera by default
        ## Initializing the cv windows
        cv2.namedWindow('HSV Window', cv2.WINDOW_AUTOSIZE)

        """Initializing the trackbars for hsv"""
        # cv2.createTrackbar('Hue Low', 'HSV Window', 0, 179, self.nothing)
        # cv2.createTrackbar('Hue High', 'HSV Window', 0, 179, self.nothing)
        # cv2.createTrackbar('Sat Low', 'HSV Window', 0, 255, self.nothing)
        # cv2.createTrackbar('Sat High', 'HSV Window', 0, 255, self.nothing)
        # cv2.createTrackbar('Val Low', 'HSV Window', 0, 255, self.nothing)
        # cv2.createTrackbar('Val High', 'HSV Window', 0, 255, self.nothing)
        # cv2.createTrackbar('Blur Size', 'Blurred Image', 1, 30, self.nothing)
        # cv2.createTrackbar('Sigma Space', 'Blurred Image', 0, 100, self.nothing)
        # cv2.createTrackbar('Sigma Color', 'Blurred Image', 0, 100, self.nothing)


        ## Setting up the GUI constants
        self.CommunicationSinceTimer = False

        ## Check the communication and data reception flags
        self.ConnectionTimer = QtCore.QTimer(self)
        self.ConnectionTimer.timeout.connect(self.ConnectionCallback)
        self.ConnectionTimer.start(CV_CONNECTION_CHECK_PERIOD)

        ## Redrawing the GUI
        self.RedrawTimer = QtCore.QTimer(self)
        self.RedrawTimer.timeout.connect(self.CentroidRedrawCallback)
        self.RedrawTimer.start(CV_GUI_UPDATE_PERIOD)

        self.sub_togglecount = rospy.Subscriber('/ardrone/togglecount', Int8, self.ImgTogglecam)
        self.pub_centroids = rospy.Publisher('/ardrone/shapeCentroids', vector21, queue_size=2)

    def ConnectionCallback(self):
        self.connected = self.CommunicationSinceTimer
        self.CommunicationSinceTimer = False


    def CentroidRedrawCallback(self):
        if self.hsv_img is not None:
            self.cv_thread_lock.acquire()
            cent_measured_x = 0
            cent_measured_y = 0
            try:
                self.cv_hsv = self.cvbridge.imgmsg_to_cv2(self.hsv_img, desired_encoding="bgr8")
                self.cv_hsv2 = cv2.cvtColor(self.cv_hsv, cv2.COLOR_BGR2HSV)
                if self.ToggleCount % 2 is 0 or self.ToggleCount is 0:  ## Front camera is running
                    # self.hue_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.hue_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.sat_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.sat_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.val_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.val_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')

                    self.hue_low = 0
                    self.hue_high = 10
                    self.sat_low = 50
                    self.sat_high = 255
                    self.val_low = 100
                    self.val_high = 255

                else:  ## Bottom camera is running
                    # self.hue_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.hue_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.sat_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.sat_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.val_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    # self.val_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')

                    self.hue_low = 140
                    self.hue_high = 179
                    self.sat_low = 40
                    self.sat_high = 255
                    self.val_low = 100
                    self.val_high = 255

                """ The Blur kernel needs to be odd numbered """
                self.cv_hsv3 = cv2.GaussianBlur(self.cv_hsv2, (5, 5), 5, 5)
                """ Setting the different hsv values fro the front and the bottom camera"""
                low_red = np.array([self.hue_low, self.sat_low, self.val_low])
                high_red = np.array([self.hue_high, self.sat_high, self.val_high])
                self.mask_red = cv2.inRange(self.cv_hsv3, low_red, high_red)
                self.mask_red = cv2.erode(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
                                          iterations=10)
                self.mask_red = cv2.dilate(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
                                           iterations=10)
                self.mask_red = cv2.dilate(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
                                           iterations=10)
                self.mask_red = cv2.erode(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
                                          iterations=10)
                self.cv_ophsv = self.cv_hsv.copy()
                self.cv_ophsv[np.where(self.mask_red == 0)] = 0
                res = cv2.bitwise_and(self.cv_hsv2, self.cv_hsv2, mask=self.mask_red)
                contours, _ = cv2.findContours(self.mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
                cnt = contours
                if len(cnt) is 0:
                    self.centroid.x = -1.0
                    self.centroid.y = -1.0
                    self.pub_centroids.publish(self.centroid)
                # else:
                c_ini = [0,0]
                for i in range(1,10,1):
                    for c in cnt:
                        M = cv2.moments(c)
                        if M["m00"] is not 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                        # peri = cv2.arcLength(c, True)
                        # apprx = cv2.approxPolyDP(c, 0.03 * peri, True)
                            cv2.drawContours(res, [c], 0, (0, 255, 0), 2)
                            cv2.circle(res, (cx, cy), 7, (255, 255, 255), -1)
                            cv2.putText(res, "centroid", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 4)
                            self.centroid.x = cx
                            self.centroid.y = cy
                        else:
                            self.centroid.x =-0.1
                            self.centroid.y =-0.1
                    c_ini[0] += self.centroid.x
                    c_ini[1] += self.centroid.y
                        # self.pub_centroids.publish(self.centroid)
                c_ini[0] = int(c_ini[0]/i)
                c_ini[1] = int(c_ini[1]/i)
                cent_measured_x= int((cent_measured_x +c_ini[0])/2)
                cent_measured_y = int((cent_measured_y + c_ini[1]) / 2)
                self.centroid.x = float(cent_measured_x)
                self.centroid.y = float(cent_measured_y)
                self.pub_centroids.publish(self.centroid)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    cv2.destroyAllWindows()
                else:
                    cv2.imshow('HSV Window', res)
                    # cv2.imshow('Blurred Image', self.cv_hsv3)
            finally:
                self.cv_thread_lock.release()


    def HSVReceiveImage(self, data):
        self.CommunicationSinceTimer = True
        self.cv_thread_lock.acquire()
        try:
            self.hsv_img = data
        except CvBridgeError as e:
            print e
        finally:
            self.cv_thread_lock.release()

    def ImgTogglecam(self, togglecount):
        self.ToggleCount = togglecount.data

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    rospy.init_node('Centroid_Calculator')

    disp = Centroid_Calculator()
    disp.hsv_subs = rospy.Subscriber('/ardrone/image_raw', Image, disp.HSVReceiveImage)
    disp.HSVReceiveImage(disp.hsv_img)
    disp.cv_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)
    disp.hsv_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)
    disp.pyqt_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)
    disp.show()
    status = app.exec_()
    rospy.signal_shutdown('End img_proc thread')
    # rospy.spin()
    sys.exit(status)
