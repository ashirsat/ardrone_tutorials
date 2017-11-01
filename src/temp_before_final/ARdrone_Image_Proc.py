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
CV_GUI_UPDATE_PERIOD = 20 #ms
CV_CONNECTION_CHECK_PERIOD = 250 #ms
# TOGGLE_COUNT = 0 ## This corresposnds to the front camera


class ARdrone_Image_proc(QtGui.QMainWindow):
    """This class contains the functions for the image processing that is to be done from the ardrone video stream"""
    def __init__(self):
        super(ARdrone_Image_proc, self).__init__()
        self.initCVGUI()
        # XboxController()

    def initCVGUI(self):

        self.cv_thread_lock = Lock()
        self.cv_thread = Thread()

        ## Initialising the cvbridge module
        self.cvbridge = CvBridge()

        # Initialsing the Image
        self.image = None
        self.cv_hsv = None
        self.hsv_img = None
        self.hsv_img_bottom = None
        self.xbox = XboxController()
        # self.pyqt_image = None

        # self.camchannel = CamSelect
        ## Initializing the service name
        self.serviceName = None

        ## Initializing the subscriber
        self.cv_subs = None
        self.hsv_subs = None
        self.img_subs = None

        self.flag = False
        ## Initialising the publisher
        self.cv_pubs = None
        self.hsv_pubs = None
        self.img_pubs = None
        self.ToggleCount = 0   ## Corresponds to the front camera by default

        ## Initializing the cv windows
        cv2.namedWindow('Image Processing', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('HSV Window', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Blurred Image', cv2.WINDOW_AUTOSIZE)
        # cv2.namedWindow('Circle Detect', cv2.WINDOW_AUTOSIZE)

        # Initializing the trackbars for hsv
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
        # self.RedrawTimer.timeout.connect(self.RedrawCallback)
        self.RedrawTimer.timeout.connect(self.HSVRedrawCallback)
        # self.RedrawTimer.timeout.connect(self.GrayCircleRedrawCallback)
        self.RedrawTimer.start(CV_GUI_UPDATE_PERIOD)

        self.sub_togglecount = rospy.Subscriber('/ardrone/togglecount', Int8, self.ImgTogglecam)
        self.pub_centroids  = rospy.Publisher('/ardrone/shapeCentroids', vector21, queue_size=2)
        # self.pub_centroids_y  = rospy.Publisher('/ardrone/centroids', Int8, queue_size=10)

    def ConnectionCallback(self):
        self.connected = self.CommunicationSinceTimer
        self.CommunicationSinceTimer = False

    # def RedrawCallback(self):
    #     if self.image is not None:
    #         self.cv_thread_lock.acquire()
    #         try:
    #             self.cv_img = self.cvbridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
    #             k = cv2.waitKey(1) & 0xFF
    #             if k == ord('q'):
    #                 cv2.destroyAllWindows()
    #             else:
    #                 cv2.imshow('Image Processing', self.cv_img)
    #                 # self.ImageFilterHSV()
    #         finally:
    #             self.cv_thread_lock.release()

    def HSVRedrawCallback(self):
        if self.hsv_img is not None:
            self.cv_thread_lock.acquire()
            try:
                self.cv_hsv = self.cvbridge.imgmsg_to_cv2(self.hsv_img, desired_encoding="bgr8")
                self.cv_hsv2 = cv2.cvtColor(self.cv_hsv, cv2.COLOR_BGR2HSV)
                # print 'Inside HSV redraw'
                # if self.flag is True:
                #     # self.ToggleCount += 1
                # print "Toggle Count:%s"%(self.ToggleCount)

                # self.hue_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                # self.hue_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                # self.sat_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                # self.sat_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                # self.val_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                # self.val_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')

                if self.ToggleCount % 2 is 0 or self.ToggleCount is 0:  ## Front camera is running
                    self.hue_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.hue_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.sat_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.sat_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.val_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.val_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')

                    # self.hue_low = 0
                    # self.hue_high = 10
                    # self.sat_low = 50
                    # self.sat_high = 255
                    # self.val_low = 100
                    # self.val_high = 255

                    # self.cv_hsv = self.cvbridge.imgmsg_to_cv2(self.hsv_img, desired_encoding="bgr8")
                    # self.cv_hsv2 = cv2.cvtColor(self.cv_hsv,cv2.COLOR_BGR2HSV)
                else:                                                   ## Bottom camera is running
                    self.hue_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.hue_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.sat_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.sat_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.val_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                    self.val_high = cv2.getTrackbarPos('Hue Low', 'HSV Window')

                    # self.hue_low = 140
                    # self.hue_high = 179
                    # self.sat_low = 40
                    # self.sat_high = 255
                    # self.val_low = 100
                    # self.val_high = 255

                    # self.cv_hsv = self.cvbridge.imgmsg_to_cv2(self.hsv_img, desired_encoding="rgb8")
                    # self.cv_hsv2 = cv2.cvtColor(self.cv_hsv, cv2.COLOR_RGB2HSV)

                # self.gaus_blur_size = cv2.getTrackbarPos('Blur Size', 'Blurred Image')
                # self.gaus_blur_size = 2*self.gaus_blur_size - 1
                # self.s_space = cv2.getTrackbarPos('Sigma Space', 'Blurred Image')
                # self.s_color = cv2.getTrackbarPos('Sigma Color', 'Blurred Image')
                """ The Blur kernel needs to be odd numbered """
                self.cv_hsv3 = cv2.GaussianBlur(self.cv_hsv2, (3, 3), 0)
                # self.cv_hsv2 = cv2.bilateralFilter(self.cv_hsv, d=10,   sigmaColor=self.s_space, sigmaSpace=self.s_space)
                # cv2.imshow('Blurred Image', self.cv_hsv)
                # self.statusBar()
                """ Setting the different hsv values fro the front and the bottom camera"""
                # self.hue_low = 0
                #
                # self.hue_high = 10
                #
                # self.sat_low = 50
                #
                # self.sat_high = 255
                #
                # self.val_low = 100
                #
                # self.val_high = 255
                # else:                                               ### Bottom camera is running
                    # self.hue_low = 150
                    #
                    # self.hue_high = 179
                    #
                    # self.sat_low = 50
                    #
                    # self.sat_high = 255
                    #
                    # self.val_low = 100
                    #
                    # self.val_high = 255
                # self.hue_low = cv2.getTrackbarPos('Hue Low', 'HSV Window')
                #
                # self.hue_high = cv2.getTrackbarPos('Hue High', 'HSV Window')
                #
                # self.sat_low = cv2.getTrackbarPos('Sat Low', 'HSV Window')
                #
                # self.sat_high = cv2.getTrackbarPos('Sat High', 'HSV Window')
                #
                # self.val_low = cv2.getTrackbarPos('Val Low', 'HSV Window')
                #
                # self.val_high = cv2.getTrackbarPos('Val High', 'HSV Window')
                # # print "Hlow=%d, Hhi=%d, Slo=%d, Shi=%d, Vlo=%d, Vhi=%d"%(self.hue_low, self.hue_high, self.sat_low, self.sat_high, self.val_low, self.val_high)
                low_red = np.array([self.hue_low, self.sat_low, self.val_low])
                high_red = np.array([self.hue_high, self.sat_high, self.val_high])
                self.mask_red = cv2.inRange(self.cv_hsv3, low_red, high_red)
                self.mask_red = cv2.erode(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=8)
                self.mask_red = cv2.dilate(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=8)
                self.mask_red = cv2.dilate(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                                           iterations=8)
                self.mask_red = cv2.erode(self.mask_red, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                                          iterations=8)
                self.cv_ophsv = self.cv_hsv.copy()
                self.cv_ophsv[np.where(self.mask_red == 0)] = 0
                res = cv2.bitwise_and(self.cv_hsv2, self.cv_hsv2, mask= self.mask_red)
                contours, _ = cv2.findContours(self.mask_red.copy(), cv2.RETR_EXTERNAL,   cv2.CHAIN_APPROX_TC89_L1)
                cnt = contours
                shape= "undefined"
                Cx = 100
                Cy = 200

                for c in cnt:
                    M = cv2.moments(c)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # print "CentreX:%d, CentreY:%d"%(cx, cy)
                    # self.pub_centroids.publish([cx, cy])
                    # self.pub_centroids_x.publish(cx)
                    # self.pub_centroids_y.publish(cy)
                    peri = cv2.arcLength(c, True)
                    apprx = cv2.approxPolyDP(c, 0.03*peri, True)
                    # if len(apprx) is 4:
                    #     # (x, y, w, h) = cv2.boundingRect(apprx)
                    #     # ar = w/float(h)
                    #     # if ar >=0.95 and ar <=1.05:
                    #     shape = "Square"
                    #     # else:
                    #     #     shape = "Rectangle"
                    # else:
                    #     shape = "Undefined"
                    # if len(apprx) >=3 and len(apprx) <=6:
                    #     shape = "Triangle"
                    # elif len(apprx)>6 and len(apprx)<=8:
                    #     shape="Arrow"
                    cv2.drawContours(res,[c], 0, (0,255,0), 2)
                    cv2.circle(res, (cx, cy), 7, (255, 255, 255), -1)
                    cv2.putText(res, "centroid", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 4)
                    self.pub_centroids.publish([cx, cy])
                    # cv2.putText(res, shape, (Cx, Cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 4)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    cv2.destroyAllWindows()
                else:
                    cv2.imshow('HSV Window', res)
                    cv2.imshow('Blurred Image', self.cv_hsv3)
            finally:
                self.cv_thread_lock.release()

    # def GrayCircleRedrawCallback(self):
    #     if self.hsv_img is not None:
    #         self.cv_thread_lock.acquire()
    #         try:
    #             self.gray = self.cvbridge.imgmsg_to_cv2(self.hsv_img, desired_encoding="bgr8")
    #             res = self.gray.copy()
    #             self.cv_gray2 = cv2.cvtColor(self.gray, cv2.COLOR_BGR2GRAY)
    #             """ The Blur kernel needs to be odd numbered """
    #             self.cv_gray3 = cv2.GaussianBlur(self.cv_gray2, (3,3), 2, 2)
    #             self.cv_circle = cv2.HoughCircles(self.cv_gray2, cv2.cv.CV_HOUGH_GRADIENT, 0.8, 50)
    #             # res2 = cv2.bitwise_and(self.cv_gray3, self.cv_gray3, mask=self.cv_circle)
    #             if self.cv_circle is not None:
    #                 circle = np.round(self.cv_circle[0, :]).astype("int")
    #                 for (x,y,r) in circle:
    #                     cv2.circle(res, (x, y), r, (0, 255, 0), 4)
    #                     cv2.rectangle(res, (x - 5, y- 5), (x + 5, y + 5), (0, 128, 255), -1)
    #                     cv2.putText(res, "center", (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 255), 4)
    #                     print "Circle Center:%d , %d" %(x,y)
    #                     # cv2.putText(self.cv_gray3, "Xc:", x, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 4)
    #                     # cv2.putText(self.cv_gray3, "Yc:", y, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 4)
    #             else:
    #                 pass
    #                 # print "No circle detected"
    #             k = cv2.waitKey(1) & 0xFF
    #             if k == ord('q'):
    #                 cv2.destroyAllWindows()
    #             else:
    #                 cv2.imshow('Circle Detect', self.cv_gray3)
    #         finally:
    #             self.cv_thread_lock.release()

    def nothing(self, x):
        pass

    def HSVReceiveImage(self, data):
        self.CommunicationSinceTimer = True
        self.cv_thread_lock.acquire()
        try:
            self.hsv_img = data
        except CvBridgeError as e:
            print e
        finally:
            self.cv_thread_lock.release()

    def HSVReceiveImageBottom(self, data):
        self.CommunicationSinceTimer = True
        self.cv_thread_lock.acquire()
        try:
            self.hsv_img_bottom = data
        except CvBridgeError as e:
            print e
        finally:
            self.cv_thread_lock.release()

    def ImgTogglecam(self, togglecount):
        self.ToggleCount = togglecount.data


# class PyqtGUI(QtGui.QMainWindow):
#     def __init__(self):
#         super(PyqtGUI, self).__init__()
#         self.pyqtGUI()
#
#     def pyqtGUI(self):
#         self.setWindowTitle('Pyqt Window')
#         self.pyqt_imagebox = QtGui.QLabel(self)
#         self.setCentralWidget(self.pyqt_imagebox)
#         self.pyqt_thread_lock = Lock()
#         self.pyqt_thread = Thread()
#
#         ## Initialising the cvbridge module
#         self.pyqtbridge = CvBridge()
#
#         # Initialsing the Image
#         self.pyqt_img = None
#
#         ## Initializing the service name
#         self.serviceName = None
#
#         ## Initializing the subscriber
#         self.pyqt_subs = None
#
#         ## Initialising the publisher
#         self.pyqt_pubs = None
#
#         ## Setting up the GUI constants
#         self.CommunicationSinceTimer = False
#
#         ## Check the communication and data reception flags
#         self.ConnectionTimer = QtCore.QTimer(self)
#         self.ConnectionTimer.timeout.connect(self.PyqtConnectionCallback)
#         self.ConnectionTimer.start(CV_CONNECTION_CHECK_PERIOD)
#
#         ## Redrawing the GUI
#         self.RedrawTimer = QtCore.QTimer(self)
#         self.RedrawTimer.timeout.connect(self.PyqtRedrawCallback)
#         self.RedrawTimer.start(CV_GUI_UPDATE_PERIOD)
#
#         ## Creating the toggle button
#         self.ToggleBtn = QtGui.QPushButton('Toggle Camera', self)
#         self.ToggleBtn.setGeometry(200, 300, 150, 25)
#
#
#     def PyqtConnectionCallback(self):
#         self.connected = self.CommunicationSinceTimer
#         self.CommunicationSinceTimer = False
#
#     def PyqtRedrawCallback(self):
#         if self.pyqt_img is not None:
#             self.pyqt_thread_lock.acquire()
#             try:
#                 pyqt_img = QtGui.QPixmap.fromImage(QtGui.QImage(self.pyqt_img.data, self.pyqt_img.width, self.pyqt_img.height, QtGui.QImage.Format_RGB888))
#             finally:
#                 self.pyqt_thread_lock.release()
#             self.resize(pyqt_img.width(), pyqt_img.height())
#             self.pyqt_imagebox.setPixmap(pyqt_img)
#
#     def PyqtReceiveImage(self, data):
#         self.CommunicationSinceTimer = True
#         self.pyqt_thread_lock.acquire()
#         try:
#             self.pyqt_img = data
#         except CvBridgeError as e:
#             print e
#         finally:
#             self.pyqt_thread_lock.release()
#
#     def TogleService(self):
#         rospy.wait_for_service(self.serviceName)
#         try:
#             self.Toggle = rospy.ServiceProxy(self.serviceName, Empty)
#             self.Toggle_response = self.Toggle()
#         except rospy.ServiceException, e:
#             print 'Service call failed %s'%e

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    rospy.init_node('ARdrone_image_proc')

    disp = ARdrone_Image_proc()
    # pyqt = PyqtGUI()
    #
    # pyqt.pyqt_subs = rospy.Subscriber('/ardrone/image_raw', Image, pyqt.PyqtReceiveImage)
    # pyqt.PyqtReceiveImage(pyqt.pyqt_img)
    #
    # pyqt.pyqt_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)
    # pyqt.serviceName = '/ardrone/togglecam'
    #
    # pyqt.ToggleBtn.clicked.connect(pyqt.TogleService)
    # xbox = XboxController()
    # xbox = XboxController()
    # disp.ToggleCount = xbox.count
    disp.hsv_subs = rospy.Subscriber('/ardrone/image_raw', Image, disp.HSVReceiveImage)
    disp.HSVReceiveImage(disp.hsv_img)

    # disp.img_subs = rospy.Subscriber('/ardone/image_raw', Image, disp.HSVReceiveImageBottom)
    # disp.HSVReceiveImageBottom(disp.hsv_img_bottom)

    disp.cv_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)

    disp.hsv_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)

    # disp.img_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)
    disp.pyqt_pubs = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)

    # disp.CamchannelSelect()
    # pyqt.show()
    disp.show()
    status = app.exec_()
    rospy.signal_shutdown('End img_proc thread')
    # rospy.spin()
    sys.exit(status)
    