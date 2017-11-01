#!/usr/bin/env python
import sys
import cv2
from ardrone_autonomy import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from PyQt4 import QtGui, QtCore
from std_srvs.srv import Empty
import numpy as np
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from threading import Lock

# import cvgui as cvg

### Connection Constanta
CONNECTION_CHECK_PERIOD = 250 # ms
GUI_UPDATE_PERIOD = 20 # ms

class CVGUI(QtGui.QMainWindow):
    def __init__(self):
        super(CVGUI, self).__init__()
        self.initcvGUI()

    def initcvGUI(self):
        self.setWindowTitle('CV Window')
        self.cvimageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.cvimageBox)
        self.cvSubNavdata = None
        self.cvSubVideo = None
        self.cvPubVideo = None
        self.cvbridge = CvBridge()

        # Setting the image frame to the current
        self.cvimage = None
        self.cvimageLock = Lock()
        self.cvcommunicationSinceTimer = False
        self.cvconnected = False
        self.cvservice_name = None
        ## Check the connection and data reception flags
        self.cvconnectionTimer = QtCore.QTimer(self)
        self.cvconnectionTimer.timeout.connect(self.cvConnectionCallback)
        self.cvconnectionTimer.start(CONNECTION_CHECK_PERIOD)

        ## Redrawing the GUI
        self.cvredrawTimer = QtCore.QTimer(self)
        self.cvredrawTimer.timeout.connect(self.cvRedrawCallback)
        self.cvredrawTimer.start(GUI_UPDATE_PERIOD)

    def cvConnectionCallback(self): ## Done
        self.cvconnected = self.cvcommunicationSinceTimer
        self.cvcommunicationSinceTimer = False

    def cvRedrawCallback(self): ## Done
        if self.cvimage is not None:
            self.cvimageLock.acquire()
            try:
                cvimage = QtGui.QPixmap(QtGui.QImage(self.cvimage.data, self.cvimage.width, self.cvimage.height, QtGui.QImage.Format_RGB888))
            finally:
                self.cvimageLock.release()

            self.resize(cvimage.width(), cvimage.height())
            self.cvimageBox.setPixmap(cvimage)

    def cvReceiveImage(self, data):
        self.cvcommunicationSinceTimer = True
        self.cvimageLock.acquire()
        try:
            self.cvimage = data
        finally:
            self.cvimageLock.release()

    def createNewWindow(self):
        # self.cvSubVideo = rospy.Subscriber('/ardrone/image_raw', Image, self.cvReceiveImage)
        # self.cvReceiveImage(self.cvimage)
        self.cvDetectColor()
        # cv2.imshow('mask', self.mask)
        self.show()

    def cvReceiveNavdata(self, navdata):
        self.cvcommunicationSinceTimer = True
        self. cvnav_data = navdata

    def cvDetectColor(self):
        self.cvcommunicationSinceTimer = True
        self.cvimageLock.acquire()

        try:
            cvimg = self.cvbridge.imgmsg_to_cv2(self.cvimage)
            self.cvimg_hsv = cv2.cvtColor(cvimg, cv2.COLOR_BGR2HSV)

            ## Creating a mask for detecting color
            ## Lower bounds
            low_red = np.array([50, 50, 10])
            high_red = np.array([255, 255, 70])
            mask_low = cv2.inRange(self.cvimg_hsv, low_red, high_red)

            ## Upper bounds
            low_red = np.array([50, 50, 170])
            high_red = np.array([255, 255, 180])
            mask_high = cv2.inRange(self.cvimg_hsv, low_red, high_red)

            ## Final mask
            self.mask = mask_low + mask_high
            self.cvop_hsv = self.cvimg_hsv.copy()
            self.cvop_hsv[np.where(self.mask == 0)] = 0
            self.cvimage = self.cvbridge.cv2_to_imgmsg(self.mask)
            self.cvPubVideo = rospy.Publisher('/ardrone/front/image_raw', Image, queue_size=230400)
            self.cvPubVideo.publish(self.cvimage)
        except CvBridgeError as e:
            print e
        finally:
            self.cvimageLock.release()


class MultiRobot_GUI(QtGui.QMainWindow):
    def __init__(self):
        super(MultiRobot_GUI, self).__init__()
        self.initMainUI()

    def initMainUI(self):
        self.setWindowTitle('Ardrone Video Feed')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        self.SubNavdata = None
        self.SubVideo = None

        # Setting the image frame to the current
        self.image = None
        self.imageLock = Lock()
        self.communicationSinceTimer = False
        self.connected = False
        self.service_name = None
        ## Check the connection and data reception flags
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        ## Redrawing the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

        ## Gui Buttons
        self.toggleBtn = QtGui.QPushButton('Toggle Camera', self)
        self.toggleBtn.setGeometry(100, 300, 25, 25)
        self.toggleBtn.resize(self.toggleBtn.sizeHint())
        # toggleBtn.clicked.connect(self.ToggleService)

        self.colorBtn = QtGui.QPushButton('Detect Color', self)
        self.colorBtn.setGeometry(300, 300, 25, 25)
        self.colorBtn.resize(self.colorBtn.sizeHint())
        # colorBtn.clicked.connect(self.DetectColor)

    def ConnectionCallback(self): ## Done
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    def RedrawCallback(self): ## Done
        if self.image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(), image.height())
            self.imageBox.setPixmap(image)

    def ReceiveImage(self, data):
        self.communicationSinceTimer = True
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()


    def ReceiveNavdata(self, navdata):
        self.communicationSinceTimer = True
        self. nav_data = navdata

    def ToggleService(self):
        rospy.wait_for_service(self.service_name)
        # rospy.wait_for_service('/ardrone/getcamchannel')
        try:
            self.toggle = rospy.ServiceProxy(self.service_name, Empty)
            self.toggle_response = self.toggle()
        except rospy.ServiceException, e:
            print 'Service call Failed %s'%e

    # def DetectColor(self, data):
    #     # print msg
    #     self.communicationSinceTimer = True
    #     self.imageLock.acquire()
    #     # self.image = msg
    #     # print(self.image)
    #     # print self.image.encoding
    #     # print type(msg)
    #     # print type(self.image)
    #     # print type(self.image.encoding)
    #     # self.image.encoding = '8UC3'
    #     try:
    #         # dtype, n_channels = self.bridge.encoding_to_dtype_with_channels(self.Image_cam.encoding)
    #         cv_image = self.bridge.imgmsg_to_cv2(self.image)
    #         # print cv_image.shape
    #         # (self.img_rows, self.img_cols, self.img_chanl) = self.cv_image.shape
    #         self.img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #
    #         ## Creating a mask for detecting color
    #         ## Lower bounds
    #         low_red = np.array([50, 50, 0])
    #         high_red = np.array([255, 255, 10])
    #         mask_low = cv2.inRange(self.img_hsv, low_red, high_red)
    #
    #         ## Upper bounds
    #         low_red = np.array([50, 50, 170])
    #         high_red = np.array([255, 255, 180])
    #         mask_high = cv2.inRange(self.img_hsv, low_red, high_red)
    #
    #         ## Final mask
    #         mask = mask_low + mask_high
    #         self.op_hsv = self.img_hsv.copy()
    #         self.op_hsv[np.where(mask == 0)] = 0
    #         self.image_cv =self.bridge.cv2_to_imgmsg(self.op_hsv)
    #         # print self.op_hsv//////////////////////////////////////////////////////////////////////////////////////////
    #
    #         #
    #         # ## Displaying the mask img
    #         # cv2.namedWindow("CV2 Window")
    #         # cv2.imshow("Mask", self.op_hsv)
    #         # cv2.waitKey(0)
    #
    #     except CvBridgeError as e:
    #         print e
    #     finally:
    #         self.imageLock.release()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    rospy.init_node('ardrone_video_display')
    display = MultiRobot_GUI()
    cvdisplay = CVGUI()
    ## Navdata Subscribers
    display.SubNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, display.ReceiveNavdata)

    ## Video Subscribers
    display.SubVideo = rospy.Subscriber('/ardrone/image_raw', Image, display.ReceiveImage)
    display.ReceiveImage(display.image)
    cvdisplay.cvSubVideo = rospy.Subscriber('/ardrone/front/image_raw', Image, cvdisplay.cvReceiveImage)
    # cvdisplay.cvDetectColor()

    ## Toggling between front and bottom camera
    display.service_name='/ardrone/togglecam'
    display.toggleBtn.clicked.connect(display.ToggleService)
    display.colorBtn.clicked.connect(cvdisplay.createNewWindow)
    display.show()

    status = app.exec_()
    rospy.signal_shutdown('Works')
    sys.exit(status)






