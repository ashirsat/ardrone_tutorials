#!/usr/bin/env python
import sys
import cv2
import math
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
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms

class CVGUI(QtGui.QMainWindow):
    def __init__(self):
        super(CVGUI, self).__init__()
        self.initcvGUI()
        self.cvPubVideo = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)

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
        self.cvop_hsv = None
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
                cvimage = QtGui.QPixmap(QtGui.QImage(self.cvimage, 640, 360, QtGui.QImage.Format_RGB888))
            finally:
                self.cvimageLock.release()

            self.resize(cvimage.width(), cvimage.height())
            self.cvimageBox.setPixmap(cvimage)

    def cvReceiveImage(self, data):
        self.cvcommunicationSinceTimer = True
        self.cvimageLock.acquire()
        try:
            cvimg = self.cvbridge.imgmsg_to_cv2(data)
            self.cvimg_hsv = cv2.cvtColor(cvimg, cv2.COLOR_RGB2HSV)
            # print (self.cvimg_hsv)
            # self.cvimg_gray = cv2.cvtColor(cvimg, cv2.COLOR_BGR2GRAY)
            # img_gray = self.cvimg_gray
            ## Creating a mask for detecting color
            ## Lower bounds
            low_red = np.array([0, 100, 100])
            high_red = np.array([10, 255, 255])
            mask_low = cv2.inRange(self.cvimg_hsv, low_red, high_red)

            ## Upper bounds
            low_red = np.array([160, 100, 100])
            high_red = np.array([179, 255, 255])
            mask_high = cv2.inRange(self.cvimg_hsv, low_red, high_red)

            ## Final mask
            self.mask = mask_low + mask_high
            self.mask = cv2.erode(self.mask, None, iterations=6)
            self.mask = cv2.dilate(self.mask, None, iterations=6)
            self.cvop_hsv = self.cvimg_hsv.copy()
            self.cvop_hsv[np.where(self.mask == 0)] = 0

            countours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnt = countours
            if len(cnt) > 0:
                # for c in cnt:
                #     # cv2.drawContours(cvimg, [c], (0, 255, 0), 2)
                rect_max = max(cnt, key=cv2.contourArea)
                rect = cv2.minAreaRect(rect_max)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(cvimg, [box], 0, (0, 255, 0), 2)
            # box = cv2.boxPoints(rect)
            # box= np.int0(box)
            ### Simple Blurring
            # blur_img = cv2.medianBlur(self.cvimg_hsv, 9)
            # # print enumerate(cnt)
            # for i, c in enumerate(cnt):
            #
            #     # rect = cv2.minAreaRect(c)
            # #
            # #     box = cv2.boxPoints(rect)
            # #     box = np.int0(box)
            # #     cv2.drawContours(blur_img, [box], 0, (0,0,0),3)
            #     area = cv2.contourArea(c)
            #     # print area
            #     if area > 300:
            #         cv2.drawContours(blur_img, cnt, i, (0, 0, 0), 3)
            #
            # # cv2.drawContours(blur_img, [box], 0, (0, 0, 0), 2)
            # # cv2.bitwise_and(blur_img,self.mask, self.cvop_hsv, mask=None)
            # # (bl_left, bl_upper, bl_right, bl_lower) = self.cvop_hsv.
            # # cv2.rectangle(blur_img, bl_left, bl_right, color=(0, 0, 255), thickness=1.5)
            # # sub_mask = self.img_bg.apply(blur_img)
            # # sub_mask = cv2.morphologyEx(sub_mask, cv2.MORPH_OPEN, self.back_ker)
            self.cvimage = cvimg
        finally:
            # pass
            self.cvimageLock.release()

    def createNewWindow(self):
        #self.cvSubVideo = rospy.Subscriber('/ardrone/front/image_raw', Image, self.cvReceiveImage)
        #self.cvReceiveImage(self.cvimage)
        # self.cvDetectColor()
        # cv2.namedWindow('CV')
        # cv2.imshow('mask', self.mask)
        # rospy.spin()
        self.show()


    def cvReceiveNavdata(self, navdata):
        self.cvcommunicationSinceTimer = True
        self.cvnav_data = navdata

    def cvDetectColor(self):
        self.cvcommunicationSinceTimer = True
        self.cvimageLock.acquire()
        try:
            pass
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

        ## Setting up the menubar
        self.mainMenu = self.menuBar()
        self.mainMenu.setGeometry(0, 0, 1280, 25)
        drone_choice = self.mainMenu.addMenu('&Choose Drone')
        camera_function = self.mainMenu.addMenu('&Image Processing')

        ## Defining Action
        ### Connecting to the various ARDrone 2.0
        drone1_connect = QtGui.QAction('&Alpha_One', self)
        drone2_connect = QtGui.QAction('&Alpha_Two', self)
        drone3_connect = QtGui.QAction('&Alpha_Three', self)
        drone4_connect = QtGui.QAction('&Alpha_Four', self)

        ## Adding the Toggle button to switch camera
        self.ToggleBtn = QtGui.QPushButton('Toggle Camera', self)
        self.ToggleBtn.setGeometry(240, 300, 25, 25)
        self.ToggleBtn.resize(self.ToggleBtn.sizeHint())

        ## Defining various image processing functionalities for the ARDrone
        self.CamFunc_Redcolor = QtGui.QAction('&Track Red Color', self)
        self.CamFunc_DetectFaces = QtGui.QAction('&Detect Faces', self)
        self.CamFunc_LineTrack = QtGui.QAction('&Track Lines', self)
        self.CamFunc_LedArray = QtGui.QAction('&Track LED', self)


        ## Adding the connecting actions to the menu items
        drone_choice.addAction(drone1_connect)
        drone_choice.addAction(drone2_connect)
        drone_choice.addAction(drone3_connect)
        drone_choice.addAction(drone4_connect)

        camera_function.addAction(self.CamFunc_Redcolor)
        camera_function.addAction(self.CamFunc_DetectFaces)
        camera_function.addAction(self.CamFunc_LineTrack)
        camera_function.addAction(self.CamFunc_LedArray)

        ## Setting up the status bar
        self.Status = self.statusBar()  # QtGui.QStatusBar()

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
    print type(display.image)
    cvdisplay.cvSubVideo = rospy.Subscriber('/ardrone/image_raw', Image, cvdisplay.cvReceiveImage)
    # cvdisplay.cvDetectColor()

    ## Toggling between front and bottom camera
    display.service_name='/ardrone/togglecam'
    display.ToggleBtn.clicked.connect(display.ToggleService)
    display.CamFunc_Redcolor.triggered.connect(cvdisplay.createNewWindow)
    # display.colorBtn.clicked.connect(cvdisplay.createNewWindow)
    display.show()

    status = app.exec_()
    rospy.signal_shutdown('Works')
    sys.exit(status)






