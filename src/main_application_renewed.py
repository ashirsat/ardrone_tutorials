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
        self.cvPubVideo = rospy.Publisher('/ardrone/front/image_raw', Image, queue_size=230400)

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
            # gray = cv2.cvtColor(cvimg, cv2.COLOR_RGB2GRAY)
            # self.cvimg_blur = cv2.bilateralFilter(cvimg, 5, sigmaColor=100, sigmaSpace=100)#GaussianBlur(cvimg, (3, 3))
            self.cvimg_med = cv2.medianBlur(cvimg, 3)
            # self.cvimg_bilat = cv2.bilateralFilter(self.cvimg_med, 3, sigmaColor=12, sigmaSpace=5)
            thresh = cv2.threshold(self.cvimg_med, 145, 200, cv2.THRESH_BINARY)[1]
            self.cvimg_blur = cv2.bilateralFilter(self.cvimg_med, 5, sigmaColor=10, sigmaSpace=10)
            self.cvimg_hsv = cv2.cvtColor(self.cvimg_bilat, cv2.COLOR_RGB2HSV)
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
            self.mask = cv2.erode(self.mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=8)
            self.mask = cv2.dilate(self.mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=8)
            self.cvop_hsv = self.cvimg_hsv.copy()
            self.cvop_hsv[np.where(self.mask == 0)] = 0
            # cv2.imshow('HSV',self.cvimg_hsv)
            contours, _ = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
            cnt = contours
            shape="undefined"
            Cx = 400
            Cy = 400
            for c in contours:
                peri = cv2.arcLength(c, True)
                apprx = cv2.approxPolyDP(c, 0.03 * peri, True)
                # if len(apprx) >= 9 and len(apprx) <=15:
                #     shape = "Arrow"
                #     Cx = 240
                #     Cy = 240
                if len(apprx) >= 3 and len(apprx)<=5:
                    shape = "Triangle"
                elif len(apprx) is 4:
                    (x, y, w, h) = cv2.boundingRect(apprx)
                    ar = w / float(h)
                    shape = "square" if ar > 0.95 and ar <= 1.05 else "rectangle"
                    Cx = 360 if ar > 0.95 and ar <=1.05 else 350
                    Cy = 200 if ar > 0.95 and ar <= 1.05 else 500
                elif len(apprx) > 5 and len(apprx) <= 9:
                    shape="arrow"
            # if len(cnt) > 0:
            #     rect_max = max(cnt, key=cv2.contourArea)
            #     rect = cv2.minAreaRect(rect_max)
            #     box = cv2.cv.BoxPoints(rect)
            #     box = np.int0(box)
            #     cv2.drawContours(cvimg, [box], 0, (0, 255, 0), 2)
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
                cv2.drawContours(cvimg, [c], 0, (0, 255, 0), 2)
                cv2.putText(cvimg, shape, (Cx, Cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 4)
            self.cvimage = self.cvimg_hsv
        finally:
            # pass
            self.cvimageLock.release()

    def createNewWindow(self):
        #self.cvSubVideo = rospy.Subscriber('/ardrone/front/image_raw', Image, self.cvReceiveImage)
        #self.cvReceiveImage(self.cvimage)
        self.cvDetectColor()
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
    # print type(display.image)
    cvdisplay.cvSubVideo = rospy.Subscriber('/ardrone/image_raw', Image, cvdisplay.cvReceiveImage)
    # cvdisplay.cvDetectColor()

    ## Toggling between front and bottom camera
    display.service_name='/ardrone/togglecam'
    display.toggleBtn.clicked.connect(display.ToggleService)
    display.colorBtn.clicked.connect(cvdisplay.createNewWindow)
    display.show()

    status = app.exec_()
    rospy.signal_shutdown('Works')
    # cv2.destroyAllwindows()
    sys.exit(status)







