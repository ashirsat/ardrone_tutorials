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
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms

class MultiRobot_GUI(QtGui.QMainWindow):
    def __init__(self):
        super(MultiRobot_GUI, self).__init__()
        self.initMainUI()

    def initMainUI(self):
        self.setWindowTitle('Ardrone Video Feed')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)
        self.subVideo = rospy.Subscriber('/ardrone/image_raw', Image, self.ReceiveImage)
        self.subcv2Video = rospy.Subscriber('/ardrone/image_raw', Image, self.DetectColor)
        self.bridge = CvBridge()
        # self.Image_cam = Image()
        # Setting the image frame to the current
        self.image = None
        self.imageLock = Lock()
        self.communicationSinceTimer = False
        self.connected = False

        ## Check the connection and data reception flags
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        ## Redrawing the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

        ## Gui Buttons
        toggleBtn = QtGui.QPushButton('Toggle Camera', self)
        toggleBtn.setGeometry(100, 300, 25, 25)
        toggleBtn.resize(toggleBtn.sizeHint())
        toggleBtn.clicked.connect(self.ToggleService)

        colorBtn = QtGui.QPushButton('Detect Color', self)
        colorBtn.setGeometry(300, 300, 25, 25)
        colorBtn.resize(colorBtn.sizeHint())
        colorBtn.clicked.connect(self.DetectColor)

    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    def RedrawCallback(self):
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
        rospy.wait_for_service('/ardrone/togglecam')
        # rospy.wait_for_service('/ardrone/getcamchannel')
        try:
            self.toggle = rospy.ServiceProxy('/ardrone/togglecam', Empty)
            self.toggle_response = self.toggle()
            # self.channel = rospy.ServiceProxy('/ardrone/getcamchannel', CamChannel)
            # self.camchannel = self.channel
        except rospy.ServiceException, e:
            print 'Service call Failed %s'%e

    def DetectColor(self, data):
        # print msg
        self.communicationSinceTimer = True
        self.imageLock.acquire()
        # self.image = msg
        # print(self.image)
        # print self.image.encoding
        # print type(msg)
        # print type(self.image)
        # print type(self.image.encoding)
        # self.image.encoding = '8UC3'
        try:
            # dtype, n_channels = self.bridge.encoding_to_dtype_with_channels(self.Image_cam.encoding)
            cv_image = self.bridge.imgmsg_to_cv2(self.image, '8UC3')
            # print cv_image.shape
            # (self.img_rows, self.img_cols, self.img_chanl) = self.cv_image.shape
            self.img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            ## Creating a mask for detecting color
            ## Lower bounds
            low_red = np.array([10, 10, 0])
            high_red = np.array([255, 255, 10])
            mask_low = cv2.inRange(self.img_hsv, low_red, high_red)

            ## Upper bounds
            low_red = np.array([10, 10, 170])
            high_red = np.array([255, 255, 180])
            mask_high = cv2.inRange(self.img_hsv, low_red, high_red)

            ## Final mask
            mask = mask_low + mask_high
            self.op_hsv = self.img_hsv.copy()
            self.op_hsv[np.where(mask == 0)] = 0
            self.image_cv =self.bridge.cv2_to_imgmsg(self.op_hsv)
            # self.image = self.image_cv
            # print self.op_hsv//////////////////////////////////////////////////////////////////////////////////////////

            #
            # ## Displaying the mask img
            # cv2.namedWindow("CV2 Window")
            # cv2.imshow("Mask", self.op_hsv)
            # cv2.waitKey(0)

        except CvBridgeError as e:
            print e
        finally:
            self.imageLock.release()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    rospy.init_node('ardrone_video_display')
    display = MultiRobot_GUI()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Works')
    sys.exit(status)






