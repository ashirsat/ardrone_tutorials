#!/usr/bin/env python
import sys
import numpy as np
import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from PyQt4 import QtGui, QtCore
from threading import Lock
from ardrone_autonomy import *
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from ARdrone_Image_Proc import ARdrone_Image_proc
## GUI Constants
CONNECTION_CHECK_PERIOD = 250 #MS
GUI_UPDATE_PERIOD = 20 #MS

class GUI(QtGui.QMainWindow):
    def __init__(self):
        super(GUI, self).__init__()
        self.initGUI()
        self.img_process = ARdrone_Image_proc()

    def initGUI(self):
        self.setWindowTitle('ARDrone Vidoe Feed')
        self.imageBox = QtGui.QLabel(self)
        self.setCentralWidget(self.imageBox)

        ### Subscribers
        self.SubNavdata = None
        self.SubVideo = None

        ### Publishers
        self.PubNavdata = None
        self.PubVideo = None


        ## Setting the current image frame
        self.image = None
        self.imageLock = Lock()
        self.CommunicationSinceTimer = False
        self.connected = False
        self.serviceName = None

        ## Checking the data connection and data reception flags
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        ## Redraw the GUI
        self.redrawTiemr = QtCore.QTimer(self)
        self.redrawTiemr.timeout.connect(self.RedrawCallback)
        self.redrawTiemr.start(GUI_UPDATE_PERIOD)

        ## Setting up the Menubar for different actions for the experiment
        self.MainMenu = self.menuBar()
        self.MainMenu.setGeometry(0, 0, 640, 25)
        Choose_ardrone = self.MainMenu.addMenu('&Choose ARDrone')
        Image_functions = self.MainMenu.addMenu('&Image Processing')

        ## Deinfing Menu actions
        ## Connecting to the various ARDrones
        ARDrone1_connect = QtGui.QAction('&Alpha One', self)
        ARDrone2_connect = QtGui.QAction('&Alpha Two', self)
        ARDrone3_connect = QtGui.QAction('&Alpha Three', self)
        ARDrone4_connect = QtGui.QAction('&Alpha Four', self)

        ## Adding the toggle camera functionality
        self.ToggleBtn = QtGui.QPushButton('Toggle Camera', self)
        self.ToggleBtn.setGeometry(240, 300, 25, 25)
        self.ToggleBtn.resize(self.ToggleBtn.sizeHint())

        ## Setting the Status bar
        self.Status = self.statusBar()

        ## Defining the various camera functionalities for individual ARDrone
        self.Image_process1 = QtGui.QAction('&Track Color',self)
        self.Image_process2 = QtGui.QAction('&Detect Shapes',self)

        ## Adding all the actions to the menubar
        Choose_ardrone.addAction(ARDrone1_connect)
        Choose_ardrone.addAction(ARDrone2_connect)
        Choose_ardrone.addAction(ARDrone3_connect)
        Choose_ardrone.addAction(ARDrone4_connect)

        Image_functions.addAction(self.Image_process1)
        Image_functions.addAction(self.Image_process2)

    def ConnectionCallback(self):
        self.connected = self.CommunicationSinceTimer
        self.CommunicationSinceTimer = False

    def RedrawCallback(self):
        if self.image is not None:
            self.imageLock.acquire()
            try:
                Image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(Image.width(), Image.height())
            self.imageBox.setPixmap(Image)

    def ReceiveImage(self, data):
        self.communicationSinceTimer = True
        self.imageLock. acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

    # def ReceiveNavdata(self, navdata):
    #     pass
    #     # self.communicationSinceTimer = True
    #     # self.imageLock.acquire()
    #     # try:
    #     #     Nav_data= navdata
    #     #     # print len(Nav_data)
    #     # finally:
    #     #     self.imageLock.release()

    def ToggleService(self):
        rospy.wait_for_service(self.serviceName)
        try:
            self.toggle = rospy.ServiceProxy(self.serviceName, Empty)
            self.toggle_response = self.toggle()
        except rospy.ServiceException, e:
            print 'Service call Failed:%s'%e

if __name__ == '__main__':
    rospy.init_node('Ardrone_gui_app')
    app = QtGui.QApplication(sys.argv)
    ## Initializing the various GUI windows
    display = GUI()
    # img_proc = ARdrone_Image_proc()

    ## Setting up the subscribers for the Main GUI
    display.SubVideo = rospy.Subscriber('/ardrone/image_raw', Image, display.ReceiveImage)
    display.ReceiveImage(display.image)
    display.SubNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, display.ReceiveNavdata)
    ## Setting up the ros service
    display.serviceName = '/ardrone/togglecam'
    display.ToggleBtn.clicked.connect(display.ToggleService)

    ## Triggering different actions from the main GUI
    display.Image_process1.triggered.connect(display.img_process.HSVReceiveImage)
    display.img_process.HSVReceiveImage(display.image)

    ## Showing the GUI
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying')
    sys.exit(status)

