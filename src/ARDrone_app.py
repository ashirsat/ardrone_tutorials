#!/usr/bin/env python
import sys
import cv2
import math
from ardrone_autonomy import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from PyQt4 import QtGui, QtCore
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from threading import Lock, Thread
from drone_status import DroneStatus
from ARDrone_Image_proc import img_proc
import numpy as np

## Connection Constants ##
GUI_UPDATE_PERIOD= 20 #ms
CONNECTION_CHECK_PERIOD= 250 #ms

class ardrone_gui(QtGui.QMainWindow):
    """ This Class contains the basic GUI class and setup functions"""
    def __init__(self):
        super(ardrone_gui, self).__init__()
        self.initGUI()
        # self.img_thread = Thread(target=self.DetectredColor, args=())
        # self.img_thread.start()
        self.img_thread_lock = Lock()

    def initGUI(self):
        self.setWindowTitle('Camera Feed')
        self.imagefeedWidget = QtGui.QLabel(self)
        self.setCentralWidget(self.imagefeedWidget)

        ## Assigning a namespace for the toggle service
        self.toggle = None
        self.toggle_response = None
        self.service_name = None

        ## Setting up the lock
        self.ImageLock = Lock()

        ## Setting the cvbrideg module
        self.cvbridge = CvBridge()

        ## Setting up the Image data sets
        self.Image = None

        ## Setting up subscribers
        self.VideoSubscriber  = None #rospy.Subscriber('/ardrone/image_raw', Image, self.ReceiveImage)
        self.VideoPublisher = None


        ## Defing the GUI constsants
        CONNECTION_CHECK_PERIOD = 250  # ms
        GUI_UPDATE_PERIOD = 20  # ms
        self.CommunicationSinceTimer = False

        ## Check the connection and data reception flags
        self.ConnectionTimer = QtCore.QTimer(self)
        self.ConnectionTimer.timeout.connect(self.ConnectionCallBack)
        self.ConnectionTimer.start(CONNECTION_CHECK_PERIOD)

        ## Redraw the GUI
        self.RedrawTimer = QtCore.QTimer(self)
        self.RedrawTimer.timeout.connect(self.RedrawCallBack)
        self.RedrawTimer.start(GUI_UPDATE_PERIOD)

        ## Setting up the menubar
        self.mainMenu = self.menuBar()  # QtGui.QMenuBar()
        self.mainMenu.setGeometry(0, 0, 1280, 25)
        drone_choice = self.mainMenu.addMenu('&Choose Drone')
        camera_function = self.mainMenu.addMenu('&Camera Functionality')

        ##Defining action
        ### Connecting to the various ARDrones in the swarm
        drone1_connect = QtGui.QAction('&Drone 1', self)
        drone2_connect = QtGui.QAction('&Drone 2', self)
        drone3_connect = QtGui.QAction('&Drone 3', self)
        drone4_connect = QtGui.QAction('&Drone 4', self)

        ## Adding the Push button to the interface for toggling between front and bottom camera
        self.ToggleBtn = QtGui.QPushButton('Toggle Camera', self)
        self.ToggleBtn.setGeometry(240, 300, 25, 25)
        self.ToggleBtn.resize(self.ToggleBtn.sizeHint())


        ### Defining various camera functionalities for each individual ARDrone
        self.camera_function1 = QtGui.QAction('&Track Red Color',
                                         self)  ## Call the function from the image processing script
        self.camera_function2 = QtGui.QAction('&Detect faces', self)  ## Call the function from the image processing script
        self.camera_function3 = QtGui.QAction('&Track Lines', self)  ## Call the function from the image processing script
        self.camera_function4 = QtGui.QAction('&Track Led Pattern', self)  ## Call the function from the image processing script

        ## Adding The action to the menubar
        drone_choice.addAction(drone1_connect)  ## Connect to the  ARDrone Alpha Uno
        drone_choice.addAction(drone2_connect)  ## Connect to the  ARDrone Aplha Duo
        drone_choice.addAction(drone3_connect)  ## Connect to the  ARDrone Alpha Tres
        drone_choice.addAction(drone4_connect)  ## Connect to the  ARDrone Quatro

        camera_function.addAction(self.camera_function1)
        camera_function.addAction(self.camera_function2)
        camera_function.addAction(self.camera_function3)
        camera_function.addAction(self.camera_function4)

        ## Triggering the menu actions
        # self.camera_function1.
        ## Setting up the status bar
        self.Status = self.statusBar()  # QtGui.QStatusBar()

        # self.Status.showMessage(self.StatusMessages if self.connected else self.Disconnected) ### To do

    def ConnectionCallBack(self):
        self.connected = self.CommunicationSinceTimer
        self.CommunicationSinceTimer = False

    def RedrawCallBack(self):
        if self.Image is not None:
            self.ImageLock.acquire()
            try:
                Image = QtGui.QPixmap.fromImage(QtGui.QImage(self.Image.data, self.Image.width, self.Image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.ImageLock.release()

            self.resize(Image.width(), Image.height())
            self.imagefeedWidget.setPixmap(Image)

    def ReceiveImage(self, data):
        self.CommunicationSinceTimer = True
        self.ImageLock.acquire()
        try:
            self.Image = data
        finally:
            self.ImageLock.release()

    def ToggleService(self):
        rospy.wait_for_service(self.service_name)
        try:
            self.toggle = rospy.ServiceProxy(self.service_name, Empty)
            self.toggle_response = self.toggle()
        except rospy.ServiceException, e:
            print 'Service call Failed:%s'%e

if __name__=='__main__':
    rospy.init_node('Main_App')
    app = QtGui.QApplication(sys.argv)
    display = ardrone_gui()
    img_process_main = img_proc()

    ## Setting up the subscribers
    display.VideoSubscriber = rospy.Subscriber('/ardrone/image_raw', Image, display.ReceiveImage)
    display.ReceiveImage(display.Image)
    # img_process_main.DroneVidSubscriber = rospy.Subscriber('/ardrone/image_raw', Image, img_process_main.DroneReceiveImage)
    # img_process_main.DroneReceiveImage(display.Image)
    # img_process_main.DroneVidSubsciberRedColor = rospy.Subscriber('/ardrone/image_raw', Image, img_process_main.DroneDetectRedColor)
    # img_process_main.DroneDetectRedColor(display.Image)


    ## Setting up the publishers
    display.VideoPublisher = rospy.Publisher('/ardrone/image_raw', Image, queue_size=921600)
    img_process_main.DroneVidPublisher = rospy.Publisher('/ardrone/image_raw', Image, queue_size=921600)


    ## Setting up the toggle service
    display.service_name = '/ardrone/togglecam'
    display.ToggleBtn.clicked.connect(display.ToggleService)

    # print type(display.Image)`
    ## Setting up the menu driven functions
    display.camera_function1.triggered.connect(img_process_main.DetectRedColorWindowNew(display.Image))
    # img_process_main.DetectRedColorWindowNew(display.Image)
    display.camera_function2.triggered.connect(img_process_main.SharpWindowNew(display.Image))
    # img_process_main.SharpWindowNew(display.Image)
    # img_proc.DroneDetectRedColor(display.Image)
    # display.ReceiveImage(img_proc.DroneImage)
    # display.RedrawCallBack()
    display.show()
    status = app.exec_()
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
