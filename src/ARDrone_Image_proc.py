#!/usr/bin/env python
import sys
import numpy as np
import cv2
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from threading import Lock, Thread
# from ARDrone_app import ardrone_gui
from PyQt4 import QtGui, QtCore         

### Connection Constanta
CONNECTION_CHECK_PERIOD = 250 # ms
GUI_UPDATE_PERIOD = 20 # ms

class img_proc(QtGui.QMainWindow):
    """ This class will handle all the image processing functionalities for the main ardrone GUI """
    def __init__(self):
        super(img_proc, self).__init__()
        self.init_Image_proc_CLASS()
        self.img_feed = QtGui.QLabel(self)
        self.setCentralWidget(self.img_feed)


    def init_Image_proc_CLASS(self):
        """ Inintialising variables for the class"""
        # self.DroneImage = None
        self.DroneVidSubscriber = None #rospy.Subscriber('/ardrone/image_raw', Image, self.DroneDetectRedColor)
        self.DroneVidSubscriberSharp = None #rospy.Subscriber('/ardrone/image_raw', Image, self.DroneImageSharpen)
        self.DroneVidSubsciberRedColor= None

        self.DroneVidPublisher = rospy.Publisher('/ardrone/image_raw', Image, queue_size=921600)
        self.cvbridge = CvBridge()
        self.img_thread_lock = Lock()
        self.DroneImage = None
        self.setWindowTitle('Image Processing')
        ## Redrawing the GUI
        self.img_procredrawTimer = QtCore.QTimer(self)
        # self.img_procredrawTimer.timeout.connect(self.DroneRedrawCallBack)
        self.img_procredrawTimer.start(GUI_UPDATE_PERIOD)


    def DroneReceiveImage(self, data):
        # self.CommunicationSinceTimer = True
        self.img_thread_lock.acquire()
        try:
            self.DroneImage = data
        finally:
            self.img_thread_lock.release()

    def DroneRedrawCallBack(self):
        if self.DroneImage is not None:
            self.img_thread_lock.acquire()
            try:
                DroneImage = QtGui.QPixmap(QtGui.QImage(self.DroneImage.data, 640, 360, QtGui.QImage.Format_RGB888))
            finally:
                self.img_thread_lock.release()

            self.resize(DroneImage.width(), DroneImage.height())
            self.img_feed.setPixmap(DroneImage)
            # self.imagefeedWidget.setPixmap(DroneImage)


    def DroneDetectRedColor(self, data):
        self.img_thread_lock.acquire()
        try:
            ## Converting the image msg to cv2 datatype ###
            cvimg = self.cvbridge.imgmsg_to_cv2(data)
            self.cvimg_hsv = cv2.cvtColor(cvimg, cv2.COLOR_RGB2HSV)

            ## Creating the mask for detecting the red color
            ## Lower bounds
            low_red = np.array([0, 100, 100])
            high_red = np.array([10, 255, 255])
            mask_low_red = cv2.inRange(self.cvimg_hsv, low_red, high_red)

            ## Upper Bounds
            low_red = np.array([160, 100, 100])
            high_red = np.array([179, 255, 255])
            mask_high_red = cv2.inRange(self.cvimg_hsv, low_red, high_red)

            ## Final mask
            self.mask_red = mask_low_red + mask_high_red
            self.mask_red = cv2.erode(self.mask_red, None, iterations=4)
            self.mask_red = cv2.dilate(self.mask_red, None, iterations=4)
            self.cv_ophsv = self.cvimg_hsv.copy()
            self.cv_ophsv[np.where(self.mask_red == 0)] = 0

            ## Detecting and drawing bounding boxes
            contours, _ = cv2.findContours(self.mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnt = contours
            if len(cnt) > 0:
                # print "Detected Contours. "
                rect_max = max(cnt, key=cv2.contourArea)
                rect = cv2.minAreaRect(rect_max)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(cvimg, [box], 0, (0, 255, 0), 2)
            self.DroneImage = self.cvbridge.cv2_to_imgmsg(cvimg)
            # print type(self.DroneImage)
            self.DroneVidPublisher.publish(self.DroneImage)
            # self.DroneVidPublisher.publish(self.cvbridge.cv2_to_imgmsg(cvimg))
        except CvBridgeError as e:
            print e
        finally:
            self.img_thread_lock.release()

    def DroneImageSharpen(self, data):
        self.img_thread_lock.acquire()
        try:
            sharp_kern = np.array([(-1, -1, -1, -1, -1), (-1, -1, -1, -1, -1), (-1, -1, 10, -1, -1), (-1, -1, -1, -1, -1),
                                   (-1, -1, -1, -1, -1)])
            img = self.cvbridge.imgmsg_to_cv2(data)
            self.orig_image = img
            self.sharp_img = cv2.filter2D(self.orig_image, -1, sharp_kern)
            self.DroneImage = self.sharp_img
            # self.DroneVidPublisher.publish(self.cvbridge.cv2_to_imgmsg(self.sharp_img))
        except CvBridgeError as e:
            print e
        finally:
            self.img_thread_lock.release()

    def SharpWindowNew(self, data):
        self.DroneImageSharpen(data)
        self.show()


    def DetectRedColorWindowNew(self, data):
        self.DroneDetectRedColor(data)
        self.show()



if __name__ == '__main__':
    rospy.init_node('Drone_Image_Processing')
    img_process = img_proc()
    img_process.SharpWindowNew(img_process.DroneImage)
    img_process.DetectRedColorWindowNew(img_process.DroneImage)
    # img_process.DroneReceiveImage(img_process, img_process.DroneImage)
    img_process.show()
    status = img_process.exec_()
    rospy.signal_shutdown('Drone Video Processing works')
    sys.exit(status)

    # img_proc.DroneVideoSubscriber = rospy.Subscriber('/ardrone/image_raw', Image, img_proc.DroneReceiveImage)
    # img_proc.DroneVidPublisher = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)

