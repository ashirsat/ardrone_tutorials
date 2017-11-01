#!/usr/bin/env python

import sys
sys.path.insert(0, '/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/')
import rospy
import cv2
import datetime
# import signal
# import pygame
# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from cv_bridge import CvBridge, CvBridgeError
# from drone_video_display import DroneVideoDisplay
# import cv2
import numpy as np
# Finally the GUI libraries
from PySide import QtCore, QtGui
from threading import Thread, Lock
import time
from std_srvs.srv import Empty
from ardrone_autonomy import *
from std_msgs.msg import Int8, Empty
from ardrone_autonomy.msg import vector21
from sensor_msgs.msg import Image
from drone_status import DroneStatus
from ardrone_autonomy.msg import Navdata, vector21
from geometry_msgs.msg import Twist
import math

# global freq = 100      # Control loop rate in Hz
Kd_x = 0.005
Kd_y = 0.005
CV_GUI_UPDATE_PERIOD = 20 #ms
CV_CONNECTION_CHECK_PERIOD = 250 #ms
K_intrinsic_camera = [[686.994766, 0, 329.323208], [0, 668.195055, 159.323007], [0, 0, 1]]
img_width = 640
img_height = 360
fx = K_intrinsic_camera[0][0]
fy = K_intrinsic_camera[1][1]

class ImageBasedServoing(QtGui.QMainWindow):
    def __init__(self):
        super(ImageBasedServoing, self).__init__()
        # pygame.init()
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0
        self.status =-1
        self.done = False
        self.running = True
        self.pox= 0
        self.poy =0
        self.cent_x = 0
        self.cent_y = 0

        self.done = True
        self.cvbridge= CvBridge()
        self.hsv_img = None
        self.batt_percent = rospy.Subscriber('/ardrone/battpercent', Int8, self.batteryPercentage)
        self.altitude = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)
        self.takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.centroid_subscriber = rospy.Subscriber('/ardrone/shapeCentroids', vector21, self.ReceiveCentroids)
        self.img_subscriber = None
        self.img_publisher = None
        self.img_bottom_hsv2 = None
        self.CommunicationSinceTimer = False

        ## Check the communication and data reception flags
        self.ConnectionTimer = QtCore.QTimer(self)
        self.ConnectionTimer.timeout.connect(self.ConnectionCallback)
        self.ConnectionTimer.start(CV_CONNECTION_CHECK_PERIOD)

        ## Redrawing the GUI
        self.RedrawTimer = QtCore.QTimer(self)
        # self.RedrawTimer.timeout.connect(self.RedrawCallback)
        # self.RedrawTimer.timeout.connect(self.CentroidDetect)
        self.RedrawTimer.start(CV_GUI_UPDATE_PERIOD)


        self.count = 0
        self.navdata = Navdata
        self.command = Twist()
        thread = Thread(target=self.SemiautonomousFlight, args=())
        self.threadLock = Lock()
        thread.daemon = True
        thread.start()

    def ConnectionCallback(self):
        self.connected = self.CommunicationSinceTimer
        self.CommunicationSinceTimer = False

    def HSVReceiveImage(self, data):
        self.CommunicationSinceTimer = True
        self.threadLock.acquire()
        try:
            self.hsv_img = data
        except CvBridgeError as e:
            print e
        finally:
            self.threadLock.release()

    def ReceiveNavdata(self, navdata):
        self.navdata = navdata
        self.vx = navdata.vx
        self.vy = navdata.vy


    def ToggleCam(self):
        rospy.wait_for_service('/ardrone/togglecam')
        try:
            self.toggle = rospy.ServiceProxy('/ardrone/togglecam', Empty)
            self.toggle_response = self.toggle()
        except rospy.ServiceException, e:
            print 'Service call failed:%s'%e


    def SemiautonomousFlight(self):
        if controller is not None:
            print "Started Semi-Autonomous Flight"
            self.threadLock.acquire()
            rate = rospy.Rate(50)
            # print "Done :%s"%(self.done)
            print "Setting the loop rate to 50 Hz"
            t_i = rospy.get_time()
            i =1
            try:
                while self.done is True:
                    # if self.status == DroneStatus.Landed or self.status is not DroneStatus.Emergency:
                    # print "Drone is about to take off"
                    while (rospy.get_time() - t_i) < 8.0:
                        controller.SendTakeoff()
                    if self.batteryPercentage <= 20:
                        print "BATTERY NOT SUFFICIENT FOR FLIGHT. LANDING THE DRONE"
                        controller.SetCommand(0.0, 0.0, 0.0, 0.0)
                        controller.SendLand()
                        self.land.publish(Empty())
                        self.done = False
                    res = self.AltitudeHold()
                    print" Altitude Hold succeded"
                    time.sleep(5)
                    self.pitch = -0.1*Kd_x * self.navdata.vx
                    self.roll = -0.1*Kd_y * self.navdata.vy
                    self.posx_error_ini = -6.1
                    self.posy_error_ini = 0

                    while res is 1 and self.pox < 6.1:
                        # [self.pox, self.poy] = self.forward_control(rate)
                        self.pox += self.navdata.vx * 0.001 / 50
                        self.poy += self.navdata.vy * 0.001 / 50
                        # # TODO: include the focal length and the sensor paranteter for the bottom camera in case of ardrone
                        # [px, py] = self.Position_correction()
                        # if px is 0 and py is 0:
                        self.pitch = -0.35 * (self.pox - 6.1) - 0.1 * Kd_x * (self.navdata.vx)  ## SCALING FACTOR FOR MM/SEC TO M/SEC 0.02 = k_D GAIN
                        self.roll = -0.5 * (self.poy - 0) - 0.1 * Kd_y * (self.navdata.vy)  # -0.0001*(self.posy_error_ini)/50
                        # else:
                        #     self.pox_ref = 6.1 - px
                        #     self.poy_ref = -py
                        #     self.pitch = -0.03 * (self.pox - self.pox_ref) - 0.1 * Kd_x * (
                        #     self.navdata.vx)  ## SCALING FACTOR FOR MM/SEC TO M/SEC 0.02 = k_D GAIN
                        #     self.roll = -0.5 * (self.poy - self.poy_ref) - 0.1 * Kd_y * (
                        #     self.navdata.vy)  # -0.0001*(self.posy_error_ini)/50
                        self.yaw_velocity = 0.0
                        self.z_velocity = -0.0005 * (self.navdata.altd - 2100)
                        self.command.linear.x = self.pitch
                        self.command.linear.y = self.roll
                        self.command.linear.z = self.z_velocity
                        self.command.angular.z = 0.0
                        self.pubCommand.publish(self.command)
                        # controller.SetCommand(0.3, 0.0, 0.0, self.z_velocity)
                        self.posx_error_ini -= (self.pox - 6.1)
                        self.posy_error_ini -= (self.poy - 0)
                        print "Posx:%.3f, Posy:%.3f, Pitch:%3f" % (self.pox, self.poy, self.pitch)
                        rate.sleep()
                    self.done = False
            except KeyboardInterrupt:
                pass
                # self.done = False
            finally:
                self.command.linear.x = 0.0
                self.command.linear.y = 0.0
                self.command.linear.z = 0.0
                self.command.angular.z = 0.0
                self.pubCommand.publish(self.command)
                time.sleep(5)
                controller.SendLand()
                # controller.pubLand.publish(Empty())
                # self.land.publish(Empty())
                self.threadLock.release()
                # controller.SendReset()
                self.done = False

    def ForwardControl(self,rate):
        self.pox += self.navdata.vx * 0.001 / 50
        self.poy += self.navdata.vy * 0.001 / 50
        # TODO: include the focal length and the sensor paranteter for the bottom camera in case of ardrone
        # [px, py] = self.Position_correction()
        # if px is 0 and py is 0:
        self.pitch = -0.35 * (self.pox - 6.1) - 0.1 * Kd_x * (self.navdata.vx)  ## SCALING FACTOR FOR MM/SEC TO M/SEC 0.02 = k_D GAIN
        self.roll = -0.5 * (self.poy - 0) - 0.1 * Kd_y * (self.navdata.vy)  # -0.0001*(self.posy_error_ini)/50
        # else:
        #     pox_ref = 6.1 - px
        #     poy_ref = -py
        #     self.pitch = -0.03 * (self.pox - poy_ref) - 0.1 * Kd_x * (self.navdata.vx)  ## SCALING FACTOR FOR MM/SEC TO M/SEC 0.02 = k_D GAIN
        #     self.roll = -0.5 * (self.poy - pox_ref) - 0.1 * Kd_y * (self.navdata.vy)  # -0.0001*(self.posy_error_ini)/50
        self.yaw_velocity = 0.0
        self.z_velocity = -0.0005 * (self.navdata.altd - 2100)
        self.command.linear.x = self.pitch
        self.command.linear.y = self.roll
        self.command.linear.z = self.z_velocity
        self.command.angular.z = 0.0
        self.pubCommand.publish(self.command)
        # controller.SetCommand(0.3, 0.0, 0.0, self.z_velocity)
        self.posx_error_ini -= (self.pox - 6.1)
        self.posy_error_ini -= (self.poy - 0)
        print "Posx:%.3f, Posy:%.3f, Pitch:%3f" % (self.pox, self.poy, self.pitch)
        rate.sleep()
        return [self.pox, self.poy]

    def ReceiveCentroids(self, centroid):
        self.CommunicationSinceTimer = True
        self.threadLock.acquire()
        try:
            if centroid.x < 0 and centroid.y <0:
                self.cent_x = 160
                self.cent_y = 120
            else:
                self.cent_x = int(centroid.x)
                self.cent_y = int(centroid.y)
        except KeyboardInterrupt as e:
            print e
        finally:
            self.threadLock.release()

    def Position_correction(self):
        cu = self.cent_x  ## The y displacement corresponds to the change in pitch ( forward or backward)
        cv = self.cent_y  ## The x displacement corresponds to the change in roll ( left or right)
        alt = self.navdata.altd
        # phi = self.navdata.rotX
        # theta = self.navdata.rotY
        # img_based_x = fx * (cu-320/alt)
        # img_based_y = fy * (cv-180/alt)
        img_based_x = (160 - cu)/alt #cu - 320
        img_based_y = (120 - cv)/alt #cv - 180
        return [img_based_x, img_based_y]
        #img_based_x = alt *(math.tan(phi - (cu * 47.5/img_height)))
        #img_based_y = alt *(math.tan(theta - (cv * 37.5/img_width)))

# #     def datawrite(self):
# #         file = open("/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorialsArdrone_params"+str(datetime.datetime.now())+".txt", 'r+')
# #
# #         while self.done is True:
# #             file.write(str(self.navdata.vx)+"%10d"+str(self.navdata.vy)+"%10d")
# #             # file.write("%10s%10s%10s%10s%10s%10s%10s%10s%10s\n"%(str(self.navdata.vx), str(self.navdata.vy), str(self.navdata.vz), str(self.pox), str(self.poy),str(self.navdata.altd), str(self.pitch), str(self.roll), str(self.yaw_velocity) ))

    def AltitudeHold(self):
        rate = rospy.Rate(50)
        time = rospy.get_time()
        self.altitude_next = self.navdata.altd
        while np.abs(self.navdata.altd - 2100) >=100:
            #TODO: Incorporate centroid based position hold in addition to current roll and pitch channel for more preceise
            #REVIEW: Done
            self.z_velocity = -0.003*(self.navdata.altd - 2100) #-0.3*(self.navdata.az/50.0)-0.7*((self.altitude_next- self.navdata.altd)*50.0)
            [image_alt_x_ini, image_alt_y_ini]= self.Position_correction()
                # [image_alt_x_new, image_alt_y_new]= self.Position_correction()
            self.pitch = -Kd_x*self.navdata.vx #- 0*0.001 * ((image_alt_x_ini))#(image_alt_x_new - image_alt_x_ini)
            self.roll = -Kd_y*self.navdata.vy #- 0*0.001 * ((image_alt_y_ini))#(image_alt_y_new - image_alt_y_ini)
                # self.image_alt_x_ini = image_alt_x_new
                # self.image_alt_y_ini = image_alt_y_new
            self.yaw_velocity = 0
            controller.SetCommand(self.pitch, self.roll, self.yaw_velocity, self.z_velocity)
            print "Altitude:%d, Cx:%d, Cy:%d, cen_x:%d, cen_y:%d" %(self.navdata.altd, image_alt_x_ini, image_alt_y_ini, self.cent_x, self.cent_y)
            self.altitude_next = self.navdata.altd
            rate.sleep()
        return 1

    def batteryPercentage(self, battpercent):
        self.batteryPercentage = int(battpercent.data)


if __name__ == '__main__':
    import sys
    rospy.init_node('ARDrone_semi_autonomus_controller')
    app = QtGui.QApplication(sys.argv)
    # controller = BasicDroneController()
    controller = BasicDroneController()
    display = ImageBasedServoing()
    display.img_subscriber = rospy.Subscriber('/ardrone/image_raw', Image, display.HSVReceiveImage)
    display.HSVReceiveImage(display.hsv_img)
    display.img_publisher = rospy.Publisher('/ardrone/image_raw', Image, queue_size=230400)
    display.img_publisher.publish(display.hsv_img)
    display.show()
    print "Togglecount:::%s"%(display.count)
    status = app.exec_()
    rospy.signal_shutdown('Great Flying')
    sys.exit(app.exec_())