#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/aniketrs/ardrone_catkin_ws/src/ardrone_tutorials/src/')
import rospy
# import signal
import pygame
# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
# from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui
import threading
import time
from std_srvs.srv import Empty
from ardrone_autonomy import *
from std_msgs.msg import Int8
import numpy as np
# from ardrone_autonomy.msg import Navdata
Kd_x = 0.005
Kd_y = 0.005

class XboxController(object):
    def __init__(self):
        super(XboxController, self).__init__()
        pygame.init()
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

        self.done = False
        self.running = True
        thread = threading.Thread(target=self.StickActiveEvent, args=())
        thread.daemon = True
        thread.start()
        self.threadLock = threading.Lock()
        self.count = 0
        self.pub_togglecount = rospy.Publisher('/ardrone/togglecount', Int8, queue_size=10)
        self.batt_percent = rospy.Subscriber('/ardrone/battpercent', Int8, self.batteryPercentage)
        # self.batteryPercentage = Int8

    def ToggleCam(self):
        rospy.wait_for_service('/ardrone/togglecam')
        try:
            self.toggle = rospy.ServiceProxy('/ardrone/togglecam', Empty)
            self.toggle_response = self.toggle()
        except rospy.ServiceException, e:
            print 'Service call failed:%s'%e


    def StickActiveEvent(self):
        # print 'Starting'
        if controller is not None:
            print 'Controller is running'
            self.threadLock.acquire()
            try:
                while self.done is False:
                    if self.batteryPercentage <= 10:
                        controller.SetCommand(0.0, 0.0, 0.0, 0.0)
                        controller.SendLand()
                        print "Not Enough Battery to take off. Landing Initiated"
                    else:
                        stick_key = pygame.event.get()
                        # print 'Running While'
                        for x in stick_key:
                        ## Default cases for safety while flying
                            if x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 7:
                                """ Press the start button for making the drone takeoff """
                                controller.SendTakeoff()
                                print 'Start Pressed and Taking Off'
                            elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 6:
                                """Press the back button for setting the mode to emergency"""
                                controller.SendEmergency()
                                print 'Emergency Detected and landing'
                            elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 8:
                                """Press the Back button on the controller for landing the parrot"""
                                controller.SendLand()
                                print 'Landing the Ardrone'
                            elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 5:
                                """ Calling the Toggle camera service"""
                                self.ToggleCam()
                                # COUNT += 1
                                self.count +=1
                                print 'Toggle service called %s times'%(self.count)
                                self.pub_togglecount.publish(self.count)
                            elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 0:
                                """Send the yaw angle rate"""
                                if -0.9 < x.dict['value'] < -0.2:
                                    self.yaw_velocity += x.dict['value']*-1.0
                                    # print 'Yaw Value=%f' % self.yaw_velocity
                                elif 0.2 < x.dict['value'] < 0.9:
                                    self.yaw_velocity += x.dict['value']*-1.0
                                    # print 'Yaw Value=%f' % self.yaw_velocity
                                else:
                                    self.yaw_velocity = 0.0
                                    # print 'Yaw Value= %f' % self.yaw_velocity
                            elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 4:
                                """ Send the pitch angle rate """
                                if -0.9 < x.dict['value'] < -0.2:
                                    self.pitch += x.dict['value']*-1.0
                                    # print 'Pitch Value=%f' % self.pitch
                                elif 0.2 < x.dict['value'] < 0.9:
                                    self.pitch += -1.0*x.dict['value']
                                    # print 'Pitch Value=%f' % self.pitch
                                else:
                                    self.pitch = 0.0
                                    # print 'Pitch value=%f' % self.pitch
                            elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 3:
                                """ Send the roll angle rate """
                                if -0.9 < x.dict['value'] < -0.2:
                                    self.roll += -1.0 * x.dict['value']
                                    # print 'Roll value=%f' % self.roll
                                elif 0.2 < x.dict['value'] < 0.9:
                                    self.roll += -1.0 * x.dict['value']
                                    # print 'Roll value=%f' % self.roll
                                else:
                                    self.roll = 0.0
                                    # print 'Roll value=%f' % self.roll
                            elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 1:
                                """ Send the thrust rate """
                                if -0.9 < x.dict['value'] < -0.2:
                                    # print 'Thrust=%f'%(x.dict['value'])
                                    self.z_velocity += x.dict['value']*-1.0
                                elif 0.2 < x.dict['value'] < 0.9:
                                    # print 'Thrust=%f'%(x.dict['value'])
                                    self.z_velocity += x.dict['value']*-1.0
                                else:
                                    self.z_velocity = 0.0
                        joystick = pygame.joystick.Joystick(0)
                        joystick.init()
                        controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

            except KeyboardInterrupt:
                self.done = True
                self.running = False
                print 'Pygame failed due to %s'%(pygame.get_error())
                sys.exit()
            finally:
                self.threadLock.release()

    def batteryPercentage(self, battpercent):
        self.batteryPercentage = int(battpercent.data)


if __name__ == '__main__':
    import sys
    rospy.init_node('ARDrone_xbox_controller')
    app = QtGui.QApplication(sys.argv)
    # controller = BasicDroneController()
    controller = BasicDroneController()
    display = XboxController()
    # display.show()
    print "Togglecount:::%s"%(display.count)
    status = app.exec_()
    rospy.signal_shutdown('Great Flying')
    sys.exit(app.exec_())
