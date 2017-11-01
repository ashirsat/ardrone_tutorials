    #!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import rospy
import roslib; roslib.load_manifest('ardrone_tutorials')
import pygame
# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
# from drone_CONTROLLER2 import BasicDroneController2
from drone_video_display import DroneVideoDisplay
# from drone_video_DISPLAY2 import DroneVideoDisplay2
# from geometry_msgs.msg import Twist
# Finally the GUI libraries
from PySide import QtCore, QtGui
import threading
import time
from std_srvs.srv import Empty
from ardrone_autonomy import *

import signal
from drone_status import DroneStatus
# from std_srvs.srv import Empty
# import xbox
# joy = xbox.Joystick()
# class StickMapping(object):
#     def __init__(self):
        # pygame.init()
        # pygame.joystick.init()
        # self.joystick = pygame.joystick.Joystick()
    # Pitch = selfjoystick.get_axis(4) ## Left stick y axis
    # # PitchBackward = QtCore.Qt.Key.Key_D
    # Roll =pygame.joystick.get_axis(3) ## Right stick x axis
    # # RollRight = QtCore.Qt.Key.Key_F
    # Yaw = pygame.joystick.get_axis(0) ## Left Stick x axis
    # # YawRight = QtCore.Qt.Key.Key_R
    # Altitude = pygame.joystick.get_axis(1) ## Right Stick y axis
    # # DecreaseAltitude = QtCore.Qt.Key.Key_A
    # Takeoff = pygame.joystick.get_button(7) ## Start Button
    # Land = pygame.joystick.get_button(6) ## Back Button
    # Emergency = pygame.joystick.get_button(2) ## X Button


class XboxController(DroneVideoDisplay):
    def __init__(self):
        super(XboxController, self).__init__()
        pygame.init()
        # self.joystick = pygame.joystick.Joystick(1)
        # self.joystick.init()
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0
        # self.twist = Twist()
        # self.SubsTwist = rospy.Subscriber('/ardrone/cmd_vel')
        # self.PubVideo = rospy.Publisher('/ardrone/front/image_raw', Image, )
        # self.SerToggle = rospy.ServiceProxy('/ardrone/togglecam', Empty)
        self.done = False
        self.running = True

        thread = threading.Thread(target=self.StickActiveEvent, args=())
        thread.daemon = True
        thread.start()
        self.threadLock = threading.Lock()
    def run(self):
        while self.running:
            print 'Doing something in the background'
            time.sleep(0.5)
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
                # print 'Entered try'
                while self.done is False:
                # for i in range(1,20,1):
                    # print 'entered try loop'
                    stick_key = pygame.event.get()
                    # print 'Running While'
                    for x in stick_key:
                ## Default cases for safety while flying
                        if x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 7:
                            ## Press the start button for making the drone takeoff
                            controller.SendTakeoff()
                            # controller2.SendTakeoff2()
                            print 'Start Pressed and Taking Off'
                        elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 6:
                            ## Press the back button for setting the mode to emergency
                            controller.SendEmergency()
                            # controller2.SendEmergency2()
                            print 'Emergency Detected and landing'
                        elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 8:
                            ## Press the Back button on the controller for landing the parrot
                            controller.SendLand()
                            # controller2.SendLand2()
                            print 'Landing the Ardrone'
                        # elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 1:
                        #     # controller.SendEmergency()
                        #     print 'B Pressed'
                        # elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 2:
                        #     print 'X Pressed'
                        # elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 3:
                        #     print 'Y Pressed'
                        # elif x.type == pygame.JOYBUTTONDOWN and x.dict['button'] is 0:
                        #     controller.SendTakeoff()
                        #     print 'A Pressed'
                        # # elif x.type == pygame.JOYBUTTONDOWN and x.dict['button'] == 6:
                        # #     controller.SendLand()
                        # #     print 'Drone Landing'
                        # elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 1:
                        #     print 'Thrust Value=%f' %x.dict['value']
                        elif x.type is pygame.JOYBUTTONDOWN and x.dict['button'] is 5:
                            self.ToggleCam()

                        elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 0:  #or x.dict['value'] > 0.2 or x.dict['value'] <-0.2 or x.dict['value'] < 0.8 or x.dict['value'] > -0.8:
                            ## Send the yaw angle rate
                            # pass
                            if -0.9 < x.dict['value'] < -0.3:
                                self.yaw_velocity += x.dict['value']*-1.0
                                # print 'Yaw Value=%f' % self.yaw_velocity
                                # self.yaw_velocity = x.dict['value']
                            elif 0.3 < x.dict['value'] < 0.8:
                                self.yaw_velocity += x.dict['value']*-1.0
                                # print 'Yaw Value=%f' % self.yaw_velocity
                            else:
                                self.yaw_velocity = 0.0
                                # print 'Yaw Value= %f' % self.yaw_velocity
                        # elif x.dict['value'] < -0.2 and x.dict['value'] > -0.8:
                        #         # self.yaw_velocity = x.dict['value']
                        #         print 'Yaw Value=%f' % x.dict['value']
                        #     else:
                        #         # self.yaw_velocity = 0
                        #         print 'Yaw Value=%f' %x.dict['value']
                        #     # pass
                            # print type(x.dict['value'])
                        elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 4:
                            # pass
                            if -0.9 < x.dict['value'] < -0.3:
                                self.pitch += x.dict['value']*-1.0
                                # print 'Pitch Value=%f' % self.pitch
                            elif 0.3 < x.dict['value'] < 0.9:
                                self.pitch += -1.0*x.dict['value']
                                # print 'Pitch Value=%f' % self.pitch
                            else:
                                self.pitch = 0.0
                                # print 'Pitch value=%f' % self.pitch

                            # self.pitch = -1* x.dict['value']
                            # print 'Pitch Value=%f'%(-1*x.dict['value'])
                            # pass
                            # print type(x.dict['value']*-1.0)
                        elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 3:
                            # pass
                            if -0.9 < x.dict['value'] < -0.3:
                                self.roll += -1.0 * x.dict['value']
                                # print 'Roll value=%f' % self.roll
                            elif 0.3 < x.dict['value'] < 0.9:
                                self.roll += -1.0 * x.dict['value']
                                # print 'Roll value=%f' % self.roll
                            else:
                                self.roll = 0.0
                                # print 'Roll value=%f' % self.roll

                            # self.roll = x.dict['value']
                            # print 'Roll Value=%f'% x.dict['value']
                            # pass
                            # print type(x.dict['value'])
                        elif x.type == pygame.JOYAXISMOTION and x.dict['axis'] == 1:
                            if -0.5 < x.dict['value'] < -0.3:
                                print 'Thrust=%f'%(x.dict['value'])
                                # pass
                                self.z_velocity = x.dict['value']*-1.0
                            elif 0.3 < x.dict['value'] < 0.5:
                                print 'Thrust=%f'%(x.dict['value'])
                                # pass
                                self.z_velocity = x.dict['value']*-1.0
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
                    # controller2.SetCommand2(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
                    # print "Thrust=%f, Roll=%f, Yaw=%f, Pitch=%f" % (self.z_velocity, self.roll, self.yaw_velocity, self.pitch)
                    # print i
            except KeyboardInterrupt:
                self.done = True
                self.running = False
                print 'Pygame failed due to %s'%(pygame.get_error())
                sys.exit()
            finally:
                # controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
                self.threadLock.release()
                # pass
                # else:
                #     if stick_key == StickMapping.Yaw:
                #         if Yaw > 0:
                #             self.yaw_velocity += 1
                #         elif Yaw < 0:
                #             self.yaw_velocity += -1
                #         else:
                #             self.yaw_velocity = 0
                #
                #     if stick_key == StickMapping.Roll:
                #         if joy.leftX > 0:
                #             self.roll += 1
                #         elif joy.leftX < 0:
                #             self.roll += -1
                #         else:
                #             self.roll = 0
                #
                #     if stick_key == StickMapping.Pitch:
                #         if joy.leftY > 0:
                #             self.pitch += 1
                #         elif joy.leftY < 0:
                #             self.pitch += -1
                #         else:
                #             self.pitch = 0
                #
                #     if stick_key == StickMapping.Altitude:
                #         if joy.rightY > 0:
                #             self.z_velocity += 1
                #         elif joy.rightY < 0:
                #             self.z_velocity += -1
                #         else:
                #             self.z_velocity = 0



    # def StickDeactiveEvent(self, event):
    #     stick_key = event.key()
    #     if controller is not None and not event.isAutorepeat():
    #         if stick_key == StickMapping.Yaw:
    #             if joy.rightX > 0:
    #                 self.yaw_velocity -= 1
    #             elif joy.rightX < 0:
    #                 self.yaw_velocity -= -1
    #             else:
    #                 self.yaw_velocity = 0
    #
    #         if stick_key == StickMapping.Roll:
    #             if joy.leftX > 0:
    #                 self.roll -= 1
    #             elif joy.leftX < 0:
    #                 self.roll -= -1
    #             else:
    #                 self.roll = 0
    #
    #         if stick_key == StickMapping.Pitch:
    #             if joy.leftY > 0:
    #                 self.pitch -= 1
    #             elif joy.leftY <0:
    #                 self.pitch -= -1
    #             else:
    #                 self.pitch = 0
    #
    #         if stick_key == StickMapping.Altitude:
    #             if joy.rightY > 0:
    #                 self.z_velocity -= 1
    #             elif joy.rightY < 0:
    #                 self.z_velocity -= -1
    #             else:
    #                 self.z_velocity = 0
    #
    #         controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

if __name__ == '__main__':
    import sys
    rospy.init_node('ardrone_xbox_controller')
    app = QtGui.QApplication(sys.argv)
    controller = BasicDroneController()
    display = XboxController()
    # display.show()
    # display.StickActiveEvent()
    status = app.exec_()
    # print status
    rospy.signal_shutdown('Great Flying')
    sys.exit(status)
