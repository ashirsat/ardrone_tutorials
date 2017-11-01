#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib;

roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  # for sending commands to the drone
from std_msgs.msg import Empty  # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata  # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_STATUS2 import DroneStatus2

# Some Constants
COMMAND_PERIOD = 100  # ms


class BasicDroneController2(object):
    def __init__(self):
        # Holds the current drone status
        self.status2 = -1

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata2 = rospy.Subscriber('/Alpha2/ardrone/navdata', Navdata, self.ReceiveNavdata2)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand2 = rospy.Publisher('/Alpha2/ardrone/land', Empty, queue_size=1)

        self.pubTakeoff2 = rospy.Publisher('/Alpha2/ardrone/takeoff', Empty, queue_size=1)

        self.pubReset2 = rospy.Publisher('/Alpha2/ardrone/reset', Empty, queue_size=1)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand2 = rospy.Publisher('/Alpha2/cmd_vel', Twist, queue_size=1)

        # Setup regular publishing of control packets
        self.command2 = Twist()
        self.commandTimer2 = rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000.0), self.SendCommand2)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand2)

    def ReceiveNavdata2(self, navdata):
        # Although there is a lot of data in this packet, we're only interested in the state at the moment
        self.status2 = navdata.state

    def SendTakeoff2(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if (self.status2 == DroneStatus2.Landed):
            self.pubTakeoff2.publish(Empty())

    def SendLand2(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand2.publish(Empty())

    def SendEmergency2(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset2.publish(Empty())

    def SetCommand2(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        # Called by the main program to set the current command
        self.command2.linear.x = pitch
        self.command2.linear.y = roll
        self.command2.linear.z = z_velocity
        self.command2.angular.z = yaw_velocity

    def SendCommand2(self, event):
        # The previously set command is then sent out periodically if the drone is flying
        # if ((self.status1 == DroneStatus.Flying) and (self.status2 is DroneStatus.Flying)) or ((self.status1 == DroneStatus.GotoHover) and (self.status2 is DroneStatus.GotoHover)) or ((self.status1 == DroneStatus.Hovering) and (self.status2 is DroneStatus.Hovering)):
        if self.status2 == DroneStatus2.Flying or self.status2 == DroneStatus2.GotoHover or self.status2 == DroneStatus2.Hovering:
            self.pubCommand2.publish(self.command2)


