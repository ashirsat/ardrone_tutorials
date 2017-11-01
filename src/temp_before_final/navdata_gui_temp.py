#!/usr/bin/env python

# import roslib
import sys
# roslib.load_manifest('ardrone_tutorials')
# import rospy
from PyQt4 import QtGui, QtCore
from threading import Lock
# from ardrone_autonomy import *
# from std_srvs.srv import Empty
# from sensor_msgs.msg import Imu
# from ardrone_autonomy.msg import Navdata
# from geometry_msgs.msg import Vector3Stamped, Twist, Transform
## GUI Constants
import rospy
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu
from threading import Lock, Thread
CONNECTION_CHECK_PERIOD = 250 #MS
GUI_UPDATE_PERIOD = 20 #MS

class Navdata_GUI(QtGui.QMainWindow):
    def __init__(self):
        super(Navdata_GUI, self).__init__()
        self.initGUI()

    def initGUI(self):
        """ GUI Display code"""
        self.navdata_widget = QtGui.QWidget()
        self.setWindowTitle('AR Drone Navdata HUD')
        self.navdatabox = QtGui.QLabel(self)
        self.setCentralWidget(self.navdatabox)
        self.setGeometry(0, 0, 640, 360)
        self.base_x_cord = 20
        self.base_y_cord = 10
        self.box_width = 50
        self.box_height = 20
        self.font_style = 'SansSerif'
        self.font_weight = 10
        self.data_lock = Lock()
        self.CommunicationSinceTimer = False
        self.connected = False

        ## Checking the data connection and data reception flags
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        ## Redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

        """ Setting up subscribers for the navdata and the imu topic
        """
        self.navdata_Subs = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)
        self.imu_Subs = rospy.Subscriber('/ardrone/imu', Imu, self.ReceiveImudata)
        self.navdata = Navdata()
        self.imu = Imu()

        """ Defining varibales for the different  sensor readings"""
        """ ardrone/navadata readings"""
        self.navdata_battery_percent_reading = ''
        self.navdata_drone_state_reading = ''
        self.navdata_mag_x_reading = ''
        self.navdata_mag_y_reading = ''
        self.navdata_mag_z_reading = ''
        self.navdata_rot_x_reading = ''
        self.navdata_rot_y_reading = ''
        self.navdata_rot_z_reading = ''
        self.navdata_vx_reading = ''
        self.navdata_vy_reading = ''
        self.navdata_vz_reading = ''
        self.navdata_ax_reading = ''
        self.navdata_ay_reading = ''
        self.navdata_az_reading = ''
        self.navdata_motor1_reading = ''
        self.navdata_motor2_reading = ''
        self.navdata_motor3_reading = ''
        self.navdata_altitude = ''
        self.navdata_motor4_reading = ''

        """ ardrone/imu readings"""
        self.imu_orientaion_x_reading = ''
        self.imu_orientaion_y_reading = ''
        self.imu_orientaion_z_reading = ''
        self.imu_ang_vel_x_reading = ''
        self.imu_ang_vel_y_reading = ''
        self.imu_ang_vel_z_reading = ''



        """ Defining the magentometer messages"""
        self.navdata_mag_box_x = QtGui.QLabel('Mag X', self)
        self.navdata_mag_box_x.setGeometry(self.base_x_cord, self.base_y_cord, self.box_width,
                                   self.box_height) ## (20, 10, 50, 20)
        # self.mag_box_x.resize(self.mag_box_x.sizeHint())
        self.navdata_mag_box_x.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_mag_box_y = QtGui.QLabel('Mag Y', self)
        self.navdata_mag_box_y.setGeometry(self.base_x_cord, self.base_y_cord + self.box_height, self.box_width,
                                   self.box_height) ## (20, 30, 50, 20)
        # self.mag_box_y.resize(self.mag_box_y.sizeHint())
        self.navdata_mag_box_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_mag_box_z = QtGui.QLabel('Mag Z', self)
        self.navdata_mag_box_z.setGeometry(self.base_x_cord, self.base_y_cord + (2*(self.box_height)), self.box_width,
                                   self.box_height) ## (20, 50, 50, 20)
        # self.mag_box_z.resize(self.mag_box_z.sizeHint())
        self.navdata_mag_box_z.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the various texbox for mag readings"""
        self.navdata_mag_box_x_read = QtGui.QLineEdit(self)
        self.navdata_mag_box_x_read.setGeometry(self.base_x_cord + self.box_width, self.base_y_cord, self.box_width,
                                        self.box_height) ## (80, self.font_weight, 50, 20)
        self.navdata_mag_box_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_mag_box_x_read.setText("yup")

        self.navdata_mag_box_y_read = QtGui.QLineEdit(self)
        self.navdata_mag_box_y_read.setGeometry(self.base_x_cord + self.box_width, self.base_y_cord + self.box_height,
                                                self.box_width, self.box_height) ## (80, 30, 50, 20)
        self.navdata_mag_box_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_mag_box_y_read.setText("yup")

        self.navdata_mag_box_z_read = QtGui.QLineEdit(self)
        self.navdata_mag_box_z_read.setGeometry(self.base_x_cord + self.box_width,
                                                self.base_y_cord + (2*(self.box_height)), self.box_width,
                                                self.box_height) ## (80, 50, 50, 20)
        self.navdata_mag_box_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_mag_box_z_read.setText("yup")

        ################################################################################################################
        """ Defining the imu/orientation messages"""
        self.imu_base_x_cord = self.base_x_cord + 2*self.box_width + 10
        self.imu_base_y_cord = self.base_y_cord
        # self.imu_box_offset = 50

        self.imu_box_x = QtGui.QLabel('Roll', self)
        self.imu_box_x.setGeometry(self.imu_base_x_cord, self.imu_base_y_cord, self.box_width,
                                   self.box_height)  ## (20, 10, 50, 20)
        # self.mag_box_x.resize(self.mag_box_x.sizeHint())
        self.imu_box_x.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_box_y = QtGui.QLabel('Pitch', self)
        self.imu_box_y.setGeometry(self.imu_base_x_cord, self.imu_base_y_cord + self.box_height, self.box_width,
                                   self.box_height)  ## (20, 30, 50, 20)
        # self.mag_box_y.resize(self.mag_box_y.sizeHint())
        self.imu_box_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_box_z = QtGui.QLabel('Yaw', self)
        self.imu_box_z.setGeometry(self.imu_base_x_cord, self.imu_base_y_cord + (2 * (self.box_height)), self.box_width,
                                   self.box_height)  ## (20, 50, 50, 20)
        # self.mag_box_z.resize(self.mag_box_z.sizeHint())
        self.imu_box_z.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the various texbox for imu/orientation readings"""
        self.imu_box_x_read = QtGui.QLineEdit(self)
        self.imu_box_x_read.setGeometry(self.imu_base_x_cord + self.box_width, self.imu_base_y_cord, self.box_width,
                                        self.box_height)  ## (80, 10, 50, 20)
        self.imu_box_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_box_x_read.setText("yup")

        self.imu_box_y_read = QtGui.QLineEdit(self)
        self.imu_box_y_read.setGeometry(self.imu_base_x_cord + self.box_width, self.imu_base_y_cord + self.box_height,
                                        self.box_width, self.box_height)  ## (80, 30, 50, 20)
        self.imu_box_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_box_y_read.setText("yup")

        self.imu_box_z_read = QtGui.QLineEdit(self)
        self.imu_box_z_read.setGeometry(self.imu_base_x_cord + self.box_width,
                                        self.imu_base_y_cord + (2 * (self.box_height)),self.box_width,
                                        self.box_height)  ## (80, 50, 50, 20)
        self.imu_box_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_box_z_read.setText("yup")

        ################################################################################################################
        """ Defining imu/angular velocity messages """
        self.imu_ang_vel_base_x_cord = self.imu_base_x_cord + 2*self.box_width + 10
        self.imu_ang_vel_base_y_cord = self.imu_base_y_cord

        self.imu_ang_vel_x = QtGui.QLabel('Roll Rate', self)
        self.imu_ang_vel_x.setGeometry(self.imu_ang_vel_base_x_cord, self.imu_ang_vel_base_y_cord,
                                       self.box_width, self.box_height)
        self.imu_ang_vel_x.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_ang_vel_y = QtGui.QLabel('Pitch Rate', self)
        self.imu_ang_vel_y.setGeometry(self.imu_ang_vel_base_x_cord, self.imu_ang_vel_base_y_cord + self.box_height,
                                       self.box_width, self.box_height)
        self.imu_ang_vel_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_ang_vel_z = QtGui.QLabel('Yaw Rate', self)
        self.imu_ang_vel_z.setGeometry(self.imu_ang_vel_base_x_cord, self.imu_ang_vel_base_y_cord + 2*self.box_height,
                                       self.box_width, self.box_height)
        self.imu_ang_vel_z.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining imu/angular velocity readings """
        self.imu_ang_vel_x_read = QtGui.QLineEdit(self)
        self.imu_ang_vel_x_read.setGeometry(self.imu_ang_vel_base_x_cord + self.box_width,
                                            self.imu_ang_vel_base_y_cord, self.box_width, self.box_height)  ## (80, self.font_weight, 50, 20)
        self.imu_ang_vel_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_ang_vel_x_read.setText("yup")

        self.imu_ang_vel_y_read = QtGui.QLineEdit(self)
        self.imu_ang_vel_y_read.setGeometry(self.imu_ang_vel_base_x_cord + self.box_width,
                                            self.imu_ang_vel_base_y_cord + self.box_height, self.box_width,
                                            self.box_height)  ## (80, 30, 50, 20)
        self.imu_ang_vel_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_ang_vel_y_read.setText("yup")

        self.imu_ang_vel_z_read = QtGui.QLineEdit(self)
        self.imu_ang_vel_z_read.setGeometry(self.imu_ang_vel_base_x_cord + self.box_width,
                                        self.imu_ang_vel_base_y_cord + (2 * (self.box_height)), self.box_width,
                                        self.box_height)  ## (80, 50, 50, 20)
        self.imu_ang_vel_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_ang_vel_z_read.setText("yup")

        ################################################################################################################

        """ Defining the /ardrone/navdata linear accelerations messages """
        self.navdata_lin_accn_base_x_cord = self.imu_ang_vel_base_x_cord + 2*self.box_width + 10
        self.navdata_lin_accn_base_y_cord = self.imu_ang_vel_base_y_cord

        self.navdata_lin_accn_x = QtGui.QLabel('Accn in X', self)
        self.navdata_lin_accn_x.setGeometry(self.navdata_lin_accn_base_x_cord, self.navdata_lin_accn_base_y_cord,
                                            self.box_width, self.box_height)
        self.navdata_lin_accn_x.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_lin_accn_y = QtGui.QLabel('Accn in Y', self)
        self.navdata_lin_accn_y.setGeometry(self.navdata_lin_accn_base_x_cord,
                                            self.navdata_lin_accn_base_y_cord + self.box_height, self.box_width,
                                            self.box_height)
        self.navdata_lin_accn_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_lin_accn_z = QtGui.QLabel('Accn in Z', self)
        self.navdata_lin_accn_z.setGeometry(self.navdata_lin_accn_base_x_cord,
                                            self.navdata_lin_accn_base_y_cord + 2*self.box_height, self.box_width,
                                            self.box_height)
        self.navdata_lin_accn_z.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the /ardrone/navdata linnear accleration readings """
        self.navdata_lin_accn_x_read = QtGui.QLineEdit(self)
        self.navdata_lin_accn_x_read.setGeometry(self.navdata_lin_accn_base_x_cord + self.box_width,
                                             self.navdata_lin_accn_base_y_cord, self.box_width, self.box_height)  ## (80, self.font_weight, 50, 20)
        self.navdata_lin_accn_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_lin_accn_x_read.setText("yup")

        self.navdata_lin_accn_y_read = QtGui.QLineEdit(self)
        self.navdata_lin_accn_y_read.setGeometry(self.navdata_lin_accn_base_x_cord + self.box_width,
                                             self.navdata_lin_accn_base_y_cord + self.box_height,
                                        self.box_width, self.box_height)  ## (80, 30, 50, 20)
        self.navdata_lin_accn_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_lin_accn_y_read.setText("yup")

        self.navdata_lin_accn_z_read = QtGui.QLineEdit(self)
        self.navdata_lin_accn_z_read.setGeometry(self.navdata_lin_accn_base_x_cord + self.box_width,
                                        self.navdata_lin_accn_base_y_cord + (2 * (self.box_height)), self.box_width,
                                        self.box_height)  ## (80, 50, 50, 20)
        self.navdata_lin_accn_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_lin_accn_z_read.setText("yup")

        ################################################################################################################

        """ Defining the /ardrone/navdata rotation angles in deg """
        self.navdata_rot_x_base_cord = self.navdata_lin_accn_base_x_cord + 2*self.box_width +10
        self.navdata_rot_y_base_cord = self.navdata_lin_accn_base_y_cord

        self.navdata_rot_x = QtGui.QLabel('Rot X', self)
        self.navdata_rot_x.setGeometry(self.navdata_rot_x_base_cord, self.navdata_rot_y_base_cord, self.box_width,
                                       self.box_height)
        self.navdata_rot_x.setFont(QtGui.QFont(self.font_style, self.font_weight))
        #
        self.navdata_rot_y = QtGui.QLabel('Rot Y', self)
        self.navdata_rot_y.setGeometry(self.navdata_rot_x_base_cord, self.navdata_rot_y_base_cord + self.box_height,
                                       self.box_width, self.box_height)
        self.navdata_rot_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_rot_z = QtGui.QLabel('Rot Z', self)
        self.navdata_rot_z.setGeometry(self.navdata_rot_x_base_cord ,self.navdata_rot_y_base_cord + 2*self.box_height,
                                   self.box_width, self.box_height)

        """ Defining the imu/rotation readings"""
        self.navdata_rot_x_read = QtGui.QLineEdit(self)
        self.navdata_rot_x_read.setGeometry(self.navdata_rot_x_base_cord + self.box_width,
                                             self.navdata_rot_y_base_cord, self.box_width,
                                             self.box_height)  ## (80, 10, 50, 20)
        self.navdata_rot_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_rot_x_read.setText("yup")

        self.navdata_rot_y_read = QtGui.QLineEdit(self)
        self.navdata_rot_y_read.setGeometry(self.navdata_rot_x_base_cord + self.box_width,
                                             self.navdata_rot_y_base_cord + self.box_height,
                                             self.box_width, self.box_height)  ## (80, 30, 50, 20)
        self.navdata_rot_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_rot_y_read.setText("yup")

        self.navdata_rot_z_read = QtGui.QLineEdit(self)
        self.navdata_rot_z_read.setGeometry(self.navdata_rot_x_base_cord + self.box_width,
                                             self.navdata_rot_y_base_cord + (2 * (self.box_height)), self.box_width,
                                             self.box_height)  ## (80, 50, 50, 20)
        self.navdata_rot_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_rot_z_read.setText("yup")

        ################################################################################################################

        """ Defining the imu/orientation positions """
        self.imu_orientation_base_x_cord = self.base_x_cord
        self.imu_orientation_base_y_cord = self.base_y_cord + 2*self.box_height + 20

        self.imu_orientation = QtGui.QLabel('Orientation Quaternion', self)
        self.imu_orientation.setGeometry(self.imu_orientation_base_x_cord, self.imu_orientation_base_y_cord,
                                         2*self.box_width+50, self.box_height)
        self.imu_orientation.setFont(QtGui.QFont(self.font_style, self.font_weight,  weight=QtGui.QFont.Bold))

        self.imu_orientation_x = QtGui.QLabel('X', self)
        self.imu_orientation_x.setGeometry(self.imu_orientation_base_x_cord, self.imu_orientation_base_y_cord +
                                           self.box_height,self.box_width, self.box_height)
        self.imu_orientation_x.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_orientation_y = QtGui.QLabel('Y', self)
        self.imu_orientation_y.setGeometry(self.imu_orientation_base_x_cord,
                                           self.imu_orientation_base_y_cord + 2*self.box_height, self.box_width,
                                           self.box_height)
        self.imu_orientation_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_orientation_z = QtGui.QLabel('Z', self)
        self.imu_orientation_z.setGeometry(self.imu_orientation_base_x_cord,
                                           self.imu_orientation_base_y_cord + 3 * self.box_height, self.box_width,
                                           self.box_height)
        self.imu_orientation_z.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.imu_orientation_w = QtGui.QLabel('W', self)
        self.imu_orientation_w.setGeometry(self.imu_orientation_base_x_cord,
                                           self.imu_orientation_base_y_cord + 4 * self.box_height, self.box_width,
                                           self.box_height)
        self.imu_orientation_w.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the imu/orientation position readings """
        self.imu_orientation_x_read = QtGui.QLineEdit(self)
        self.imu_orientation_x_read.setGeometry(self.imu_orientation_base_x_cord + self.box_width,
                                                self.imu_orientation_base_y_cord + self.box_height,
                                                self.box_width, self.box_height)
        self.imu_orientation_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_orientation_x_read.setText("yup")

        self.imu_orientation_y_read = QtGui.QLineEdit(self)
        self.imu_orientation_y_read.setGeometry(self.imu_orientation_base_x_cord + self.box_width,
                                                self.imu_orientation_base_y_cord + 2*self.box_height,
                                                self.box_width, self.box_height)
        self.imu_orientation_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_orientation_y_read.setText("yup")

        self.imu_orientation_z_read = QtGui.QLineEdit(self)
        self.imu_orientation_z_read.setGeometry(self.imu_orientation_base_x_cord + self.box_width,
                                                self.imu_orientation_base_y_cord + 3*self.box_height,
                                                self.box_width, self.box_height)
        self.imu_orientation_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_orientation_z_read.setText("yup")

        self.imu_orientation_w_read = QtGui.QLineEdit(self)
        self.imu_orientation_w_read.setGeometry(self.imu_orientation_base_x_cord + self.box_width,
                                                self.imu_orientation_base_y_cord + 4*self.box_height,
                                                self.box_width, self.box_height)
        self.imu_orientation_w_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.imu_orientation_w_read.setText("yup")

        ################################################################################################################
        """ Defining navdata/linear_velocities """
        self.navdata_linear_vel_base_x_cord = self.imu_base_x_cord
        self.navdata_linear_vel_base_y_cord = self.imu_base_y_cord + 3*self.box_height + 20

        self.navdata_linear_vel_x = QtGui.QLabel('X dot', self)
        self.navdata_linear_vel_x.setGeometry(self.navdata_linear_vel_base_x_cord, self.navdata_linear_vel_base_y_cord,
                                          self.box_width, self.box_height)
        self.navdata_linear_vel_x.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_linear_vel_y = QtGui.QLabel('Y dot', self)
        self.navdata_linear_vel_y.setGeometry(self.navdata_linear_vel_base_x_cord,
                                              self.navdata_linear_vel_base_y_cord + self.box_height, self.box_width,
                                              self.box_height)
        self.navdata_linear_vel_y.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_linear_vel_z = QtGui.QLabel('Z dot', self)
        self.navdata_linear_vel_z.setGeometry(self.navdata_linear_vel_base_x_cord,
                                              self.navdata_linear_vel_base_y_cord + 2*self.box_height, self.box_width,
                                              self.box_height)
        self.navdata_linear_vel_z.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the navdata/linear_velocity readings """
        self.navdata_linear_vel_x_read = QtGui.QLineEdit(self)
        self.navdata_linear_vel_x_read.setGeometry(self.navdata_linear_vel_base_x_cord + self.box_width,
                                                   self.navdata_linear_vel_base_y_cord, self.box_width, self.box_height)
        self.navdata_linear_vel_x_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_linear_vel_x_read.setText("yup")

        self.navdata_linear_vel_y_read = QtGui.QLineEdit(self)
        self.navdata_linear_vel_y_read.setGeometry(self.navdata_linear_vel_base_x_cord + self.box_width,
                                               self.navdata_linear_vel_base_y_cord + self.box_height,
                                               self.box_width, self.box_height)
        self.navdata_linear_vel_y_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_linear_vel_y_read.setText("yup")

        self.navdata_linear_vel_z_read = QtGui.QLineEdit(self)
        self.navdata_linear_vel_z_read.setGeometry(self.navdata_linear_vel_base_x_cord + self.box_width,
                                               self.navdata_linear_vel_base_y_cord + 2*self.box_height,
                                               self.box_width, self.box_height)
        self.navdata_linear_vel_z_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_linear_vel_z_read.setText("yup")

        ################################################################################################################
        """ Defining the messages for ardorne/navadta motor rpm"""
        self.navdata_motor_base_x_cord = self.imu_ang_vel_base_x_cord
        self.navdata_motor_base_y_cord = self.imu_ang_vel_base_y_cord + 3*self.box_height +20

        self.navdata_motor1_rpm = QtGui.QLabel('Motor 1', self)
        self.navdata_motor1_rpm.setGeometry(self.navdata_motor_base_x_cord, self.navdata_motor_base_y_cord,
                                            self.box_width, self.box_height)
        self.navdata_motor1_rpm.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_motor2_rpm = QtGui.QLabel('Motor 2', self)
        self.navdata_motor2_rpm.setGeometry(self.navdata_motor_base_x_cord,
                                            self.navdata_motor_base_y_cord + self.box_height, self.box_width,
                                            self.box_height)
        self.navdata_motor2_rpm.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_motor3_rpm = QtGui.QLabel('Motor 3', self)
        self.navdata_motor3_rpm.setGeometry(self.navdata_motor_base_x_cord,
                                            self.navdata_motor_base_y_cord + 2*self.box_height, self.box_width,
                                            self.box_height)
        self.navdata_motor3_rpm.setFont(QtGui.QFont(self.font_style, self.font_weight))

        self.navdata_motor4_rpm = QtGui.QLabel('Motor 4', self)
        self.navdata_motor4_rpm.setGeometry(self.navdata_motor_base_x_cord,
                                            self.navdata_motor_base_y_cord + 3*self.box_height, self.box_width,
                                            self.box_height)
        self.navdata_motor4_rpm.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the readings for /ardrone/navdata motor rpm"""
        self.navdata_motor1_rpm_read = QtGui.QLineEdit(self)
        self.navdata_motor1_rpm_read.setGeometry(self.navdata_motor_base_x_cord + self.box_width,
                                                 self.navdata_motor_base_y_cord, self.box_width, self.box_height)
        self.navdata_motor1_rpm_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_motor1_rpm_read.setText("yup")

        self.navdata_motor2_rpm_read = QtGui.QLineEdit(self)
        self.navdata_motor2_rpm_read.setGeometry(self.navdata_motor_base_x_cord + self.box_width,
                                                 self.navdata_motor_base_y_cord + self.box_height,
                                                 self.box_width, self.box_height)
        self.navdata_motor2_rpm_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_motor2_rpm_read.setText("yup")

        self.navdata_motor3_rpm_read = QtGui.QLineEdit(self)
        self.navdata_motor3_rpm_read.setGeometry(self.navdata_motor_base_x_cord + self.box_width,
                                                 self.navdata_motor_base_y_cord + 2*self.box_height,
                                                 self.box_width, self.box_height)
        self.navdata_motor3_rpm_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_motor3_rpm_read.setText("yup")

        self.navdata_motor4_rpm_read = QtGui.QLineEdit(self)
        self.navdata_motor4_rpm_read.setGeometry(self.navdata_motor_base_x_cord + self.box_width,
                                                 self.navdata_motor_base_y_cord + 3*self.box_height,
                                                 self.box_width, self.box_height)
        self.navdata_motor4_rpm_read.setFont(QtGui.QFont(self.font_style, self.font_weight))
        # self.navdata_motor4_rpm_read.setText("yup")

        ################################################################################################################
        self.navdata_alt_base_x_cord = self.navdata_lin_accn_base_x_cord
        self.navdata_alt_base_y_cord = self.navdata_lin_accn_base_y_cord + 3*self.box_height + 20
        """ Defining the ardrone/navdata altitude message"""
        self.navdata_alt = QtGui.QLabel('Altitude', self)
        self.navdata_alt.setGeometry(self.navdata_alt_base_x_cord, self.navdata_alt_base_y_cord, self.box_width,
                                     self.box_height)
        self.navdata_alt.setFont(QtGui.QFont(self.font_style, self.font_weight))

        """ Defining the ardrone/navdata altitude reading """
        self.navdata_alt_reading = QtGui.QLineEdit(self)
        self.navdata_alt_reading.setGeometry(self.navdata_alt_base_x_cord + self.box_width, self.navdata_alt_base_y_cord, self.box_width, self.box_height)
        self.navdata_alt_reading.setFont(QtGui.QFont(self.font_style, self.font_weight))
        self.navdata_alt_reading.setText("yup")


    def ReceiveNavdata(self, navdata):
        self.navdata = navdata

    def ReceiveImudata(self, imu):
        self.imu = imu

    def ConnectionCallback(self):
        self.connected = self.CommunicationSinceTimer
        self.CommunicationSinceTimer = False

    def RedrawCallback(self):
        # self.data_lock.acquire()
        # try:
        self.navdata_mag_box_x_read.setText("%.3f"%(self.navdata.magX))
        self.navdata_mag_box_y_read.setText("%.3f"%(self.navdata.magY))
        self.navdata_mag_box_z_read.setText("%.3f"%(self.navdata.magZ))
        self.imu_box_x_read.setText("%.3f"%(10.0003))
        self.imu_box_y_read.setText("%.3f"%(10.0003))
        self.imu_box_z_read.setText("%.3f"%(10.0003))
        self.imu_ang_vel_x_read.setText("%.3f"%(self.imu.angular_velocity.x))
        self.imu_ang_vel_y_read.setText("%.3f"%(self.imu.angular_velocity.y))
        self.imu_ang_vel_z_read.setText("%.3f"%(self.imu.angular_velocity.x))
        self.navdata_lin_accn_x_read.setText("%.3f"%(self.navdata.ax))
        self.navdata_lin_accn_y_read.setText("%.3f"%(self.navdata.ay))
        self.navdata_lin_accn_z_read.setText("%.3f"%(self.navdata.az))
        self.navdata_rot_x_read.setText("%.3f"%(self.navdata.rotX))
        self.navdata_rot_y_read.setText("%.3f"%(self.navdata.rotY))
        self.navdata_rot_z_read.setText("%.3f"%(self.navdata.rotZ))
        self.imu_orientation_x_read.setText("%.3f"%(self.imu.orientation.x))
        self.imu_orientation_y_read.setText("%.3f"%(self.imu.orientation.y))
        self.imu_orientation_z_read.setText("%.3f"%(self.imu.orientation.z))
        self.imu_orientation_w_read.setText("%.3f"%(self.imu.orientation.w))
        self.navdata_linear_vel_x_read.setText("%.3f"%(self.navdata.vx))
        self.navdata_linear_vel_y_read.setText("%.3f"%(self.navdata.vy))
        self.navdata_linear_vel_z_read.setText("%.3f"%(self.navdata.vz))
        self.navdata_motor1_rpm_read.setText("%.3f"%(self.navdata.motor1))
        self.navdata_motor2_rpm_read.setText("%.3f"%(self.navdata.motor2))
        self.navdata_motor3_rpm_read.setText("%.3f"%(self.navdata.motor3))
        self.navdata_motor4_rpm_read.setText("%.3f"%(self.navdata.motor4))
        self.navdata_alt_reading.setText("%.3f"%(self.navdata.altd))
        ################################################################################################################
        """ Defining the status bar for getting the battery percent and the drone state """
        self.statusMessage = '{} (Battery: {}%)'.format(self.navdata.state, int(self.navdata.batteryPercent))
        self.statusBar().showMessage(self.statusMessage)
        self.statusBar().setFont(QtGui.QFont(self.font_style, 12))
        # finally:
        #     self.data_lock.release()


        # self.imu_orientation_x = QtGui.QLabel('X')
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    rospy.init_node('ARDrone_Navdata')
    disp = Navdata_GUI()
    """ Subscribers"""
    # disp.ReceiveNavdata(disp.navdata)
    # disp.ReceiveImudata(disp.imu)
    disp.show()
    # rospy.spin()
    sys.exit(app.exec_())