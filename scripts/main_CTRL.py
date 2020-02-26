#!/usr/bin/env python
import getpass

import numpy as np
import math
import rospy

from std_msgs.msg import Header

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from threading import Thread, Event

# msgs
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn, State, Thrust, AttitudeTarget, ActuatorControl

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# about QTGUI
import sys
from PyQt5 import QtWidgets
from PyQt5 import uic
from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsPixmapItem

# import my classes
from class_callback_func import *

# data
class Data_storage(object):
    def __init__(self, idx_uav):
        # if idx_uav == 1:
        self.arm  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.pub_rawtargetatt = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)

        self.des_x = 0
        self.des_y = 0
        self.des_z = 0

        self.roll_cmd     = 0
        self.pitch_cmd    = 0
        self.yaw_cmd      = 0
        self.throttle_cmd = 0

        self.mavmsg = cb()

# define agent (global variable)
agent1 = Data_storage(idx_uav=1)

class PX4_GUI(QtWidgets.QDialog):
    def __init__(self, parent=None):
        QtWidgets.QDialog.__init__(self, parent)
        user = getpass.getuser()
        self.ui = uic.loadUi("/home/" + user + "/workspaces/hgt_est_ws/src/px4_controller/scripts/gui_TK1.ui", self)
        self.ui.show()

        self.srv_reset = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, agent1.mavmsg.local_position_callback)
        rospy.Subscriber("/mavros/state", State, agent1.mavmsg.uav_state_callback)
        rospy.Subscriber("/mavros/imu/data_raw", Imu, agent1.mavmsg.imu_data_raw_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, agent1.mavmsg.gps_local_callback)
        rospy.Subscriber("/mavros/target_actuator_control", ActuatorControl, agent1.mavmsg.ctrl_callback)

        agent1.OT = Offboard_thread(idx_uav=1)

        self.offboard_thread_chk = 2

        self.slider_roll_1 = self.horizontalSlider_roll_1
        self.slider_pitch_1 = self.verticalSlider_pitch_1
        self.slider_yaw_1 = self.horizontalSlider_yaw_1
        self.slider_throttle_1 = self.verticalSlider_throttle_1

        self.slider_des_x_1 = self.horizontalSlider_des_x_1
        self.slider_des_y_1 = self.horizontalSlider_des_y_1
        self.slider_des_z_1 = self.horizontalSlider_des_z_1

        self.text_des_x_1 = self.plainTextEdit_des_x_1
        self.text_des_y_1 = self.plainTextEdit_des_y_1
        self.text_des_z_1 = self.plainTextEdit_des_z_1

        self.text_state_x_1 = self.plainTextEdit_state_x_1
        self.text_state_y_1 = self.plainTextEdit_state_y_1
        self.text_state_z_1 = self.plainTextEdit_state_z_1

        self.chkbox_FCU_CC = self.checkBox_Conn_FCU_CC
        self.chkbox_CC_GCS = self.checkBox_Conn_CC_GCS

        self.scene = QGraphicsScene()

        self.waypoint_run = 0

        # timer for periodic update of GUI
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)  # Throw event timeout with an interval of 100 milliseconds
        self.timer.timeout.connect(self.update_gui)  # each time timer counts a second, call self.blink
        self.color_flag = True

        self.timer_start()

    def timer_start(self):
        self.timer.start()

    def timer_stop(self):
        self.timer.stop()

    def update_gui(self):
        t = rospy.get_time()

        self.slider_roll_1.setValue(agent1.roll_cmd*100+50)
        self.slider_pitch_1.setValue(agent1.pitch_cmd*100+50)
        self.slider_yaw_1.setValue(agent1.yaw_cmd*100+50)
        self.slider_throttle_1.setValue(agent1.throttle_cmd*100+50)

        self.text_des_x_1.setPlainText(str("{0:.2f}".format(agent1.des_x)))
        self.text_des_y_1.setPlainText(str("{0:.2f}".format(agent1.des_y)))
        self.text_des_z_1.setPlainText(str("{0:.2f}".format(agent1.des_z)))

        self.text_state_x_1.setPlainText(str("{0:.2f}".format(agent1.mavmsg.state.pose.position.x)))
        self.text_state_y_1.setPlainText(str("{0:.2f}".format(agent1.mavmsg.state.pose.position.y)))
        self.text_state_z_1.setPlainText(str("{0:.2f}".format(agent1.mavmsg.state.pose.position.z)))

        if agent1.mavmsg.status.connected == True:
            self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("connected"))
            self.chkbox_FCU_CC.setCheckState(True)
            self.chkbox_FCU_CC.setText("connected")
        else:
            self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("disconnected"))
            self.chkbox_FCU_CC.setCheckState(False)
            self.chkbox_FCU_CC.setText("disconnected")

        if agent1.mavmsg.status.armed == True:
            self.tableWidget.setItem(1, 0, QtWidgets.QTableWidgetItem("armed"))
        else:
            self.tableWidget.setItem(1, 0, QtWidgets.QTableWidgetItem("disarmed"))
        self.tableWidget.setItem(2, 0, QtWidgets.QTableWidgetItem(agent1.mavmsg.status.mode))

        if self.offboard_thread_chk == 1:
            self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        elif self.offboard_thread_chk == 2:
            self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("off"))
        elif self.offboard_thread_chk == 3:
            self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("suspend"))
        elif self.offboard_thread_chk == 4:
            self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))

        pixmap = QtGui.QPixmap()
        pixmap.load('catkin_ws/src/wc_gazebo/scripts/camera_image.jpeg')
        item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(item)

    # UAV_1
    @pyqtSlot()
    def slot1(self):  # pushButton_arm
        # self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("armed"))
        agent1.arm(True)

    @pyqtSlot()
    def slot2(self): # pushButton_disarm
        # self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("disarmed"))
        agent1.arm(False)

    @pyqtSlot()
    def slot3(self): # click offboard radio button
        # !! should check there is periodic ctrl command
        check = agent1.mode(custom_mode = "OFFBOARD")

    @pyqtSlot()
    def slot4(self): # click stabilize radio button
        # check = self.mode(custom_mode="STABILIZED")
        agent1.mode(custom_mode='MANUAL')

    @pyqtSlot() ##
    def slot5(self): # click offboard thread on
        if self.offboard_thread_chk == 3:
            agent1.OT.myResume()
            self.offboard_thread_chk = 1
        elif self.offboard_thread_chk == 2:
            agent1.OT.start()
            self.offboard_thread_chk = 1

    @pyqtSlot() ##
    def slot6(self):  # click offboard thread off
        if self.offboard_thread_chk == 1:
            agent1.OT.mySuspend()
            self.offboard_thread_chk = 3
        else:
            agent1.OT.myExit()
            agent1.OT = Offboard_thread(idx_uav=1)
            self.offboard_thread_chk = 2

    @pyqtSlot() ##
    def slot7(self):  # set reference GPS
        agent1.mavmsg.lat_ref = agent1.mavmsg.gps_raw.latitude
        agent1.mavmsg.lon_ref = agent1.mavmsg.gps_raw.longitude
        agent1.mavmsg.alt_ref = agent1.mavmsg.gps_raw.altitude

    @pyqtSlot()
    def slot15(self): # horizontal slider(desired pos (x))
        agent1.des_x = self.slider_des_x_1.value()

    @pyqtSlot()
    def slot16(self): # horizontal slider(desired pos (y))
        agent1.des_y = self.slider_des_y_1.value()

    @pyqtSlot()
    def slot17(self): # horizontal slider(desired pos (z))
        agent1.des_z = self.slider_des_z_1.value()/10.0

    @pyqtSlot()
    def slot18(self): # run waypoint flight (set waypoint trajectory)
        self.traj = traj_gen()
        self.traj.coeff_x = self.traj.calc_coeff(self.traj.wp[0, :], self.traj.T, self.traj.S)
        self.traj.coeff_y = self.traj.calc_coeff(self.traj.wp[1, :], self.traj.T, self.traj.S)
        self.traj.coeff_z = self.traj.calc_coeff(self.traj.wp[2, :], self.traj.T, self.traj.S)
        self.waypoint_run=1

    # reset all
    @pyqtSlot()
    def slot19(self): # reset simulation
        # before reset, set all the desired pos/command to 0
        agent1.des_x = 0
        agent1.des_y = 0
        agent1.des_z = 0
        agent1.roll_cmd = 0
        agent1.pitch_cmd = 0
        agent1.yaw_cmd = 0
        agent1.throttle_cmd = 0.066

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 2
        q = quaternion_from_euler(0,0,0)
        pose.orientation = Quaternion(*q)
        state = ModelState()
        state.model_name = "iris_opt_flow"
        state.pose = pose
        self.srv_reset(state)

# minimum snap trajectory
class traj_gen():
    def __init__(self):
        self.tn = rospy.get_time()         # time now
        self.wp = np.matrix('0 1 2 3 2 0;\
                           0 2 4 5 2 0;\
                           0 4 5 4 2 1')   # wp_z
        self.T = np.matrix('3 3 3 3 3')    # Time to go for each path
        self.S = np.matrix('0 3 6 9 12')
        self.S = self.S + self.tn
        self.ts = self.S[0,0]
        self.dt = 0.01
        self.tf = self.ts + np.sum(self.T)

        self.n_wp = np.size(self.wp, 1)  # col size
        self.n_p = self.n_wp - 1

        self.coeff_x = np.zeros((self.n_p, 8))
        self.coeff_y = np.zeros((self.n_p, 8))
        self.coeff_z = np.zeros((self.n_p, 8))

    def calc_coeff(self, wp, T, S):
        n_wp = np.size(wp, 1)  # col size
        n_p = n_wp - 1

        wp = wp.astype(float)
        T = T.astype(float)
        S = S.astype(float)

        a = np.zeros((8 * n_p, 1))
        a = a.astype(float)
        A = np.zeros((8 * n_p, 8 * n_p))
        A = A.astype(float)
        b = np.zeros((8 * n_p, 1))
        b = b.astype(float)

        idx_row = 0

        # waypoint constraint (2n constraint) => n polynomial & 2 constraints (0,S)
        for i in range(n_p):
            A[idx_row, 8 * i:8 * (i + 1)] = [1, 0, 0, 0, 0, 0, 0, 0]
            A[idx_row + 1, 8 * i:8 * (i + 1)] = [1, 1, 1, 1, 1, 1, 1, 1]

            b[idx_row, 0] = wp[0, i]
            b[idx_row + 1, 0] = wp[0, i + 1]
            idx_row = idx_row + 2

        # derivative 1-3 are aero at endpoint(6 constraint)
        # derivative 1
        A[idx_row, 0:8] = [0, 1, 0, 0, 0, 0, 0, 0] / T[0, 0]
        A[idx_row + 1, 8 * (n_p - 1):8 * n_p] = [0, 1, 2, 3, 4, 5, 6, 7] / T[0, n_p - 1]
        idx_row = idx_row + 2;
        # derivative 2
        A[idx_row, 0:8] = [0, 0, 2, 0, 0, 0, 0, 0] / T[0, 0] ** 2
        A[idx_row + 1, 8 * (n_p - 1):8 * n_p] = [0, 0, 2, 6, 12, 20, 30, 42] / T[0, n_p - 1] ** 2
        idx_row = idx_row + 2;
        # derivative 3
        A[idx_row, 0:8] = [0, 0, 0, 6, 0, 0, 0, 0] / T[0, 0] ** 3
        A[idx_row + 1, 8 * (n_p - 1):8 * n_p] = [0, 0, 0, 6, 24, 60, 120, 210] / T[0, n_p - 1] ** 3
        idx_row = idx_row + 2;

        # derivative 1-6(6n-6 constraints)
        for i in range(n_p - 1):
            A[idx_row + 6 * i + 0, 8 * i:8 * (i + 1)] = [0, 1, 2, 3, 4, 5, 6, 7] / T[0, i]
            A[idx_row + 6 * i + 0, 8 * (i + 1):8 * (i + 2)] = [0, -1, 0, 0, 0, 0, 0, 0] / T[0, i + 1]  # dev-1
            A[idx_row + 6 * i + 1, 8 * i:8 * (i + 1)] = [0, 0, 2, 6, 12, 20, 30, 42] / T[0, i] ** 2
            A[idx_row + 6 * i + 1, 8 * (i + 1):8 * (i + 2)] = [0, 0, -2, 0, 0, 0, 0, 0] / T[0, i + 1] ** 2  # dev-2
            A[idx_row + 6 * i + 2, 8 * i:8 * (i + 1)] = [0, 0, 0, 6, 24, 60, 120, 210] / T[0, i] ** 3
            A[idx_row + 6 * i + 2, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, -6, 0, 0, 0, 0] / T[0, i + 1] ** 3  # dev-3
            A[idx_row + 6 * i + 3, 8 * i:8 * (i + 1)] = [0, 0, 0, 0, 24, 120, 360, 840] / T[0, i] ** 4
            A[idx_row + 6 * i + 3, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, 0, -24, 0, 0, 0] / T[0, i + 1] ** 4  # dev-4
            A[idx_row + 6 * i + 4, 8 * i:8 * (i + 1)] = [0, 0, 0, 0, 0, 120, 720, 2520] / T[0, i] ** 5
            A[idx_row + 6 * i + 4, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, 0, 0, -120, 0, 0] / T[0, i + 1] ** 5  # dev-5
            A[idx_row + 6 * i + 5, 8 * i:8 * (i + 1)] = [0, 0, 0, 0, 0, 0, 720, 5040] / T[0, i] ** 6
            A[idx_row + 6 * i + 5, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, 0, 0, 0, -720, 0] / T[0, i + 1] ** 6  # dev-6

        a = np.dot(np.linalg.inv(A), b)
        a_reshape = np.transpose(a.reshape(n_p, 8))
        return a_reshape

    def des_t(self, t, coeff, S, T):
        bound_low = (S <= t)
        bound_high = (S + T >= t)
        p_i = np.where(bound_low == bound_high)  # which polynomial to use
        a = coeff[:, p_i[1][0]]
        Tgo = T[0, p_i[1][0]]
        Si = S[0, p_i[1][0]]

        x = (t - Si) / Tgo
        pos = a[0] + a[1]*x + a[2]*x**2 + a[3]*x**3 + a[4]*x**4 + a[5]*x**5 + a[6]*x**6 + a[7]*x**7

        return pos

class Offboard_thread(Thread):
    def __init__(self, idx_uav):
        Thread.__init__(self)
        self.idx_uav = idx_uav
        self.rate = rospy.Rate(20) # 5Hz

        self.ctrl = setpoint_att()

        self.pub_des_pose = rospy.Publisher('/datalog/des_pose', PoseStamped, queue_size=10)
        self.des_pose_msg = PoseStamped()

        self.__suspend = False
        self.__exit = False
        self.daemon = True

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.__suspend == True:
                    continue

                self.ctrl.calc_cmd_att_thr_auto(self.idx_uav)
                self.ctrl.talk(self.idx_uav)

                self.des_pose_msg.header.stamp = rospy.Time.now()
                self.des_pose_msg.pose.position.x = agent1.des_x
                self.des_pose_msg.pose.position.y = agent1.des_y
                self.des_pose_msg.pose.position.z = agent1.des_z
                self.pub_des_pose.publish(self.des_pose_msg)

                self.rate.sleep()

                ### Exit ###
                if self.__exit:
                    break

            except rospy.ROSInterruptException:
                pass

    def mySuspend(self):
        self.__suspend = True

    def myResume(self):
        self.__suspend = False

    def myExit(self):
        self.__exit = True

class setpoint_att(object):
    def __init__(self):
        self.cmd_att = PoseStamped()
        self.cmd_thr = Thrust()

        self.cmd_rawatt = AttitudeTarget()

        self.count = 1

        self.roll = 0 # [-1, 1]
        self.pitch = 0 # [-1, 1]
        self.yaw = 0 # [-1, 1]
        self.throttle = 0 # [0, 1]

        self.state_x = 0
        self.state_y = 0
        self.state_z = 0

        self.state_phi = 0
        self.state_theta = 0
        self.state_psi = 0

        self.des_x = 0 #
        self.des_y = 0 #
        self.des_z = 0 #
        self.des_vx = 0
        self.des_vy = 0
        self.des_vz = 0
        self.des_ax = 0
        self.des_ay = 0
        self.des_az = 0

        self.des_yaw = 0 #
        self.des_yawdot = 0 #

        self.Kp_x = 20
        self.Kv_x = 5
        self.Kp_y = 20
        self.Kv_y = 5
        self.Kp_z = 5
        self.Kv_z = 2.5

        self.Kp_phi = 0.1
        self.Kv_phi = 0.0001
        self.Kp_theta = 0.1
        self.Kv_theta = 0.0001
        self.Kp_psi = 0.15
        self.Kv_psi = 0.0001

        self.m = 0.065
        self.g = 9.81

    def calc_cmd_att_thr_joy(self): # offboard control using joystic
        self.roll = agent1.roll_cmd # [-1, 1]
        self.pitch = agent1.pitch_cmd # [-1, 1]
        self.yaw = agent1.yaw_cmd # [-1, 1]
        self.throttle = agent1.throttle_cmd + 0.5 # [0, 1]

        self.cmd_att.header.stamp = rospy.Time.now()

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.cmd_att.header.seq = self.count
        self.cmd_att.pose.orientation = Quaternion(*q)
        self.cmd_thr.thrust = self.throttle

        self.count += 1

        return (self.cmd_att, self.cmd_thr)

    def calc_cmd_att_thr_auto(self, idx_uav): # offboard control autonomously (roll/pitch/yaw/throttle)
        t_prev = self.cmd_att.header.stamp.secs
        self.cmd_att.header.stamp.secs = rospy.get_time()
        t_now = self.cmd_att.header.stamp.secs

        dt = t_now - t_prev
        if dt == 0 :
            dt =0.05

        if idx_uav == 1:
            agent = agent1

        self.des_x = agent.des_x  #
        self.des_y = agent.des_y  #
        self.des_z = agent.des_z  #

        des_x_prev = self.des_x
        des_y_prev = self.des_y
        des_z_prev = self.des_z
        des_vx_prev = self.des_vx
        des_vy_prev = self.des_vy
        des_vz_prev = self.des_vz

        self.des_vx = (self.des_x - des_x_prev) / dt
        self.des_vy = (self.des_y - des_y_prev) / dt
        self.des_vz = (self.des_z - des_z_prev) / dt
        self.des_ax = (self.des_vx - des_vx_prev) / dt
        self.des_ay = (self.des_vy - des_vy_prev) / dt
        self.des_az = (self.des_vz - des_vz_prev) / dt

        state_phi_prev = self.state_phi
        state_theta_prev = self.state_theta
        state_psi_prev = self.state_psi
        (self.state_phi, self.state_theta, self.state_psi) = euler_from_quaternion([agent.mavmsg.state.pose.orientation.x, agent.mavmsg.state.pose.orientation.y, agent.mavmsg.state.pose.orientation.z, agent.mavmsg.state.pose.orientation.w])
        state_vphi = (self.state_phi - state_phi_prev) / dt
        state_vtheta = (self.state_theta - state_theta_prev) / dt
        state_vpsi = (self.state_psi - state_psi_prev) / dt

        state_x_prev = self.state_x
        state_y_prev = self.state_y
        state_z_prev = self.state_z

        self.state_x = agent.mavmsg.state.pose.position.x
        self.state_y = agent.mavmsg.state.pose.position.y
        self.state_z = agent.mavmsg.state.pose.position.z

        state_vx = (self.state_x - state_x_prev) / dt
        state_vy = (self.state_y - state_y_prev) / dt
        state_vz = (self.state_z - state_z_prev) / dt

        err_x = self.des_x - self.state_x
        err_y = self.des_y - self.state_y
        err_z = self.des_z - self.state_z

        err_vx = self.des_vx - state_vx
        err_vy = self.des_vy - state_vy
        err_vz = self.des_vz - state_vz

        acc_comm_x = self.des_ax + self.Kv_x * err_vx + self.Kp_x * err_x
        acc_comm_y = self.des_ay + self.Kv_y * err_vy + self.Kp_y * err_y
        acc_comm_z = self.des_az + self.Kv_z * err_vz + self.Kp_z * err_z

        u1 = self.m * (self.g + acc_comm_z)

        des_phi = 1/self.g * (acc_comm_x*math.sin(self.des_yaw) - acc_comm_y*math.cos(self.des_yaw))
        des_theta = 1/self.g * (acc_comm_x*math.cos(self.des_yaw) + acc_comm_y*math.sin(self.des_yaw))

        u_phi = self.Kp_phi*(des_phi-self.state_phi) + self.Kv_phi*(0-state_vphi)
        u_theta = self.Kp_theta*(des_theta-self.state_theta) + self.Kv_theta*(0-state_vtheta)
        u_psi = self.Kp_psi*(self.des_yaw-self.state_psi) + self.Kv_psi*(self.des_yawdot - state_vpsi)

        u1 = np.maximum(u1, -2.0)
        u1 = np.minimum(u1, 2.0)
        u_phi = np.maximum(u_phi, -0.6)
        u_phi = np.minimum(u_phi, 0.6)
        u_theta = np.maximum(u_theta, -0.6)
        u_theta = np.minimum(u_theta, 0.6)
        u_psi = np.maximum(u_psi, -0.6)
        u_psi = np.minimum(u_psi, 0.6)

        q = quaternion_from_euler(u_phi,u_theta,u_psi)

        self.cmd_att.pose.orientation = Quaternion(*q)
        self.cmd_thr.thrust = u1

        self.cmd_rawatt.thrust = u1
        self.cmd_rawatt.orientation = Quaternion(*q)

        # ctrl_info_talker(u_phi, u_theta, u_psi, u1)

        # print("%f %f %f %f", u_phi, u_theta, u_psi, u1)
        if idx_uav == 1:
            agent1.roll_cmd = u_phi
            agent1.pitch_cmd = u_theta
            agent1.yaw_cmd = u_psi
            agent1.throttle_cmd = u1 - 0.5

        return (self.cmd_att, self.cmd_thr)

    def talk(self, idx_uav):
        self.cmd_att.pose.position.z = 10.0
        if idx_uav == 1:
            agent1.pub_rawtargetatt.publish(self.cmd_rawatt)

if __name__ == '__main__':
    rospy.init_node('controller')

    app = QtWidgets.QApplication(sys.argv)
    W = PX4_GUI()
    app.exec_()
