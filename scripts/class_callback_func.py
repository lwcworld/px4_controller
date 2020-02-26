#!/usr/bin/env python
import numpy as np
import math
import rospy
from std_msgs.msg import Header

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from threading import Thread, Event

# msgs
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn, State, Thrust, AttitudeTarget
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from coordinate_transform import *
from own_publisher import *

class cb():
    def __init__(self):
        self.state = PoseStamped() # state of UAV : position related info
        self.status = State() # status of UAV : flight status info (arm/disarm, mode, etc..)
        self.imu_data = Imu
        self.imu_data_raw = Imu

        self.x_m = 0.
        self.y_m = 0.
        self.z_m = 0.

        self.ax_m = 0.
        self.ay_m = 0.
        self.az_m = 0.

        self.p_m = 0. # angular x velocity
        self.q_m = 0. # angular y velocity
        self.r_m = 0. # angular z velocity

        self.lat_ref = 0.
        self.lon_ref = 0.
        self.alt_ref = 0.

        self.qw_m = 0.
        self.qx_m = 0.
        self.qy_m = 0.
        self.qz_m = 0.

        self.vx_m = 0.
        self.vy_m = 0.
        self.vz_m = 0.

        self.u1 = 0.
        self.u2 = 0.
        self.u3 = 0.
        self.u4 = 0.

        self.isfault_true = 0.
        self.isfault_est = 0.

        self.time_last_msg_update = np.zeros((3,1)) # imu, gps, ctrl

        self.issend_Socket = 0

    def local_position_callback(self, msg):
        self.state = msg

    def uav_state_callback(self, msg):
        self.status = msg

    def imu_data_callback(self, msg): # fused data (raw data + computed from FCU)
        # print(msg)
        self.imu_data = msg

        self.qw_m = self.imu_data.orientation.w
        self.qx_m = self.imu_data.orientation.x
        self.qy_m = self.imu_data.orientation.y
        self.qz_m = self.imu_data.orientation.z

        self.p_m = self.imu_data.angular_velocity.x
        self.q_m = self.imu_data.angular_velocity.y
        self.r_m = self.imu_data.angular_velocity.z

        self.time_last_msg_update[0] = self.imu_data.header.stamp.secs + self.imu_data.header.stamp.nsecs / 1.0e+9


    def imu_data_raw_callback(self, msg): # raw data
        self.imu_data_raw = msg

        self.ax_m = self.imu_data_raw.linear_acceleration.x
        self.ay_m = self.imu_data_raw.linear_acceleration.y
        self.az_m = self.imu_data_raw.linear_acceleration.z

        self.p_m = self.imu_data_raw.angular_velocity.x
        self.q_m = self.imu_data_raw.angular_velocity.y
        self.r_m = self.imu_data_raw.angular_velocity.z

    def gps_local_callback(self, msg):
        self.gps_local = msg

        self.x_m = self.gps_local.pose.position.x
        self.y_m = self.gps_local.pose.position.y
        self.z_m = self.gps_local.pose.position.z

        # self.vx_m = self.gps_local.twist.twist.linear.x
        # self.vy_m = self.gps_local.twist.twist.linear.y
        # self.vz_m = self.gps_local.twist.twist.linear.z

        # print(self.time_last_msg_update)

        self.time_last_msg_update[1] = self.gps_local.header.stamp.secs + self.gps_local.header.stamp.nsecs / 1.0e+9

    def vel_local_callback(self, msg):
        self.vx_m = msg.twist.linear.x
        self.vy_m = msg.twist.linear.y
        self.vz_m = msg.twist.linear.z

    def gps_rawnav_callback(self, msg):
        self.gps_raw = msg

        self.x_m, self.y_m, self.z_m = transform.geodetic_to_enu(self.gps_raw.latitude, self.gps_raw.longitude, self.gps_raw.altitude, self.lat_ref, self.lon_ref, self.alt_ref)

    def ctrl_callback(self, msg):
        # print(msg.controls)
        ctrl_info_talker(msg.controls[0],msg.controls[1],msg.controls[2],msg.controls[3])

        # print(msg.controls[0])
        self.u1 = msg.controls[0]
        self.u2 = msg.controls[1]
        self.u3 = msg.controls[2]
        self.u4 = msg.controls[3]

        self.time_last_msg_update[
            2] = msg.header.stamp.secs + msg.header.stamp.nsecs / 1.0e+9

    def rc_in(self, msg):
        if msg.channels[7] > 1200.0:
            self.isfault_true = 0
        else:
            self.isfault_true = 1

    def provide_isfault_true(self, req):
        return self.isfault_true

    def provide_isfault_est(self, req):
        return self.isfault_est
