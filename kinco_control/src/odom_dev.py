#!/usr/bin/env python3

import math
from math import sin, cos, pi
import numpy as np

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import *

from struct import *
import can
from can import *
import time
import sys
import os

# Kalman Filter class
class KalmanFilter:
    def __init__(self, process_noise, measurement_noise):
        self.Q = 2  # Process noise covariance
        self.R = 2  # Measurement noise covariance
        self.x = 0.0  # Initial state
        self.P = 1.0  # Initial estimation error covariance

    def filter(self, measurement):
        # Prediction step
        self.P += self.Q

        # Update step
        K = self.P / (self.P + self.R)  # Kalman gain
        self.x += K * (measurement - self.x)  # Update the state estimate
        self.P *= (1 - K)  # Update the error covariance

        return self.x
# Main
class UpdateOdom:
    def __init__(self):

        self.node_name = "kinco"
        rospy.init_node(self.node_name)

        # canbus read param
        if not rospy.has_param("~can_device"):
            rospy.set_param("~can_device", "can0")
        if not rospy.has_param("~canbus_bitrate"):
            rospy.set_param("~canbus_bitrate", 500000)
        if not rospy.has_param("~motor_1_id"):
            rospy.set_param("~motor_1_id", 1)
        if not rospy.has_param("~motor_2_id"):
            rospy.set_param("~motor_2_id", 2)
        if not rospy.has_param("~velocity_unit_rpm"):
            rospy.set_param("~velocity_unit_rpm", 1)
        self.can_device = rospy.get_param("~can_device")
        self.can_bitrate = rospy.get_param("~canbus_bitrate")
        self.motor_1_id = rospy.get_param("~motor_1_id")
        self.motor_2_id = rospy.get_param("~motor_2_id")
        self.velocity_unit_rpm = rospy.get_param("~velocity_unit_rpm")


        # odometry calculation param
        if not rospy.has_param("~wheels_x_distance"):
            rospy.set_param("~wheels_x_distance", 0.0)
        if not rospy.has_param("~wheels_y_distance"):
            rospy.set_param("~wheels_y_distance", 0.49)
        if not rospy.has_param("~wheel_diameter"):
            rospy.set_param("~wheel_diameter", 0.2)
        if not rospy.has_param("~max_motor_rpm"):
            rospy.set_param("~max_motor_rpm", 3000.0)
        if not rospy.has_param("~gear_ratio"):
            rospy.set_param("~gear_ratio", 20.00)
        if not rospy.has_param("~total_wheels"):
            rospy.set_param("~total_wheels", 2.0)
        self.wheels_x_distance_ = rospy.get_param("~wheels_x_distance")
        self.wheels_y_distance_ = rospy.get_param("~wheels_y_distance")
        self.wheel_diameter = rospy.get_param("~wheel_diameter")
        self.max_motor_rpm = rospy.get_param("~max_motor_rpm")
        self.gear_ratio = rospy.get_param("~gear_ratio")
        self.total_wheels_ = rospy.get_param("~total_wheels")
        self.wheel_circumference_ = math.pi * self.wheel_diameter # 0.62831853071 #math.pi * self.wheel_diameter  # 0.62831853071

        # canopen
        filters = [
            {"can_id": 0x181, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x182, "can_mask": 0x7FF, "extended": False},
        ]

        self.bus = can.interface.Bus(interface='socketcan', channel='can0', can_filters=filters)

        self.reset_service = rospy.Service("/reset_odom", SetBool, self.callback_reset_odom)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        # self.last_object_time = None
        self.transforms = []

        # odometry publisher param
        if not rospy.has_param("~publish_tf"):
            rospy.set_param("~publish_tf", True)
        if not rospy.has_param("~base_frame"):
            rospy.set_param("~base_frame", "base_link")
        if not rospy.has_param("~odom_frame"):
            rospy.set_param("~odom_frame", "odom")
        self.odom_frame = rospy.get_param("~odom_frame")
        self.base_frame = rospy.get_param("~base_frame")
        self.publish_tf = rospy.get_param("~publish_tf")
        # self.wheel_circumference_ = math.pi * self.wheel_diameter

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.00
        self.vy = 0.00
        self.vth = 0.00

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()


        self.m1read = False
        self.m2read = False
        self.m1 = 0.00
        self.m2 = 0.00
        self.m1_actual_speed = 0.0
        self.m2_actual_speed = 0.0

        self.current_theta = 0.0
        self.last_theta = 0.0


        # can setup parameter
        # time.sleep(5)
        self.calculate_odom()


        # self.kalman_rpm1 = KalmanFilter(1, 1)  # Process noise Q, Measurement noise R
        # self.kalman_rpm2 = KalmanFilter(1, 1)
        # rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.calculate_odom()
            # self.filtered_rpm1, self.filtered_rpm2 = self.get_filtered_rpm()
            # rospy.loginfo(f"Filtered RPM1: {self.filtered_rpm1}, Filtered RPM2: {self.filtered_rpm2}")
            # rate.sleep()



    # def get_filtered_rpm(self):
    #     # Get raw RPM readings
    #     rpm1, rpm2 = self.get_rpm()

    #     # Apply Kalman filter

    #     filtered_rpm1 = self.kalman_rpm1.filter(rpm1)
    #     filtered_rpm2 = self.kalman_rpm2.filter(rpm2)

        # return filtered_rpm1, filtered_rpm2
    def callback_reset_odom(self, req):
        if req.data:
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0

            self.vx = 0.00
            self.vy = 0.00
            self.vth = 0.00

            self.current_time = rospy.Time.now()
            self.last_time = rospy.Time.now()


            self.m1read = False
            self.m2read = False
            self.m1 = 0.00
            self.m2 = 0.00
        return True, "odom reset success!!!"

    def send_can_message(self, can_id, data):
        message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(message)

    def get_rpm(self):
        # self.send_can_message(0x601, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        # time.sleep(0.01)
        # self.send_can_message(0x602, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])

        rpm1, rpm2 = 0, 0
        received_rpm1 = False
        received_rpm2 = False
        timeout = rospy.Time.now() + rospy.Duration(1.0)

        while True: #not (received_rpm1 and received_rpm2):
            frame = self.bus.recv(timeout=0.1)
            if frame is None:
                rospy.logwarn("Timeout waiting for CAN message")
                break

            if frame.arbitration_id == 0x181:
                    try:
                        m1 = unpack('<i', frame.data[0:4])[0]
                        rpm1 = (m1 * 1875 / (512 * 2500 * 4)) * 1 #self.velocity_unit_rpm
                        # print(f"[get_rpm]: rpm1 {rpm1}")
                        # if m1 >= 10:
                        received_rpm1 = True
                        # Filtering logic
                        # if -10 <= rpm1 <= 10:  # If RPM is between 0 and 10
                        #     rpm1 = 0
                        #     rospy.loginfo_throttle(1, f"rpm1: {int(rpm1)}")
                        # else:  # If RPM is greater than 10
                        #     rospy.loginfo_throttle(1, f"rpm1: {int(rpm1)}")

                    except Exception as e:
                        rospy.logerr(f"Error parsing frame data: {e}")
            if frame.arbitration_id == 0x182:
                    try:
                        m2 = unpack('<i', frame.data[0:4])[0]
                        rpm2 = (m2 * 1875 / (512 * 2500 * 4)) * 1 #self.velocity_unit_rpm
                        # print(f"[get_rpm]: rpm2 {rpm2}")
                        # if m2 >= 10:
                        received_rpm2 = True
                        # Filtering logic
                        # if -10 <= rpm1 <= 10:  # If RPM is between 0 and 10
                        #     rpm2 = 0
                        #     rospy.loginfo_throttle(0.1, f"rpm2: {int(rpm2)}")
                        # else:  # If RPM is greater than 10
                        #     rospy.loginfo_throttle(0.1, f"rpm2: {int(rpm2)}")

                    except Exception as e:
                        rospy.logerr(f"Error parsing frame data: {e}")

            if rospy.Time.now() > timeout:
                rospy.logerr("Failed to receive all RPM data within timeout")
                break

            if received_rpm1 and received_rpm2:
                break

        return rpm1/self.gear_ratio, rpm2/self.gear_ratio

    def getVelocities(self, rpm1, rpm2):
        # vel_x = ((rpm1 + rpm2) / 2.0) * self.wheel_circumference_ / 60.0
        # ang_z = ((rpm1 - rpm2) * self.wheel_circumference_ / self.wheels_y_distance_) / 60.0
        avg_rps_x = ((float)(rpm1-rpm2) / self.total_wheels_) / 60.000
        vel_x = avg_rps_x * self.wheel_circumference_
        vel_y = 0.0

        avg_rps_a = ((float)(rpm1 + rpm2) / self.total_wheels_) / 60.000
        ang_z = (avg_rps_a * self.wheel_circumference_) / ((self.wheels_x_distance_ / 2) + (self.wheels_y_distance_ /2 ))
        ang_z = -ang_z



        # right_vel_mps = (2 * math.pi * (self.wheel_diameter/2) * rpm1) / 60
        # left_vel_mps = (2 * math.pi * (self.wheel_diameter/2) * rpm2) / 60
        # vel_x = (right_vel_mps + left_vel_mps) * self.wheel_circumference_



        # print("{}".format(avg_rps_a))
        # ang_z = (right_vel_mps - left_vel_mps) / self.wheels_y_distance_

        # print(-vel_x, vel_y, ang_z)
        # print("vel_x:", vel_x)
        # print("vel_y:", vel_y)
        # print("vel_z:", ang_z)

        return vel_x, vel_y, ang_z

    def calculate_odom(self):
        rpm_1 = 0.00
        rpm_2 = 0.00
        rpm_m1, rpm_m2 = self.get_rpm()
        # print(f"rpm_m1: {rpm_m1}, rpm_2: {rpm_m2}")
        # print(rpm_m2)

        self.vx, self.vy, self.vth = self.getVelocities(rpm_m1, rpm_m2)
        self.vx = 0.00 if abs(self.vx) <= 0.06 else self.vx
        self.vth = 0.00 if abs(self.vth) <= 0.1 else self.vth
        # print(f"rpm_m1: {round(rpm_m1, 3)}, rpm_2: {round(rpm_m2, 3)}, self.vx: {round(self.vx, 3)}, self.vth: {round(self.vth, 3)}")


        self.current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = -self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)


        self.publish_tf = True
        if(self.publish_tf):
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                self.base_frame,
                self.odom_frame
            )
        else:
            pass
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = self.odom_frame

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = self.base_frame
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        self.odom_pub.publish(odom)

        pose2d = Pose2D()
        pose2d.x = self.x
        pose2d.y = self.y
        pose2d.theta = self.vth

        self.last_time = self.current_time

    # def send_transform(self, x, y, child_frame_id):
    #     self.odom_broadcaster.sendTransform(
    #         (x,y,0.0),
    #         (0.0,0.0,0.0,1.0),
    #         rospy.Time.now(),
    #         child_frame_id,
    #         'base_link'
    #     )

if __name__ == '__main__':
    UpdateOdom()
    rospy.spin()