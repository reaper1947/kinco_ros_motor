#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import can
import struct
import time
import math
from math import sin, cos, pi

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.subscription = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.can_interface = 'can0'
        self.can_bitrate = 500000
        self.can_bus = can.interface.Bus(self.can_interface, interface='socketcan')

        self.motor_1_id = 1
        self.motor_2_id = 2
        self.accel_time = 100

        # Initialize variables
        self.x_ = 0.0  # Initial x position
        self.y_ = 0.0  # Initial y position
        self.th_ = 0.0  # Initial orientation (theta)
        self.last_time_ = rospy.Time.now()
        self.last_left_rpm_ = 0.0
        self.last_right_rpm_ = 0.0

        # Parameters for RPM calculation
        self.max_rpm = 3000
        self.gear_ratio = 2
        self.wheel_circumference_ = 0.628  # Example value (in meters)
        self.wheels_x_distance_ = 0.5  # Distance between left and right wheels (in meters)
        self.wheel_diameter_ = 0.2
        self.device_name = "motor"

        # Odometry parameters
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Heading
        self.last_time = rospy.Time.now()

        self.init_motor()

    def init_motor(self):
        rospy.loginfo("Initializing motor...")
        if self.device_name == "motor":
            try:
                commands = [
                    [0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00],
                    [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00],
                    [0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00],
                    [0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00],
                    [0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00],
                    [0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00],
                    [0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00],
                    [0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00],
                    [0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00],
                ]
                for command in commands:
                    msg = can.Message(arbitration_id=0x601, data=command, is_extended_id=False)
                    msg = can.Message(arbitration_id=0x602, data=command, is_extended_id=False)

                    self.can_bus.send(msg)
                    time.sleep(0.1)
                rospy.loginfo("Motor initialized successfully!")
            except can.CanError as e:
                rospy.logerr(f"CAN message sending failed during initialization: {e}")
        else:
            rospy.logwarn("Unsupported device!")

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        left_rpm, right_rpm = self.calculate_rpm(linear_velocity, angular_velocity)
        self.SDO_MotorSpin(self.motor_1_id, left_rpm)
        self.SDO_MotorSpin(self.motor_2_id, right_rpm)

        self.update_odometry(linear_velocity, angular_velocity)

    def calculate_rpm(self, linear_x, angular_z):
        # convert m/s to m/min
        linear_vel_x_mins = linear_x * 60
        linear_vel_y_mins = 0

        # convert rad/s to rad/min
        angular_vel_z_mins = angular_z * 60

        tangential_vel = angular_vel_z_mins * ((self.wheels_x_distance_ / 2))
        x_rpm = linear_vel_x_mins / self.wheel_circumference_
        y_rpm = linear_vel_y_mins / self.wheel_circumference_
        tan_rpm = tangential_vel / self.wheel_circumference_

        rpm_motor_left = x_rpm + y_rpm - tan_rpm
        rpm_motor_right = -(x_rpm + y_rpm + tan_rpm)

        rpm_motor_left *= self.gear_ratio
        rpm_motor_right *= self.gear_ratio

        rpm_motor_left = min(self.max_rpm, max(-self.max_rpm, rpm_motor_left))
        rpm_motor_right = min(self.max_rpm, max(-self.max_rpm, rpm_motor_right))

        return [rpm_motor_left, rpm_motor_right]

    def update_odometry(self, linear_velocity, angular_velocity):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Compute displacement
        delta_x = linear_velocity * dt
        delta_theta = angular_velocity * dt

        rospy.loginfo(f"delta_x: {delta_x}, delta_theta: {delta_theta}")

        # Update robot's position
        self.x += delta_x * cos(self.theta)
        self.y += delta_x * sin(self.theta)
        self.theta += delta_theta

        # Normalize theta to be within [-pi, pi]
        while self.theta > pi:
            self.theta -= 2 * pi
        while self.theta < -pi:
            self.theta += 2 * pi

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        # Set the position and orientation
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)

        # Set the velocity
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Update the last time
        self.last_time = current_time


    def SDO_MotorSpin(self, MotorID, target_vel):
        success = False

        # Convert target velocity to 4-byte hexadecimal (LSB -> MSB)
        vel_hex = struct.pack('<i', int(target_vel))

        if self.device_name == "motor":
            data_frame = [0x23, 0xff, 0x60, 0x00, 0x00, 0x00, vel_hex[0], vel_hex[1]]
        else:
            data_frame = [0x23, 0xff, 0x60, 0x00, 0x00, 0x00, vel_hex[0], vel_hex[1]]

        # Send the CAN message
        success = self.SendToCan(0x600 + MotorID, data_frame)
        if success:
            rospy.loginfo(f"Motor {MotorID} set to target velocity {target_vel}")
            self.read_speed(MotorID)
        else:
            rospy.logerr(f"Failed to set target velocity for Motor {MotorID}")
        return success

    def SendToCan(self, arbitration_id, data_frame):
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data_frame, is_extended_id=False)
            self.can_bus.send(msg)
            return True
        except can.CanError as e:
            rospy.logerr(f"CAN message sending failed: {e}")
            return False

    def read_speed(self, MotorID):
        if self.device_name == "motor":
            rq = [0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
        else:
            rq = [0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Send the CAN message
        self.SendToCan(0x600 + MotorID, rq)


if __name__ == "__main__":
    try:
        MotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
