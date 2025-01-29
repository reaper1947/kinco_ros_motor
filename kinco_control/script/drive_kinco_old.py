#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import can
import struct
import time


class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.subscription = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        self.can_interface = 'can0'
        self.can_bitrate = 500000
        self.can_bus = can.interface.Bus(self.can_interface, interface='socketcan')

        self.motor_1_id = 1
        self.motor_2_id = 2
        self.accel_time = 100

        # Parameters for RPM calculation
        self.max_rpm = 30
        self.gear_ratio = 2
        self.wheel_circumference_ = 0.628  # Example value (in meters)
        self.wheels_x_distance_ = 0.0  # Distance between left and right wheels (in meters)
        self.wheels_y_distance_ = 0.5  # Ignored for a two-wheel setup
        self.wheel_diameter_ = 0.2
        self.device_name = "motor"
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
        self.send_can_message(left_rpm, right_rpm)

    def calculate_rpm(self, linear_x, angular_z):
        linear_vel_x_mins = linear_x * 60
        linear_vel_y_mins = 0
        angular_vel_z_mins = angular_z * 60

        tangential_vel = angular_vel_z_mins * ((self.wheels_x_distance_ / 2) + (self.wheels_y_distance_ / 2))
        x_rpm = linear_vel_x_mins / self.wheel_circumference_
        y_rpm = linear_vel_y_mins / self.wheel_circumference_
        tan_rpm = tangential_vel / self.wheel_circumference_

        rpm_motor_left = x_rpm + y_rpm - tan_rpm
        rpm_motor_right = -(x_rpm + y_rpm + tan_rpm)

        rpm_motor_left *= self.gear_ratio
        rpm_motor_right *= self.gear_ratio

        rpm_motor_left = min(self.max_rpm, max(-self.max_rpm, rpm_motor_left))
        rpm_motor_right = min(self.max_rpm, max(-self.max_rpm, rpm_motor_right))

        return int(rpm_motor_left), int(rpm_motor_right)

    def send_can_message(self, left_rpm, right_rpm):
        left_motor_msg = can.Message(arbitration_id=0x601, data=[0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, left_rpm & 0xFF, (left_rpm >> 8) & 0xFF], is_extended_id=False)
        right_motor_msg = can.Message(arbitration_id=0x602, data=[0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, right_rpm & 0xFF, (right_rpm >> 8) & 0xFF], is_extended_id=False)

        try:
            self.can_bus.send(left_motor_msg)
            self.can_bus.send(right_motor_msg)
            rospy.loginfo(f"Sent RPM: Left={left_rpm}, Right={right_rpm}")
        except can.CanError as e:
            rospy.logerr(f"CAN message sending failed: {e}")


if __name__ == "__main__":
    try:
        MotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
