#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import can
import struct
from struct import *
import time
import math
from dynamic_reconfigure.server import Server
from kinco_control.cfg import MotorConfig  # Assuming you have a dynamic_reconfigure config file
import tf
from nav_msgs.msg import Odometry
class MotorController:
    def __init__(self):
        # Initializing parameters with defaults or from parameter server
        self.device_name = rospy.get_param("~device_name", "kinco")
        self.wheels_x_distance_ = rospy.get_param("~wheels_x_distance", 0.0)
        self.wheels_y_distance_ = rospy.get_param("~wheels_y_distance", 0.5)
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.2)
        self.max_motor_rpm = rospy.get_param("~max_motor_rpm", 3000.0)
        self.gear_ratio = rospy.get_param("~gear_ratio", 20.00)
        self.total_wheels_ = rospy.get_param("~total_wheels", 2.0)
        self.wheel_circumference_ = math.pi * self.wheel_diameter

        # self.can_interface = rospy.get_param("~can_interface", "can0")
        self.can_interface = 'can0'
        self.can_bitrate = rospy.get_param("~can_bitrate", "500000")
        self.bus = can.interface.Bus(interface='socketcan', channel='can0')

        self.motor_1_id = rospy.get_param("~motor_1_id", "1")
        self.motor_2_id = rospy.get_param("~motor_2_id", "2")

        # odometry publisher param
        if not rospy.has_param("~publish_tf"):
            rospy.set_param("~publish_tf", True)
        if not rospy.has_param("~base_frame"):
            rospy.set_param("~base_frame", "base_footprint")
        if not rospy.has_param("~odom_frame"):
            rospy.set_param("~odom_frame", "odom")
        self.odom_frame = rospy.get_param("~odom_frame")
        self.base_frame = rospy.get_param("~base_frame")
        self.publish_tf = rospy.get_param("~publish_tf")

        self.g_req_vel = Twist()
        self.g_prev_command_time = 0
        self.COMMAND_RATE = 20

        # Motor configuration
        self.motor_1_id = 1
        self.motor_2_id = 2

        # Initialize the ROS node
        rospy.init_node('motor_controller', anonymous=True)
        self.subscription = rospy.Subscriber('cmd_vel', Twist, self.callback_vel)

        # CAN Bus setup
        self.can_interface = 'can0'
        self.can_bitrate = 500000
        self.can_bus = can.interface.Bus(self.can_interface, interface='socketcan')

        # Dynamic reconfigure server setup
        self.server = Server(MotorConfig, self.dyrecfg_callback)

        # Initialize motor
        self.init_motor()
        time.sleep(3000)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.main_control()
            rate.sleep()
        # self.close_CANnMotor()
    def init_motor(self):
        rospy.loginfo("Initializing motor...")
        if self.device_name == "kinco":
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

    # def cmd_vel_callback(self, msg):
    #     linear_velocity = msg.linear.x
    #     angular_velocity = msg.angular.z

    #     left_rpm, right_rpm = self.calculate_rpm(linear_velocity, angular_velocity)


    #     self.SDO_MotorSpin(self.motor_1_id, left_rpm)
    #     self.SDO_MotorSpin(self.motor_2_id, right_rpm)

    def main_control(self):
        # this block drives the robot based on defined rate
        if ((rospy.Time.from_sec(time.time()).to_nsec() - self.prev_control_time) >= (1000000000 / self.COMMAND_RATE)):
            self.moveBase()
            self.prev_control_time = rospy.Time.from_sec(time.time()).to_nsec()
        else:
            pass

        # this block stops the motor when no command is received
        if ((rospy.Time.from_sec(time.time()).to_nsec() - self.g_prev_command_time) >= 400000000):
            self.stopBase()
        else:
            pass

    def stopBase(self):
        self.g_req_vel.linear.x = 0.0
        self.g_req_vel.angular.z = 0.0

    def moveBase(self):
        # get the required rpm for each motor based on required velocities, and base used
        linear_velocity = self.g_req_vel.linear.x
        angular_velocity = self.g_req_vel.angular.z
        # req_motor_1, req_motor_2 = self.calculate_rpm(self.g_req_vel.linear.x, self.g_req_vel.angular.z)
        left_rpm, right_rpm = self.calculate_rpm(linear_velocity, angular_velocity)

        self.SDO_MotorSpin(self.motor_1_id, left_rpm)
        self.SDO_MotorSpin(self.motor_2_id, right_rpm)
        # self.get_rpm(self.motor_1_id)
        # self.get_rpm(self.motor_2_id)


    def callback_vel(self, msg):
        self.g_req_vel = msg
        self.g_prev_command_time = rospy.Time.from_sec(time.time()).to_nsec()
        linear_velocity = self.g_req_vel.linear.x
        angular_velocity = self.g_req_vel.angular.z
        # req_motor_1, req_motor_2 = self.calculate_rpm(self.g_req_vel.linear.x, self.g_req_vel.angular.z)
        left_rpm, right_rpm = self.calculate_rpm(linear_velocity, angular_velocity)

        self.SDO_MotorSpin(self.motor_1_id, left_rpm)
        self.SDO_MotorSpin(self.motor_2_id, right_rpm)
        # try:
        #     if self.current_motor_state != "Enabled Operation":
        #         if self.g_req_vel.linear.x != 0.00 or self.g_req_vel.angular.z != 0.00:
        #             rospy.logwarn("[matrix_CanOpenMotor_orientalBLVD-KRD]: Motor in <Switch on disabled> state cannot operation.Please go to <OPeration Enable> state")
        #             self.stopBase()
        #         else:
        #             pass
        #     else:
        #         pass
        # except:
        #     pass
    def calculate_rpm(self, linear_x, angular_z):
        # convert m/s to m/min
        linear_vel_x_mins = linear_x * 60
        linear_vel_y_mins = 0

        # convert rad/s to rad/min
        angular_vel_z_mins = angular_z * 60

        tangential_vel = angular_vel_z_mins * ((self.wheels_x_distance_ / 2) + (self.wheels_y_distance_ / 2))
        x_rpm = linear_vel_x_mins / self.wheel_circumference_
        y_rpm = linear_vel_y_mins / self.wheel_circumference_
        tan_rpm = tangential_vel / self.wheel_circumference_

        rpm_motor_left = x_rpm + y_rpm - tan_rpm
        rpm_motor_right = -(x_rpm + y_rpm + tan_rpm)

        rpm_motor_left *= self.gear_ratio
        rpm_motor_right *= self.gear_ratio

        rpm_motor_left = min(self.max_motor_rpm, max(-self.max_motor_rpm, rpm_motor_left))
        rpm_motor_right = min(self.max_motor_rpm, max(-self.max_motor_rpm, rpm_motor_right))

        value_motor_left = rpm_motor_left
        value_motor_right = rpm_motor_right
        return [value_motor_left, value_motor_right]
        # return int(rpm_motor_left), int(rpm_motor_right)

    def SDO_MotorSpin(self, MotorID, target_vel):
        success = False

        # Convert target velocity to 4-byte hexadecimal (LSB -> MSB)

        # objects DEC=[(RPM*512*Encoder_Resolution)/1875]
        # print(type(target_vel))
        objects_DEC = (int(target_vel)*512*2500*4)/1875
        vel_hex = struct.pack('<i', int(objects_DEC))
        data_frame = [0x23, 0xff, 0x60, 0x00, vel_hex[0], vel_hex[1], vel_hex[2], vel_hex[3] ]
        # Send the CAN message
        success = self.SendToCan(0x600 + MotorID, data_frame)
        # if success:
        #     rospy.loginfo(f"Motor {MotorID} set to target velocity {target_vel}")
        #     # self.read_speed(MotorID)
        # else:
        #     rospy.logerr(f"Failed to set target velocity for Motor {MotorID}")
        return success

    def SendToCan(self, arbitration_id, data_frame):
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data_frame, is_extended_id=False)
            self.can_bus.send(msg)
            return True
        except can.CanError as e:
            rospy.logerr(f"CAN message sending failed: {e}")
            return False

    # def read_speed(self,MotorID):
    #     rq = [0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00] #read speed realtime from motor
    #     # Send the CAN message
    #     self.SendToCan(0x600 + MotorID, rq)

    def get_rpm(self):
        self.send_can_message(0x601, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.01)
        self.send_can_message(0x602, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])

        rpm1 = 0
        rpm2 = 0

        try:
            frame = self.bus.recv(timeout=0.1)
            if frame is None:
                rospy.logerr("CAN read error")
                return rpm1, rpm2

            if frame.arbitration_id == 0x581:
                # rospy.loginfo(f"Received frame from motor 1 (ID 0x581): {frame.data}")
                if frame.data[0:4] == bytearray([0x43, 0x6c, 0x60, 0x00]):
                    m1 = unpack('<i', frame.data[4:8])[0]
                    rpm1 = (m1 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    # rospy.loginfo(f"rpm1: {rpm1}")

            if frame.arbitration_id == 0x582:
                # rospy.loginfo(f"Received frame from motor 2 (ID 0x582): {frame.data}")
                if frame.data[0:4] == bytearray([0x43, 0x6c, 0x60, 0x00]):
                    m2 = unpack('<i', frame.data[4:8])[0]
                    rpm2 = (m2 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    # rospy.loginfo(f"rpm2: {rpm2}")

            return rpm1, rpm2

        except can.CanError as e:
            rospy.logerr(f"CAN error occurred: {e}")
            return rpm1, rpm2

    def dyrecfg_callback(self, config, level):
        self.max_motor_rpm = config['max_motor_rpm']
        self.gear_ratio = config['gear_ratio']
        self.wheel_diameter = config['wheel_diameter']
        self.wheels_x_distance_ = config['wheels_x_distance']
        self.wheels_y_distance_ = config['wheels_y_distance']

        rospy.loginfo(f"Updated Motor Parameters: RPM: {self.max_motor_rpm}, Gear Ratio: {self.gear_ratio}")
        return config


if __name__ == "__main__":
    try:
        MotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
