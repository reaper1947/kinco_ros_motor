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

        # Motor configuration
        self.motor_1_id = 1
        self.motor_2_id = 2
        self.motor_3_id = 3

        # Initialize the ROS node
        rospy.init_node('motor_controller', anonymous=True)
        self.subscription = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # CAN Bus setup
        self.can_interface = 'can0'
        self.can_bitrate = 500000
        # self.can_bus = can.interface.Bus(self.can_interface, interface='socketcan')
        filters = [
            {"can_id": 0x181, "can_mask": 0x7FF, "extended": False},
            {"can_id": 0x182, "can_mask": 0x7FF, "extended": False},
        ]

        self.can_bus = can.interface.Bus(interface='socketcan', channel='can0', can_filters=filters)
        # Dynamic reconfigure server setup
        self.server = Server(MotorConfig, self.dyrecfg_callback)

        # Initialize motor
        self.init_motor()


        rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        #     self.get_rpm()
        #     rate.sleep()


    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        left_rpm, right_rpm = self.calculate_rpm(linear_velocity, angular_velocity)


        self.SDO_MotorSpin(self.motor_1_id, left_rpm)
        self.SDO_MotorSpin(self.motor_2_id, right_rpm)



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

    def dyrecfg_callback(self, config, level):
        self.max_motor_rpm = config['max_motor_rpm']
        self.gear_ratio = config['gear_ratio']
        self.wheel_diameter = config['wheel_diameter']
        self.wheels_x_distance_ = config['wheels_x_distance']
        self.wheels_y_distance_ = config['wheels_y_distance']

        # Real-time RPM adjustment
        left_motor_rpm = config['left_motor_rpm']
        right_motor_rpm = config['right_motor_rpm']
        lift_motor_rpm = config['lift_motor_rpm']

        # Send RPM values to the motors
        self.SDO_MotorSpin(self.motor_1_id, left_motor_rpm)
        self.SDO_MotorSpin(self.motor_2_id, right_motor_rpm)
        self.SDO_MotorSpin(self.motor_3_id, lift_motor_rpm)

        rospy.loginfo(f"Updated Parameters: RPM: {self.max_motor_rpm}, Gear Ratio: {self.gear_ratio}")
        rospy.loginfo(f"Real-Time Speed Adjustments: Left Motor: {left_motor_rpm}, Right Motor: {right_motor_rpm}")
        rospy.loginfo(f"Real-Time Speed Adjustments: Lite Motor: {lift_motor_rpm}")

        rospy.loginfo(f"Updated Motor Parameters: RPM: {self.max_motor_rpm}, Gear Ratio: {self.gear_ratio}")
        return config


    #*********************************************************#
    #
    #
    #               Motor control API
    #
    #
    #*********************************************************#

    def init_motor(self):
        success = False
        rospy.loginfo("Initializing motor...")
        if self.device_name == "kinco":

            try:
                commands = [
                    [0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00],
                    [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00],
                    # [0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00],
                    # [0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00],
                    # [0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00],
                    # [0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00],
                    [0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00], # Shutdown
                    [0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00], # Switch ON
                    [0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00], # Switch ON + Enable Operation
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

    # def init_motor(self, MotorID): # Transitions 3 + 4 *
    #     success = False

    #     # set Acceleration & Deceleration time
    #     # for Driver -> kinco
    #     if self.device_name == "kinco":
    #         # set Profile Velocity Mode
    #         self.SDO_ModeofOperation(MotorID, "Profile Velocity Mode")
    #         time.sleep(0.1)
    #         self.SDOsetAccelDecelCAN(MotorID, self.accel_time, self.decel_time, index_motor=self.motor_1_id)
    #         time.sleep(0.1)
    #         self.SDOsetAccelDecelCAN(MotorID, self.accel_time, self.decel_time, index_motor=self.motor_2_id)
    #         time.sleep(0.1)
    #     # for CANOPEN Motor driver -> SmartrisLefertDrives, OrientalmotorBLVD_KRD
    #     else:
    #         # set Profile Velocity Mode
    #         self.SDO_ModeofOperation(MotorID, "Profile Velocity Mode")
    #         time.sleep(0.1)
    #         self.SDOsetAccelDecelCAN(MotorID, self.accel_time, self.decel_time)
    #         time.sleep(0.1)

    #     # Go to --> Operation Enable
    #     #                                                              Follow instructions HP-5143E.pdf page 33 [5.1 Status Machine]
    #     #                                                                              current Status Machine
    #     #                                                                                Switch on disabled
    #     if self.MotorSwitched(MotorID, "Shutdown"):                 #Transitions 2 --->  Ready to switch on
    #         time.sleep(0.1)
    #         if self.MotorSwitched(MotorID, "Switch ON"):            #Transitions 3 --->  Switched on
    #             time.sleep(0.1)
    #             if self.MotorSwitched(MotorID, "Enable Operation"): #Transitions 4 --->  Operation enabled
    #                 success = True
    #                 rospy.loginfo("[matrix_CanOpenMotor_orientalBLVD-KRD]: Initial Motor-ID {} success!!!".format(MotorID))
    #             else:
    #                 rospy.logerr("[matrix_CanOpenMotor_orientalBLVD-KRD]: Error Enable Operation Motor-ID {} ".format(MotorID))
    #         else:
    #             rospy.logerr("[matrix_CanOpenMotor_orientalBLVD-KRD]: Error Switch ON Motor-ID {} ".format(MotorID))
    #     else:
    #         rospy.logerr("[matrix_CanOpenMotor_orientalBLVD-KRD]: Error Shutdown Motor-ID {} ".format(MotorID))
    #     return success


    def SDOclearErrorDualMotor(self):
        # for Driver -> kinco
        if self.device_name == "kinco":
            self.SDO_clearError(self.dual_motor_id)
        # for CANOPEN Motor driver -> SmartrisLefertDrives, OrientalmotorBLVD_KRD
        else:
            self.SDO_clearError(self.motor_1_id)
            self.SDO_clearError(self.motor_2_id)

    def SDO_clearError(self, MotorID):
        success = False
        if self.device_name == "OrientalmotorBLVD_KRD":
            data_frame = [0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00]
        elif self.device_name == "kinco":
            data_frame = [0x2b, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]
        success = self.SendToCan(0x600 + MotorID, data_frame)
        return success


    def MotorSwitched(self, MotorID, cmd):
        success = True
        data = 0x00
        if cmd == "Shutdown":                        #Transitions 2, 6, 8   “Shutdown” command received from controlword.
            data = 0x06
        elif cmd == "Switch ON":                     #Transitions 3         “Switch On” command received from controlword.
            data = 0x07
        elif cmd == "Switch ON + Enable Operation":  #Transitions 3, 4*
            data = 0x0F
        elif cmd == "Disable Voltage":               #Transitions 7, 9, 10, 12
            data = 0x00
        elif cmd == "Quick Stop":                    #Transitions 7, 10, 11
            data = 0x02
        elif cmd == "Disable Operation":             #Transitions 5         “Disable operation” command received from controlword.
            data = 0x03
        elif cmd == "Enable Operation":              #Transitions 4, 16     “Enable Operation” command received from controlword.
            data = 0x0F
        elif cmd == "Fault Reset":                   #Transitions 15
            data = 0x80
        else:
            success = False
        # * Automatic transition to enable operation state after executing switched on state functionality.
        if success:
            data_frame = [0x2B, 0x40, 0x60, 0x00, data, 0x00, 0x00, 0x00]
            success = self.SendToCan(0x600 + MotorID, data_frame)
            if success:
                rospy.loginfo("Succcess set Status Machine control commands Motor-ID {}, cmd {}".format(MotorID, cmd))
            else:
                rospy.logerr("Fail set Status Machine control commands Motor-ID {}, cmd {}".format(MotorID, cmd))
        else:
            rospy.logerr("Not found Status Machine control commands cmd {}".format(cmd))

        return success

    # def SDO_MotorSpin(self, MotorID, target_vel):
    #     success = False
    #     # Convert target velocity to 4-byte hexadecimal (LSB -> MSB)
    #     # objects DEC=[(RPM*512*Encoder_Resolution)/1875]
    #     objects_DEC = (int(target_vel)*512*2500*4)/1875
    #     vel_hex = struct.pack('<i', int(objects_DEC))
    #     data_frame = [0x23, 0xff, 0x60, 0x00, vel_hex[0], vel_hex[1], vel_hex[2], vel_hex[3] ]
    #     # Send the CAN message
    #     success = self.SendToCan(0x600 + MotorID, data_frame)
    #     # if success:
    #     #     rospy.loginfo(f"Motor {MotorID} set to target velocity {target_vel}")
    #     #     # self.read_speed(MotorID)
    #     # else:
    #     #     rospy.logerr(f"Failed to set target velocity for Motor {MotorID}")
    #     return success


    def SDO_MotorSpin(self, MotorID, target_vel):
        success = False
        # Convert target velocity to 4-byte hexadecimal (LSB -> MSB)
        # objects DEC=[(RPM*512*Encoder_Resolution)/1875]
        objects_DEC = (int(target_vel)*512*2500*4)/1875
        vel_hex = struct.pack('<i', int(objects_DEC))
        data_frame = [vel_hex[0], vel_hex[1],vel_hex[2], vel_hex[3], 0xFD] # vel_hex[2], vel_hex[3]
        # Send the CAN message
        success = self.SendToCan(0x170 + MotorID, data_frame)
        # if success:
        #     rospy.loginfo(f"Motor {MotorID} set to target velocity {target_vel}")
        #     # self.read_speed(MotorID)
        # else:
        #     rospy.logerr(f"Failed to set target velocity for Motor {MotorID}")
        return success

    def SDO_ModeofOperation(self, MotorID, mode):
        success = False
        data_frame = []
        if mode == "Profile Velocity Mode":
            data_frame = [0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]
            success = True
        else:
            success = False

        if success:
            success = self.SendToCan(0x600 + MotorID, data_frame)
        return success

    def SDO_SendReqMotorActualSpeed(self, MotorID):
        # for Driver -> kinco
        if self.device_name == "kinco":
            # kinco/kinco CANopen Communication Routine V1.05.pdf
            # kinco Serv vo Driver (Special for Hub Servo Motor) CANopen Communication Routine Version 1.05 | Page 17
            # Index 606Ch, Subindex 0x01, 0x02
            # Left and right target speed combination Low 16bits is left motor, High 16bits is rgiht motor
            # current speed of motor unit is 0.1r/min
            data_frame = [0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
        success = self.SendToCan(0x600 + MotorID, data_frame)
        return success
    def get_rpm(self):
        # self.send_can_message(0x601, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        # time.sleep(0.01)
        # self.send_can_message(0x602, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])

        rpm1, rpm2 = 0, 0
        received_rpm1 = False
        received_rpm2 = False
        timeout = rospy.Time.now() + rospy.Duration(1.0)

        while not (received_rpm1 and received_rpm2):
            frame = self.bus.recv(timeout=0.1)
            if frame is None:
                rospy.logwarn("Timeout waiting for CAN message")
                break

            if frame.arbitration_id == 0x181:
                    try:
                        m1 = unpack('<i', frame.data[0:4])[0]
                        rpm1 = (m1 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                        received_rpm1 = True
                        # Filtering logic
                        if -10 <= rpm1 <= 10:  # If RPM is between 0 and 10
                            rpm1 = 0
                            rospy.loginfo_throttle(1, f"rpm1: {int(rpm1)}")
                        else:  # If RPM is greater than 10
                            rospy.loginfo_throttle(1, f"rpm1: {int(rpm1)}")

                    except Exception as e:
                        rospy.logerr(f"Error parsing frame data: {e}")

            if frame.arbitration_id == 0x182:
                    try:
                        m2 = unpack('<i', frame.data[0:4])[0]
                        rpm2 = (m2 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                        received_rpm2 = True
                        # Filtering logic
                        if -10 <= rpm1 <= 10:  # If RPM is between 0 and 10
                            rpm2 = 0
                            rospy.loginfo_throttle(0.1, f"rpm2: {int(rpm2)}")
                        else:  # If RPM is greater than 10
                            rospy.loginfo_throttle(0.1, f"rpm2: {int(rpm2)}")

                    except Exception as e:
                        rospy.logerr(f"Error parsing frame data: {e}")

            if rospy.Time.now() > timeout:
                rospy.logerr("Failed to receive all RPM data within timeout")
                break

        return rpm1, rpm2


    def SDO_SendReqStatusWordDualMotor(self):
        # for Driver -> kinco
        if self.device_name == "kinco":
            self.SDO_SendReqStatusWord(self.motor_1_id)
            self.SDO_SendReqStatusWord(self.motor_2_id)

    def SDO_SendReqStatusWord(self, MotorID):
        data_frame = [0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
        success = self.SendToCan(0x600 + MotorID, data_frame)
        return success

    def SDO_DisableOperationEnabled(self, MotorID): # Transitions 11 -> 12
        success = False
        # Transitions 11
        # - “Quick Stop” command received from controlword.
        # - “QSTOP signal input” is active. (break)
        if self.MotorSwitched(MotorID, "Quick Stop"):
            # Transitions 12
            # - “Quick Stop” function is completed or “Disable Voltage” command received from controlword
            # - “HWTO signal input” is active
            if self.MotorSwitched(MotorID, "Disable Voltage"):
                success = True
        return success

    def NMT_service(self, MotorID, cmd):
        success = True
        COB_ID = 0x00
        Byte0_cmd = 0x00
        Byte1_NodeID = MotorID
        if cmd == "Switch to the <Operational> state":
            Byte0_cmd = 0x01
        elif cmd == "Switch to the <Stoped> state":
            Byte0_cmd = 0x02
        elif cmd == "Switch to the <Pre-operational> state":
            Byte0_cmd = 0x80
        elif cmd == "Reset Node":
            Byte0_cmd = 0x81
        elif cmd == "Reset Communication":
            Byte0_cmd = 0x82
        else:
            success = False

        if success:
            data_frame = [Byte0_cmd, MotorID]
            success = self.SendToCan(COB_ID, data_frame)
        else:
            pass

        return success

if __name__ == "__main__":
    try:
        MotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass