
    def DiagnosticsStatusWordnPublish(self):
            received_181 = False
            received_182 = False
            timeout = rospy.Time.now() + rospy.Duration(1.0)

            while not (received_181 and received_182):
                frame = self.bus.recv(timeout=0.1)
                if frame is None:
                    rospy.logwarn("Timeout waiting for CAN message")
                    break
                if frame.arbitration_id == 0x181:
                    sw_h_181 = bytearray([frame.data[4], frame.data[5]])
                    sw_d_181 = unpack('<h', sw_h_181)
                    sw_d_181 = sw_d_181[0]
                    sw_b_181 = bin(sw_d_181)[2:].zfill(16)
                    STW_ReadytoSwitchON_181 = int(sw_b_181[len(sw_b_181)-1])
                    STW_SwitchedON_181 = int(sw_b_181[len(sw_b_181)-2])
                    STW_OperationEnabled_181 = int(sw_b_181[len(sw_b_181)-3])
                    STW_Fault_181 = int(sw_b_181[len(sw_b_181)-4])
                    STW_VoltageEnabled_181 = int(sw_b_181[len(sw_b_181)-5])
                    STW_QuickStop_181 = int(sw_b_181[len(sw_b_181)-6])
                    STW_SwitchONDisabled_181 = int(sw_b_181[len(sw_b_181)-7])
                    STW_Warning_181 = int(sw_b_181[len(sw_b_181)-8])
                    received_181 = True

                if frame.arbitration_id == 0x182:
                    sw_h_182 = bytearray([frame.data[4], frame.data[5]])
                    sw_d_182 = unpack('<h', sw_h_182)
                    sw_d_182 = sw_d_182[0]
                    sw_b_182 = bin(sw_d_182)[2:].zfill(16)

                    STW_ReadytoSwitchON_182 = int(sw_b_182[len(sw_b_182)-1])
                    STW_SwitchedON_182 = int(sw_b_182[len(sw_b_182)-2])
                    STW_OperationEnabled_182 = int(sw_b_182[len(sw_b_182)-3])
                    STW_Fault_182 = int(sw_b_182[len(sw_b_182)-4])
                    STW_VoltageEnabled_182 = int(sw_b_182[len(sw_b_182)-5])
                    STW_QuickStop_182 = int(sw_b_182[len(sw_b_182)-6])
                    STW_SwitchONDisabled_182 = int(sw_b_182[len(sw_b_182)-7])
                    STW_Warning_182 = int(sw_b_182[len(sw_b_182)-8])
                    received_182 = True

                if received_181 and received_182:
                    rospy.loginfo("0x181 and 0x182 ::: DONE")
                    if STW_Fault_181 and STW_Fault_182:
                        print("Fault")
                        self.current_motor_state = "Fault"
                        self.motor_status.state = DeviceStateCanopen.STATE_FAULT
                    elif STW_ReadytoSwitchON_181 and STW_ReadytoSwitchON_182 and STW_SwitchedON_181 and STW_SwitchedON_182 and STW_OperationEnabled_181 and STW_OperationEnabled_182:
                        print("Enabled Operation")
                        self.current_motor_state = "Enabled Operation"
                        self.motor_status.state = DeviceStateCanopen.STATE_OPERATION_ENABLED
                    elif STW_SwitchONDisabled_181 and STW_SwitchONDisabled_182:
                        print("Switch ON Disabled")
                        self.current_motor_state = "Switech on disabled"
                        self.motor_status.state = DeviceStateCanopen.STATE_SWITCH_ON_DISABLED
                    else:
                        print("UNKNOW")
                        self.current_motor_state = "UNKNOW"
                        self.motor_status.state = DeviceStateCanopen.STATE_UNKNOW


                    # print("motor state ---- {}, mode {}".format(self.motor_status.text, self.motor_status.state))
                    array = []
                    array.append(message.data[4])
                    array.append(message.data[5])

                    self.motor_status.status_word = array


        except:
            self.current_motor_state = "UNKNOW"
            self.motor_status.state = DeviceStateCanopen.STATE_UNKNOW

        try:
            # print(message.arbitration_id)
            if message.id == (0x080 + self.motor_1_id) or message.id == (0x080 + self.motor_2_id):
                self.current_motor_state = "Fault"
                self._isFault_Detected = True
                self.motor_status.state = DeviceStateCanopen.STATE_FAULT
                self.motor_status.fault_state = message.data
                self.motor_status.status_word = []
        except:
            self.current_motor_state = "UNKNOW"
            self.motor_status.state = DeviceStateCanopen.STATE_UNKNOW

        self.motor_status.text = self.current_motor_state
        self.pub_motor_state.publish(self.motor_status)


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