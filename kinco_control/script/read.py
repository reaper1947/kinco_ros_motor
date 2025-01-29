import rospy
import can
from struct import unpack
from std_msgs.msg import Float32

class MotorController:
    def __init__(self):
        # ตั้งค่า CAN interface
        self.bus = can.interface.Bus(interface='socketcan', channel='can0')

        # ตั้งค่า publisher
        self.rpm_pub = rospy.Publisher('motor_rpm', Float32, queue_size=1)

        # ตั้งค่า node
        rospy.init_node('motor_controller', anonymous=True)
        rospy.Timer(rospy.Duration(0.00025), self.read_rpm)

        # กำหนดค่า
        self.gear_ratio = 2.0
        self.velocity_unit_rpm = 1.0

    def send_can_message(self, can_id, data):
        message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(message)

    def read_rpm(self):
        # ส่งคำสั่งเพื่ออ่าน RPM จากมอเตอร์
        self.send_can_message(0x601, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.send_can_message(0x602, [0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])

        rpm1 = 0
        rpm2 = 0

        try:
            frame = self.bus.recv()  # รับข้อมูลจาก CAN bus
            if frame is None:
                rospy.logerr("CAN read error")
                return 0.0

            # ตรวจสอบว่าเป็นข้อมูลของมอเตอร์ 1 หรือ 2 และดึงค่าจากตำแหน่งที่ 4 และ 5
            if frame.arbitration_id == 0x581:
                rospy.loginfo(f"Received frame from motor 1 (ID 0x581): {frame.data}")  # แสดงข้อมูลจาก ID 0x581
                if (frame.data[0] == 0x43 and frame.data[1] == 0x6c and frame.data[2] == 0x60 and frame.data[3] == 0x00):
                    # อ่านข้อมูลจากตำแหน่งที่ 4, 5, 6, 7 และแปลงเป็น signed 32-bit integer
                    m1 = bytearray(frame.data[4:8])  # ใช้ 4 ไบต์จากตำแหน่งที่ 4 ถึง 7
                    m1 = unpack('<i', m1)  # แปลงเป็น signed 32-bit integer
                    m1 = (m1[0]*1875/(512*2500*4)) * self.velocity_unit_rpm  # คำนวณ RPM
                    rpm1 = m1  # กำหนดค่ารอบมอเตอร์ 1
                    rospy.loginfo(f"rpm1: {rpm1}")  # แสดงข้อมูลจาก ID 0x581

            if frame.arbitration_id == 0x582:
                rospy.loginfo(f"Received frame from motor 2 (ID 0x582): {frame.data}")  # แสดงข้อมูลจาก ID 0x582
                if (frame.data[0] == 0x43 and frame.data[1] == 0x6c and frame.data[2] == 0x60 and frame.data[3] == 0x00):
                    # อ่านข้อมูลจากตำแหน่งที่ 4, 5, 6, 7 และแปลงเป็น signed 32-bit integer
                    m2 = bytearray(frame.data[4:8])  # ใช้ 4 ไบต์จากตำแหน่งที่ 4 ถึง 7
                    m2 = unpack('<i', m2)  # แปลงเป็น signed 32-bit integer
                    m2 = (m2[0]*1875/(512*2500*4)) * self.velocity_unit_rpm
                    rpm2 = m2
                    rospy.loginfo(f"rpm2: {rpm2}")

            # คำนวณ RPM เฉลี่ย
            # avg_rpm = (rpm1 + rpm2) / 2.0
            # self.rpm_pub.publish(avg_rpm)
            # return avg_rpm

        except can.CanError:
            rospy.logerr("CAN error occurred")
            return 0.0

if __name__ == '__main__':
    controller = MotorController()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        controller.read_rpm()
        rate.sleep()
