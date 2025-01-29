from struct import unpack
import can
import time

class ReadTPDO:
    def __init__(self, can_interface='can0', bitrate=500000):
        self.can_interface = can_interface
        self.can_bitrate = bitrate
        self.bus = can.interface.Bus(interface='socketcan', channel=self.can_interface)
        self.velocity_unit_rpm = 1.0

    def get_rpm(self):
        rpm1 = 0.0
        received_rpm1 = False
        timeout = time.time() + 1.0
        # time.sleep(0.01)

        while not received_rpm1:
            frame = self.bus.recv(timeout=0.1)
            if frame is None:
                print("Timeout waiting for CAN message")
                break

            if frame.arbitration_id == 0x181:
                try:
                    m1 = unpack('<i', frame.data[0:4])[0]
                    rpm1 = (m1 * 1875 / (512 * 2500 * 4)) * self.velocity_unit_rpm
                    received_rpm1 = True
                    # print(f"RPM1: {rpm1}")
                except Exception as e:
                    print(f"Error parsing frame data: {e}")

            if time.time() > timeout:
                print("Failed to receive RPM data within timeout")
                break

        return rpm1

    def close_bus(self):
        if self.bus is not None:
            self.bus.shutdown()
            print("CAN bus shutdown successfully.")

if __name__ == '__main__':
    reader = ReadTPDO()
    try:
        while True:
            rpm = reader.get_rpm()
            print(f"Final RPM: {int(rpm)}")
            # time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping RPM reader...")
    finally:
        reader.close_bus()
