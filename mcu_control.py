import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robocon_2025_interfaces.msg import Joystick
import serial
import time

class LocomotionControl(Node):
    def __init__(self):
        super().__init__('locomotion_control')
        
        # Initialize serial port
        self.header_byte = 0x59
        self.serial_port = None
        # IMPORTANT: Update this to match your actual Arduino port
        self.SERIAL_PORT = '/dev/ttyUSB0'  # Change to '/dev/ttyACM0' or 'COM9' if needed
        self.BAUD_RATE = 115200
        
        self.attempt_to_connect()
        
        self.default_cbg = MutuallyExclusiveCallbackGroup()

        self.locomotion_subscription = self.create_subscription(
            Joystick, 'joystick', self.locomotion_command_callback, 10, callback_group=self.default_cbg)
        
        # Send test data every 3 seconds to verify connection
        self.create_timer(3.0, self.send_test_data, callback_group=self.default_cbg)
        self.create_timer(0.1, self.send_data, callback_group=self.default_cbg)  # Send at 10Hz

        # Initialize variables for wheel control
        self.left_wheel_speed = 0   # Range: -127 to 127
        self.right_wheel_speed = 0  # Range: -127 to 127
        self.data_changed = False   # Flag to track if data has changed
        self.last_sent_left = 999   # Track last sent values to avoid spam
        self.last_sent_right = 999
        self.test_counter = 0

    def attempt_to_connect(self):
        self.get_logger().info(f'Attempting to connect to {self.SERIAL_PORT}...')
        max_attempts = 5
        
        for attempt in range(max_attempts):
            try:
                self.serial_port = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
                self.get_logger().info(f'Successfully connected to {self.SERIAL_PORT}')
                return True
            except (OSError, serial.SerialException) as e:
                self.get_logger().error(f"Attempt {attempt+1}/{max_attempts}: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(2)
        
        self.get_logger().error(f"Failed to connect to {self.SERIAL_PORT} after {max_attempts} attempts")
        self.get_logger().error("Please check:")
        self.get_logger().error("1. Arduino is connected and powered")
        self.get_logger().error("2. Correct serial port (try /dev/ttyACM0 or COM9)")
        self.get_logger().error("3. No other programs using the serial port")
        return False

    def send_test_data(self):
        """Send test data to verify Arduino connection"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn("Serial port not available for test data")
            return
        
        # Cycle through test values
        test_values = [
            (10, -10),   # Left forward, Right backward
            (-10, 10),   # Left backward, Right forward
            (20, 20),    # Both forward
            (0, 0)       # Stop
        ]
        
        left_test, right_test = test_values[self.test_counter % len(test_values)]
        self.test_counter += 1
        
        # Create test packet
        packet = bytearray(4)
        packet[0] = self.header_byte  # 0x59
        packet[1] = (left_test + 256) % 256
        packet[2] = (right_test + 256) % 256
        packet[3] = (packet[0] + packet[1] + packet[2]) % 256  # checksum
        
        try:
            self.serial_port.write(packet)
            self.get_logger().info(f'TEST DATA: Left={left_test}, Right={right_test}, Packet=[0x{packet[0]:02x}, {packet[1]}, {packet[2]}, {packet[3]}]')
        except Exception as e:
            self.get_logger().error(f'Failed to send test data: {e}')

    def send_data(self):
        if not self.serial_port or not self.serial_port.is_open:
            return

        # Only send if data has changed
        if not self.data_changed:
            return

        # Create 4-byte packet: [header, left_speed, right_speed, checksum]
        packet = bytearray(4)
        packet[0] = self.header_byte  # 0x59
        
        # Convert signed speeds to unsigned bytes for transmission
        packet[1] = (self.left_wheel_speed + 256) % 256
        packet[2] = (self.right_wheel_speed + 256) % 256
        
        # Calculate checksum
        checksum = (packet[0] + packet[1] + packet[2]) % 256
        packet[3] = checksum
        
        try:
            self.serial_port.write(packet)
            
            # Log when values change
            if (self.last_sent_left != self.left_wheel_speed or 
                self.last_sent_right != self.right_wheel_speed):
                
                self.get_logger().info(f'JOYSTICK DATA: Left={self.left_wheel_speed}, Right={self.right_wheel_speed}, Packet=[0x{packet[0]:02x}, {packet[1]}, {packet[2]}, {packet[3]}]')
                
                self.last_sent_left = self.left_wheel_speed
                self.last_sent_right = self.right_wheel_speed
            
            self.data_changed = False
            
        except (OSError, serial.SerialException) as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.attempt_to_connect()

    def locomotion_command_callback(self, msg):
        # FIXED: Independent wheel control
        # Left wheel controlled by left joystick Y (ly)
        # Right wheel controlled by right joystick Y (ry)
        
        left_speed = msg.ly   # Left joystick Y controls left wheel
        right_speed = msg.ry  # Right joystick Y controls right wheel
        
        # Clamp to valid range (-127 to 127) to fit in int8_t
        new_left = max(-127, min(127, int(left_speed)))
        new_right = max(-127, min(127, int(right_speed)))
        
        # Check if values changed
        if (new_left != self.left_wheel_speed or new_right != self.right_wheel_speed):
            self.left_wheel_speed = new_left
            self.right_wheel_speed = new_right
            self.data_changed = True
            
            # Log joystick input and wheel output
            self.get_logger().info(f'Joystick Input: ly={msg.ly}, ry={msg.ry} -> Wheel Commands: L={self.left_wheel_speed}, R={self.right_wheel_speed}')

    def __del__(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    locomotion_ctrl = LocomotionControl()
    try:
        rclpy.spin(locomotion_ctrl)
    except KeyboardInterrupt:
        pass
    locomotion_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
