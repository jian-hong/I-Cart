from turtle import right
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Bool,UInt32
from robocon_2025_interfaces.msg import Joystick, IMU
import serial
import time
import math
import struct

class LocomotionControl(Node):
    def __init__(self):
        super().__init__('locomotion_control')
        
        # Initialize serial port
        self.header_byte = 0x59
        self.serial_port = None
        # Initialize serial port settings
        self.SERIAL_PORT = '/dev/ttyUSB0'  # Change this to match your Arduino's serial port
        self.BAUD_RATE = 115200
        
        self.attempt_to_connect()
        
        self.default_cbg = MutuallyExclusiveCallbackGroup()

        self.locomotion_subscription = self.create_subscription(
            Joystick, 'joystick', self.locomotion_command_callback, 10, callback_group=self.default_cbg)
        
        self.create_timer(0.05, self.send_data, callback_group=self.default_cbg)  # Send at 10Hz

        # Initialize variables for simple wheel control
        self.left_wheel_speed = 0   # Range: -127 to 127
        self.right_wheel_speed = 0  # Range: -127 to 127
        self.data_changed = False   # Flag to track if data has changed
        self.last_sent_left = 999   # Track last sent values to avoid spam
        self.last_sent_right = 999

    def attempt_to_connect(self):
        while True:
            try:
                self.serial_port = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
                self.get_logger().info(f'Successfully connected to {self.SERIAL_PORT}')
                break
            except (OSError, serial.SerialException) as e:
                self.get_logger().error(f"Serial communication error: {e}")
                time.sleep(1)

    def send_data(self):
        if not self.serial_port.is_open:
            self.attempt_to_connect()

        if not self.data_changed and (self.left_wheel_speed == 0 and self.right_wheel_speed == 0):
            return
        # Create 4-byte packet: [header, left_speed, right_speed, checksum]
        packet = bytearray(4)
        packet[0] = self.header_byte  # 0x59
        
        # Convert signed speeds to unsigned bytes
        packet[1] = (self.left_wheel_speed + 256) % 256   # Handle negative properly
        packet[2] = (self.right_wheel_speed + 256) % 256 
        
        # Calculate simple checksum (sum of first 3 bytes modulo 256)
        checksum = (packet[0] + packet[1] + packet[2]) % 256
        packet[3] = checksum
        
        try:
            self.serial_port.write(packet)
            if (self.last_sent_left != self.left_wheel_speed or 
                self.last_sent_right != self.right_wheel_speed):
                
                if self.left_wheel_speed != 0 or self.right_wheel_speed != 0:
                    self.get_logger().info(f'Sending: Left={self.left_wheel_speed}, Right={self.right_wheel_speed}')
                else:
                    self.get_logger().info('Sending: STOP command')
                
                self.last_sent_left = self.left_wheel_speed
                self.last_sent_right = self.right_wheel_speed
            
            self.data_changed = False
            
        except (OSError, serial.SerialException):
            self.attempt_to_connect()


    def locomotion_command_callback(self, msg):
        # Simple tank drive control from joystick
        # Use left stick Y for forward/backward, right stick X for turning
        
        left_speed = -msg.ly  # Forward/backward movement
        right_speed = msg.ry      # Left/right turning
        
        new_left = max(-127, min(127, int(left_speed)))
        new_right = max(-127, min(127, int(right_speed)))
        
        # Check if values changed
        if (new_left != self.left_wheel_speed or new_right != self.right_wheel_speed):
            self.left_wheel_speed = new_left
            self.right_wheel_speed = new_right
            self.data_changed = True
            
            # Only log when moving
            if self.left_wheel_speed != 0 or self.right_wheel_speed != 0:
                self.get_logger().info(f'Joystick: ly={msg.ly}, ry={msg.ry} -> Wheels: L={self.left_wheel_speed}, R={self.right_wheel_speed}')

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