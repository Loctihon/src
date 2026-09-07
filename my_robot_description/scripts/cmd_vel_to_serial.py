#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class JoyToSerialNode(Node):
    def __init__(self):
        super().__init__('joy_to_serial_node')
        
        self.last_cmd = 'S'
        self.serial_port = None
        
        # 1. Kết nối cổng Serial với Arduino
        try:
            ARDUINO_PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55137313931351015011-if00'
            self.serial_port = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
            self.get_logger().info("Đã mở cổng Serial Arduino thành công!")
        except serial.SerialException as e:
            self.get_logger().error(f"Không thể kết nối Serial với Arduino: {e}")
        
        # 2. CHỈ ĐĂNG KÝ 1 TOPIC DUY NHẤT: /joy
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 3. Watchdog timer để phanh an toàn
        self.last_twist_time = self.get_clock().now()
        # self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)
        
        self.last_buttons = []
        self.last_axes = []

    def joy_callback(self, msg):
        if not self.last_buttons:
            self.last_buttons = msg.buttons
            self.last_axes = msg.axes
            return

        # --- XỬ LÝ NÚT BẤM (BUTTONS) ---
        if len(msg.buttons) > 8:
            if msg.buttons[0] == 1 and self.last_buttons[0] == 0: self.send_serial_char('T') 
            if msg.buttons[1] == 1 and self.last_buttons[1] == 0: self.send_serial_char('S') 
            if msg.buttons[2] == 1 and self.last_buttons[2] == 0: self.send_serial_char('Y') 
            if msg.buttons[3] == 1 and self.last_buttons[3] == 0: self.send_serial_char('X') 
            if msg.buttons[5] == 1 and self.last_buttons[5] == 0: self.send_serial_char('E') 
            if msg.buttons[6] == 1 and self.last_buttons[6] == 0: self.send_serial_char('Q') 
            if msg.buttons[7] == 1 and self.last_buttons[7] == 0: self.send_serial_char('D') 
            if msg.buttons[8] == 1 and self.last_buttons[8] == 0: self.send_serial_char('A') 

        # --- XỬ LÝ CẦN ANALOG (AXES) ĐỂ DI CHUYỂN ---
        cmd = self.last_cmd
        if len(msg.axes) > 2:
            if msg.axes[1] > 0.01: cmd = 'F'
            elif msg.axes[1] < -0.01: cmd = 'B'
            elif msg.axes[2] > 0.01: cmd = 'L'
            elif msg.axes[2] < -0.01: cmd = 'R'
            elif self.last_cmd in ['F', 'B', 'L', 'R']: 
                cmd = 'S' # Thả cần Analog tự động phanh

        if cmd != self.last_cmd:
            self.send_serial_char(cmd)
            self.last_cmd = cmd
            self.last_twist_time = self.get_clock().now() # Cập nhật thời gian điều khiển

        self.last_buttons = msg.buttons
        self.last_axes = msg.axes

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_twist_time).nanoseconds / 1e9
        if elapsed > 0.5 and self.last_cmd != 'S':
            self.send_serial_char('S')
            self.last_cmd = 'S'

    def send_serial_char(self, c):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(c.encode('ascii'))
                self.get_logger().info(f"Đã gửi: {c}")
            except Exception as e:
                pass

    def destroy_node(self):
        self.send_serial_char('S')
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoyToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()