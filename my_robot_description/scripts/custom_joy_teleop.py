#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CustomJoyTeleop(Node):
    def __init__(self):
        super().__init__('custom_joy_teleop')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Tốc độ giới hạn ban đầu
        self.linear_scale = 0.5
        self.angular_scale = 0.5
        
        # Biến nhớ trạng thái nút bấm (tránh việc giữ nút làm tăng liên tục)
        self.btn_inc_last = 0
        self.btn_dec_last = 0

    def joy_callback(self, msg):
        # Nút tiến (ID 5): Tăng tốc độ 0.5 
        if msg.axes[5] == 1 and self.btn_inc_last == 0:
            self.linear_scale += 0.5
            self.angular_scale += 0.5
            self.get_logger().info(f'Tăng tốc: {self.linear_scale}')
        self.btn_inc_last = msg.axes[5]

        # Nút lùi (ID 5): Giảm tốc độ 0.5 
        if msg.axes[5] == -1 and self.btn_dec_last == 0:
            self.linear_scale = max(0.0, self.linear_scale - 0.5)
            self.angular_scale = max(0.0, self.angular_scale - 0.5)
            self.get_logger().info(f'Giảm tốc: {self.linear_scale}')
        self.btn_dec_last = msg.axes[5]

        # Tính toán vận tốc dựa trên cần Analog trái
        twist = Twist()
        twist.linear.x = msg.axes[1] * self.linear_scale
        twist.angular.z = msg.axes[0] * self.angular_scale
        
        self.publisher_.publish(twist)

def main():
    rclpy.init()
    node = CustomJoyTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()