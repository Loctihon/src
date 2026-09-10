#!/usr/bin/env python3
"""
Nhận lệnh vận tốc từ teleop_twist_keyboard (topic /cmd_vel) và gửi ký tự
xuống Arduino qua Serial.

Thay thế cho bản dùng /joy: giờ chỉ cần chạy:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 run my_robot_description cmd_vel_to_serial.py

Giao thức Serial gửi xuống Arduino (1 ký tự):
    F = tiến, B = lùi, L = quay trái, R = quay phải, S = dừng
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')

        # Topic lệnh vào. Mặc định 'cmd_vel' (tên tương đối, giống hệt cách
        # teleop_twist_keyboard publish) -> ở namespace gốc sẽ là /cmd_vel.
        # Nhờ vậy chỉ cần:
        #   ros2 run teleop_twist_keyboard teleop_twist_keyboard
        #   ros2 run my_robot_description cmd_vel_to_serial.py
        # KHÔNG cần --ros-args -r gì cả.
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Ngưỡng vận tốc để tránh nhiễu/dao động nhỏ
        self.linear_threshold = 0.05    # m/s
        self.angular_threshold = 0.05   # rad/s

        self.last_cmd = 'S'
        self.serial_port = None

        # 1. Kết nối cổng Serial với Arduino
        try:
            self.serial_port = serial.Serial(
                '/dev/ttyACM0', 115200, timeout=0.1, write_timeout=0.1)
            self.get_logger().info("Đã mở cổng Serial Arduino thành công!")
        except serial.SerialException as e:
            self.get_logger().error(
                f"Không thể kết nối Serial với Arduino: {e}")

        # 2. Đăng ký topic cmd_vel (do teleop_twist_keyboard phát)
        self.sub_cmd_vel = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        # 3. Vét bộ đệm Serial (chỗ này sau này làm odometry)
        self.read_timer = self.create_timer(0.02, self.read_serial_data)

        # 4. Watchdog để phanh an toàn khi mất lệnh
        self.last_twist_time = self.get_clock().now()
        self.watchdog_timeout = 0.5     # giây
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

    def read_serial_data(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            try:
                # Vét sạch dữ liệu đang chờ để chống treo Arduino
                while self.serial_port.in_waiting > 0:
                    raw_data = self.serial_port.readline().decode(
                        'ascii', errors='ignore').strip()

                    if raw_data:
                        # CHỖ NÀY LÀ NƠI SAU NÀY BẠN LÀM ODOMETRY
                        # Ví dụ:
                        # ticks = raw_data.split(',')
                        # if len(ticks) == 2:
                        #     left_tick = int(ticks[0])
                        #     right_tick = int(ticks[1])
                        #     self.get_logger().info(
                        #         f"Odom: Left={left_tick}, Right={right_tick}")
                        pass
            except Exception:
                pass

    def cmd_vel_callback(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z

        # Ưu tiên: nếu có lệnh tiến/lùi thì đi, ngược lại mới quay
        if lin > self.linear_threshold:
            cmd = 'F'
        elif lin < -self.linear_threshold:
            cmd = 'B'
        elif ang > self.angular_threshold:
            cmd = 'L'
        elif ang < -self.angular_threshold:
            cmd = 'R'
        else:
            cmd = 'S'

        if cmd != self.last_cmd:
            self.send_serial_char(cmd)
            self.last_cmd = cmd

        # Cập nhật thời gian nhận lệnh cho watchdog
        self.last_twist_time = self.get_clock().now()

    def watchdog_check(self):
        """Nếu quá lâu không nhận được lệnh thì tự phanh."""
        elapsed = (self.get_clock().now() -
                   self.last_twist_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout and self.last_cmd != 'S':
            self.send_serial_char('S')
            self.last_cmd = 'S'

    def send_serial_char(self, c):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(c.encode('ascii'))
                self.get_logger().info(f"Đã gửi: {c}")
            except Exception:
                pass

    def destroy_node(self):
        self.send_serial_char('S')
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
