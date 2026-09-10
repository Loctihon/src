#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

static const char* SERIAL_PORT = "/dev/ttyACM0"; // Cổng USB Arduino Mega
static const int   SERIAL_BAUD = B115200;

class ArduinoBridgeNode : public rclcpp::Node {
public:
    ArduinoBridgeNode() : Node("arduino_bridge_node"), last_cmd_('S'), x_(0.0), y_(0.0), theta_(0.0), first_read_(true) {

        open_serial();

        // 1. Publisher Odom & TF
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 2. Subscriber Joy để lái xe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ArduinoBridgeNode::joy_callback, this, std::placeholders::_1));

        // 3. Thông số vật lý xe
        double PULSES_PER_REV = 330.0; 
        double WHEEL_RADIUS = 0.065; 
        double WHEEL_BASE = 0.25;    
        dist_per_pulse_ = (2.0 * M_PI * WHEEL_RADIUS) / PULSES_PER_REV;
        wheel_base_ = WHEEL_BASE;
        last_time_ = this->get_clock()->now();

        // 4. Timer đọc Odom 100Hz liên tục
        timer_ = this->create_wall_timer(10ms, std::bind(&ArduinoBridgeNode::read_serial_and_publish, this));
        RCLCPP_INFO(this->get_logger(), "Đã khởi động Arduino Bridge Node (Full-Duplex R/W)!");
    }

    ~ArduinoBridgeNode() {
        send_serial_char('S');
        close_serial();
    }

private:
    // --- GỬI LỆNH ĐIỀU KHIỂN TỪ TAY CẦM XUỐNG ARDUINO ---
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Xử lý nút bấm (Buttons)
        if (msg->buttons.size() > 8) {
            if (!last_buttons_.empty()) {
                if (msg->buttons[0] == 1 && last_buttons_[0] == 0) send_serial_char('T');
                if (msg->buttons[1] == 1 && last_buttons_[1] == 0) send_serial_char('S');
                if (msg->buttons[2] == 1 && last_buttons_[2] == 0) send_serial_char('Y');
                if (msg->buttons[3] == 1 && last_buttons_[3] == 0) send_serial_char('X');
                if (msg->buttons[5] == 1 && last_buttons_[5] == 0) send_serial_char('E');
                if (msg->buttons[6] == 1 && last_buttons_[6] == 0) send_serial_char('Q');
                if (msg->buttons[7] == 1 && last_buttons_[7] == 0) send_serial_char('D');
                if (msg->buttons[8] == 1 && last_buttons_[8] == 0) send_serial_char('A');
            }
        }

        // Xử lý cần gạt (Axes) — GIỐNG HỆT cmd_vel_to_serial.py (đã chạy OK):
        // ưu tiên cố định F > B > L > R, ngưỡng 0.1; chỉ gửi S khi đang chạy rồi thả cần.
        char cmd = last_cmd_;
        if (msg->axes.size() > 2) {
            if (msg->axes[1] > 0.1) {
                cmd = 'F';
            } else if (msg->axes[1] < -0.1) {
                cmd = 'B';
            } else if (msg->axes[2] > 0.1) {
                cmd = 'L';
            } else if (msg->axes[2] < -0.1) {
                cmd = 'R';
            } else if (last_cmd_ == 'F' || last_cmd_ == 'B' || last_cmd_ == 'L' || last_cmd_ == 'R') {
                cmd = 'S'; // Thả cần analog -> tự động phanh
            }
        }

        if (cmd != last_cmd_) {
            send_serial_char(cmd);
            last_cmd_ = cmd;
        }

        last_buttons_ = msg->buttons;
    }

    void send_serial_char(char c) {
        if (serial_fd_ != -1 && serial_ok_) {
            write(serial_fd_, &c, 1);
            RCLCPP_INFO(this->get_logger(), "Đã gửi lệnh xuống Arduino: %c", c);
        }
    }

    // --- CẤU HÌNH & ĐỌC SERIAL ODOM ---
    void open_serial() {
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Không mở được cổng %s: %s", SERIAL_PORT, std::strerror(errno));
            return;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, SERIAL_BAUD);
        cfsetospeed(&options, SERIAL_BAUD);

        // SỬA LỖI THỨ TỰ BIT: vì CS8 nằm trọn trong mask CSIZE (cùng trị 0000060),
        // phải CLEAR CSIZE TRƯỚC rồi mới SET CS8 SAU.
        // (Làm ngược: |= CS8 rồi &= ~CSIZE sẽ xóa luôn CS8 -> rớt về CS5 -> lệnh 'F' bị rác)
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_iflag &= ~(ICRNL | INLCR | IGNCR | IXON | IXOFF | IXANY | BRKINT | INPCK | ISTRIP | PARMRK);
        options.c_oflag &= ~OPOST;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);

        // ÉP CHUẨN NON-BLOCKING: VMIN=0, VTIME=0 -> read() trả 0 khi chưa có data (không phải EOF)
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIOFLUSH);
        serial_ok_ = true;
    }

    void close_serial() {
        if (serial_fd_ != -1) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        serial_ok_ = false;
    }

    void handle_disconnect() {
        if (!serial_ok_ && serial_fd_ == -1) return; // đã mất, không xử lý lặp
        RCLCPP_WARN(this->get_logger(), "Mất kết nối Serial (%s), thử kết nối lại...", std::strerror(errno));
        close_serial();
        std::this_thread::sleep_for(200ms);
        open_serial();
        if (serial_ok_) {
            RCLCPP_INFO(this->get_logger(), "Đã kết nối lại Serial %s", SERIAL_PORT);
            first_read_ = true; // đặt lại mốc xung để tránh nhảy sai quãng đường
        }
    }

    void read_serial_and_publish() {
        if (serial_fd_ == -1) {
            // Cổng chưa mở (hoặc đang mất kết nối): thử mở lại mỗi ~1s
            if (++reconnect_ticks_ >= 100) {
                reconnect_ticks_ = 0;
                open_serial();
            }
            return;
        }
        if (!serial_ok_) return;

        char buffer[256];
        int n = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (n > 0) {
            buffer[n] = '\0';
            serial_buffer_ += buffer;

            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1);
                process_line(line);
            }
        } else if (n == 0) {
            // Trả về 0 chỉ là CHƯA CÓ DỮ LIỆU tới (non-blocking, VMIN=0), KHÔNG phải rớt mạng.
            return;
        } else {
            // n == -1
            if (errno == EAGAIN || errno == EWOULDBLOCK) return; // chưa có dữ liệu, không phải lỗi
            handle_disconnect(); // lỗi thật khi rút cáp (thường EIO)
        }
    }

    void process_line(const std::string& line) {
        std::stringstream ss(line);
        std::string left_str, right_str;

        if (std::getline(ss, left_str, ',') && std::getline(ss, right_str)) {
            try {
                long current_pulse_L = std::stol(left_str);
                long current_pulse_R = std::stol(right_str);

                if (first_read_) {
                    prev_pulse_L_ = current_pulse_L;
                    prev_pulse_R_ = current_pulse_R;
                    first_read_ = false;
                    last_time_ = this->get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "Nhận dòng dữ liệu đầu tiên: L=%ld, R=%ld (Arduino đang stream)", current_pulse_L, current_pulse_R);
                    return;
                }

                long delta_L = current_pulse_L - prev_pulse_L_;
                long delta_R = current_pulse_R - prev_pulse_R_;
                prev_pulse_L_ = current_pulse_L;
                prev_pulse_R_ = current_pulse_R;

                double dist_L = delta_L * dist_per_pulse_;
                double dist_R = delta_R * dist_per_pulse_;

                rclcpp::Time current_time = this->get_clock()->now();
                double dt = (current_time - last_time_).seconds();
                last_time_ = current_time;

                if (dt <= 0) return;

                double delta_dist = (dist_R + dist_L) / 2.0;
                double delta_theta = (dist_R - dist_L) / wheel_base_;

                x_ += delta_dist * cos(theta_ + (delta_theta / 2.0));
                y_ += delta_dist * sin(theta_ + (delta_theta / 2.0));
                theta_ += delta_theta;

                double v = delta_dist / dt;
                double omega = delta_theta / dt;

                publish_odom_and_tf(current_time, v, omega);
            } catch (...) {}
        }
    }

    void publish_odom_and_tf(rclcpp::Time current_time, double v, double omega) {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = omega;
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    int serial_fd_ = -1;
    bool serial_ok_ = false;
    int reconnect_ticks_ = 0;
    std::string serial_buffer_;
    char last_cmd_;
    std::vector<int32_t> last_buttons_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, theta_;
    long prev_pulse_L_, prev_pulse_R_;
    bool first_read_;
    rclcpp::Time last_time_;
    double dist_per_pulse_, wheel_base_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}