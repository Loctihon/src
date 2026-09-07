#include <rclcpp/rclcpp.hpp>
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
#include <stdexcept>

using namespace std::chrono_literals;

// ======================= CẤU HÌNH (có thể đổi thành tham số ROS2 sau) =======================
static const char* SERIAL_PORT = "/dev/ttyACM0";   // Cổng USB của Arduino Mega
static const int   SERIAL_BAUD = B115200;          // Khớp Serial.begin(115200) trong firmware

class ArduinoOdomNode : public rclcpp::Node {
public:
    ArduinoOdomNode() : Node("arduino_odom_node"), x_(0.0), y_(0.0), theta_(0.0), first_read_(true) {

        // 1. CẤU HÌNH SERIAL LINUX CHUẨN
        //    (Mở cổng trong hàm riêng open_serial(). Nếu lỗi -> throw để node
        //     TẮT HẲN thay vì "sống nhưng câm" như trước — dễ thấy lỗi trong log)
        open_serial();

        // 2. KHAI BÁO ROS 2
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 3. THÔNG SỐ VẬT LÝ XE (ÔNG PHẢI ĐO LẠI)
        double PULSES_PER_REV = 330.0; 
        double WHEEL_RADIUS = 0.065; 
        double WHEEL_BASE = 0.25;    
        dist_per_pulse_ = (2.0 * M_PI * WHEEL_RADIUS) / PULSES_PER_REV;
        wheel_base_ = WHEEL_BASE;

        last_time_ = this->get_clock()->now();

        // 4. TIMER CHẠY 100HZ ĐỂ ĐỌC SERIAL CỰC MƯỢT
        timer_ = this->create_wall_timer(10ms, std::bind(&ArduinoOdomNode::read_serial_and_publish, this));
        RCLCPP_INFO(this->get_logger(), "Đã khởi động Node Odom C++ siêu tốc! (cổng %s)", SERIAL_PORT);
    }

    ~ArduinoOdomNode() {
        close_serial();
    }

private:
    // ---------- MỞ CỔNG SERIAL ----------
    void open_serial() {
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            throw std::runtime_error(
                std::string("Không thể mở cổng Serial ") + SERIAL_PORT +
                " — " + std::strerror(errno) +
                ". Kiểm tra: cáp cắm chưa? chạy 'sudo chmod 666 " + SERIAL_PORT +
                "' hoặc có node khác đang giữ cổng?");
        }

        struct termios options;
        if (tcgetattr(serial_fd_, &options) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::runtime_error(std::string("tcgetattr lỗi: ") + std::strerror(errno));
        }

        // Tốc độ
        cfsetispeed(&options, SERIAL_BAUD);
        cfsetospeed(&options, SERIAL_BAUD);

        // c_cflag: 8 bit data, no parity, 1 stop bit, bật nhận
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // c_iflag: TẮT mọi chuyển đổi / điều khiển luồng (raw input)
        //   ICRNL: không đổi \r -> \n (quan trọng vì firmware gửi \r\n)
        //   IXON/IXOFF/IXANY: tắt điều khiển luồng phần mềm
        //   INLCR/IGNCR: không đổi/xóa \r
        //   BRKINT/INPCK/ISTRIP: tắt xử lý lỗi ngắt / kiểm parity / strip bit
        options.c_iflag &= ~(ICRNL | INLCR | IGNCR | IXON | IXOFF | IXANY
                             | BRKINT | INPCK | ISTRIP | PARMRK);

        // c_oflag: tắt xử lý output (raw output)
        options.c_oflag &= ~OPOST;

        // c_lflag: chế độ non-canonical (không chờ Enter), không echo
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);

        // VMIN = 1: read() chỉ trả khi có ít nhất 1 byte.
        // VTIME = 0: không giới hạn thời gian chờ (non-blocking vì có O_NDELAY).
        options.c_cc[VMIN] = 1;
        options.c_cc[VTIME] = 0;

        // Áp dụng cấu hình
        if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            throw std::runtime_error(std::string("tcsetattr lỗi: ") + std::strerror(errno));
        }

        // Xóa sạch bộ đệm cũ (rác từ lúc mở cổng / lần reset trước của Mega)
        tcflush(serial_fd_, TCIOFLUSH);

        serial_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "Đã mở cổng Serial %s thành công (fd=%d)", SERIAL_PORT, serial_fd_);
    }

    void close_serial() {
        if (serial_fd_ != -1) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        serial_ok_ = false;
    }

    // ---------- ĐỌC & XỬ LÝ DỮ LIỆU ----------
    void read_serial_and_publish() {
        if (!serial_ok_ || serial_fd_ == -1) return;

        char buffer[256];
        int n = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (n > 0) {
            buffer[n] = '\0';
            serial_buffer_ += buffer;

            // Xử lý từng dòng khi có ký tự xuống dòng
            size_t pos;
            while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, pos);
                serial_buffer_.erase(0, pos + 1);
                process_line(line);
            }
        } else if (n == 0) {
            // Cổng bị đóng từ xa (VD: Mega reset, rút cáp). 
            // Ở chế độ non-blocking, n==0 = EOF.
            RCLCPP_WARN(this->get_logger(), "Cổng serial đóng (EOF) — thử kết nối lại sau 1s...");
            handle_disconnect();
        } else if (n == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Không có dữ liệu sẵn sàng — bình thường, không phải lỗi
                return;
            }
            // Lỗi thật sự khi đọc
            RCLCPP_WARN(this->get_logger(), "Lỗi đọc serial: %s — thử kết nối lại sau 1s...",
                        std::strerror(errno));
            handle_disconnect();
        }
    }

    // Tự đóng và mở lại cổng (reconnect) sau khi mất kết nối
    void handle_disconnect() {
        close_serial();
        // Thử mở lại định kỳ (mỗi timer tick = 10ms, nhưng thêm cờ để chỉ thử 1 lần/giây)
        if (reconnect_elapsed_ >= reconnect_interval_) {
            reconnect_elapsed_ = 0;
            try {
                open_serial();
                RCLCPP_INFO(this->get_logger(), "Đã kết nối lại cổng serial thành công!");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Kết nối lại thất bại: %s", e.what());
            }
        } else {
            reconnect_elapsed_ += 1;  // mỗi lần gọi ~10ms
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
                    RCLCPP_INFO(this->get_logger(), "Nhận dòng dữ liệu đầu tiên: L=%ld, R=%ld",
                                current_pulse_L, current_pulse_R);
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

                // Toán học Kinematics
                double delta_dist = (dist_R + dist_L) / 2.0;
                double delta_theta = (dist_R - dist_L) / wheel_base_;

                x_ += delta_dist * cos(theta_ + (delta_theta / 2.0));
                y_ += delta_dist * sin(theta_ + (delta_theta / 2.0));
                theta_ += delta_theta;

                double v = delta_dist / dt;
                double omega = delta_theta / dt;

                publish_odom_and_tf(current_time, v, omega);

            } catch (...) {
                // Bỏ qua rác serial
            }
        }
    }

    void publish_odom_and_tf(rclcpp::Time current_time, double v, double omega) {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // 1. Publish Odometry
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = omega;
        odom_pub_->publish(odom);

        // 2. Publish TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    int serial_fd_ = -1;
    bool serial_ok_ = false;
    long reconnect_elapsed_ = 0;
    static constexpr long reconnect_interval_ = 100;  // ~1 giây (100 tick x 10ms)
    std::string serial_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, theta_;
    long prev_pulse_L_, prev_pulse_R_;
    bool first_read_;
    rclcpp::Time last_time_;
    double dist_per_pulse_, wheel_base_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // Bọc try/catch: nếu mở cổng lỗi thì node TẮT HẲN kèm log lỗi rõ ràng,
    // thay vì tạo ra 1 "ma node" chạy âm thầm không publish gì.
    std::shared_ptr<ArduinoOdomNode> node;
    try {
        node = std::make_shared<ArduinoOdomNode>();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_odom_node"),
                     "KHỞI ĐỘNG THẤT BẠI: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
