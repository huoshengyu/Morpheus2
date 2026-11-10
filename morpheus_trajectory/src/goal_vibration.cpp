// ---------------------------------------------------------------------------
// Morpheus2: ROS2 control and feedback for manual teleoperation research
// Copyright (C) 2026 Brian Huo
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// ---------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <fcntl.h>      // open()
#include <termios.h>    // serial port control
#include <unistd.h>     // write(), close()
#include <errno.h>      // errno
#include <string.h>     // strerror()
#include <sstream>
#include <iomanip>
#include <sys/ioctl.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class SerialTrajecSender : public rclcpp::Node
{
public:
  SerialTrajecSender()
    : Node("goal_vibration"),
      last_send_time_(this->now()),
      send_interval_(rclcpp::Duration::from_seconds(0.05))  // 20 Hz
  {
    this->declare_parameter("serial_port", std::string("/dev/arduino"));
    this->declare_parameter("mock_mode", false);
    this->declare_parameter("distance_topic", std::string("/vx300s/trajectory/goal/distance"));
    this->declare_parameter("vector_topic", std::string("/vx300s/trajectory/goal/vector"));

    port_name_ = this->get_parameter("serial_port").as_string();
    mock_mode_ = this->get_parameter("mock_mode").as_bool();
    std::string distance_topic = this->get_parameter("distance_topic").as_string();
    std::string vector_topic = this->get_parameter("vector_topic").as_string();

    if (!mock_mode_)
    {
      serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
      if (serial_fd_ < 0)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s: %s. Running in MOCK mode.",
                    port_name_.c_str(), strerror(errno));
        mock_mode_ = true;
      }
      else
      {
        configurePort();
        std::this_thread::sleep_for(2s);  // Wait for Arduino reset
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened at 115200 baud", port_name_.c_str());
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Running goal_vibration in MOCK mode");
    }

    // Subscribers
    dist_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      distance_topic, 10,
      std::bind(&SerialTrajecSender::distanceCallback, this, std::placeholders::_1));

    direc_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      vector_topic, 10,
      std::bind(&SerialTrajecSender::directionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "goal_vibration node initialized");
  }

  ~SerialTrajecSender() override
  {
    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr direc_sub_;
  int serial_fd_{-1};
  bool mock_mode_{false};
  bool prev_active_{false};
  std::string port_name_{"/dev/arduino"};
  geometry_msgs::msg::Vector3 latest_direction_;
  rclcpp::Time last_send_time_;
  rclcpp::Duration send_interval_;

  void configurePort()
  {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr error: %s", strerror(errno));
      return;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  bool robustWrite(const char* buf, size_t len)
  {
    if (mock_mode_ || serial_fd_ < 0)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[MOCK GOAL HAPTICS] %s", buf);
      return true;
    }

    size_t written = 0;
    while (written < len)
    {
      ssize_t n = write(serial_fd_, buf + written, len - written);
      if (n < 0)
      {
        if (errno == EINTR) continue;
        RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", strerror(errno));
        return false;
      }
      written += n;
    }
    return true;
  }

  std::string makeActMsg(bool active)
  {
    std::ostringstream oss;
    oss << '<' << "ACT:" << (active ? '1' : '0') << ">\n";
    return oss.str();
  }

  std::string makeDirMsg(const geometry_msgs::msg::Vector3& v)
  {
    std::ostringstream oss;
    oss << '<'
        << "DIR"
        << " X:" << std::fixed << std::setprecision(3) << v.x
        << " Y:" << std::fixed << std::setprecision(3) << v.y
        << " Z:" << std::fixed << std::setprecision(3) << v.z
        << ">\n";
    return oss.str();
  }

  void directionCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_direction_ = *msg;
  }

  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    bool active = (msg->data > 0.02);
    if (active != prev_active_)
    {
      std::string act_msg = makeActMsg(active);
      robustWrite(act_msg.c_str(), act_msg.size());
      RCLCPP_INFO(this->get_logger(), "Sent %s", act_msg.c_str());
      prev_active_ = active;
    }

    if (active)
    {
      if ((this->now() - last_send_time_) >= send_interval_)
      {
        last_send_time_ = this->now();
        std::string dir_msg = makeDirMsg(latest_direction_);
        robustWrite(dir_msg.c_str(), dir_msg.size());
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialTrajecSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
