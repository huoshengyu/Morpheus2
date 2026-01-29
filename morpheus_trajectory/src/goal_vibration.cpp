// serial_trajec_sender.cpp

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

class SerialTrajecSender : public rclcpp::Node
{
  bool prev_active_{false};

public:
  SerialTrajecSender()
    : Node("~"),
      send_interval_(0.05)  // 20 Hz
  {
    // --- Serial port setup ---
    this->get_parameter("serial_port", port_name_, "/dev/arduino");
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0)
    {
      RCLCPP_FATAL("Failed to open %s: %s", port_name_.c_str(), strerror(errno));
      rclcpp::shutdown();
      return;
    }
    configurePort();
    rclcpp::Duration(2.0).sleep();  // wait for Arduino reset
    RCLCPP_INFO(this->get_logger(),"Serial port %s opened at 115200 baud",port_name_.c_str());

    // --- Subscribers ---
    dist_sub_  = nh_.subscribe("/vx300s/trajectory/goal/distance", 10,
                               &SerialTrajecSender::distanceCallback, this);
    direc_sub_ = nh_.subscribe("/vx300s/trajectory/goal/vector",   10,
                               &SerialTrajecSender::directionCallback, this);
  }

  ~SerialTrajecSender()
  {
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  rclcpp::NodeHandle        nh_;
  rclcpp::Subscription      dist_sub_, direc_sub_;
  int                    serial_fd_{-1};
  std::string            port_name_;
  termios                tty_;
  geometry_msgs::Vector3 latest_direction_;
  rclcpp::Time              last_send_time_;
  rclcpp::Duration          send_interval_;

  // Configure baud & raw mode
  void configurePort()
  {
    memset(&tty_, 0, sizeof tty_);
    if (tcgetattr(serial_fd_, &tty_) != 0)
    {
      RCLCPP_FATAL("tcgetattr error: %s", strerror(errno));
      return;
    }
    cfsetospeed(&tty_, B115200);
    cfsetispeed(&tty_, B115200);
    tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty_.c_iflag &= ~IGNBRK;                     // disable break processing
    tty_.c_lflag = 0;                            // no signaling chars, no echo
    tty_.c_oflag = 0;                            // no remapping, no delays
    tty_.c_cc[VMIN]  = 0;                        // read doesn't block
    tty_.c_cc[VTIME] = 5;                        // 0.5s read timeout
    tty_.c_cflag |= CLOCAL | CREAD;              // ignore modem controls
    tty_.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CRTSCTS;
    tcsetattr(serial_fd_, TCSANOW, &tty_);
  }

  // Attempts to write all bytes; returns false on unrecoverable error
  bool robustWrite(const char* buf, size_t len)
  {
    size_t written = 0;
    while (written < len)
    {
      ssize_t n = write(serial_fd_, buf + written, len - written);
      if (n < 0)
      {
        if (errno == EINTR) continue;
        RCLCPP_ERROR("Serial write error: %s", strerror(errno));
        return false;
      }
      written += n;
    }
    return true;
  }

  // Build "<ACT:0>" or "<ACT:1>"
  std::string makeActMsg(bool active)
  {
    std::ostringstream oss;
    oss << '<' << "ACT:" << (active ? '1' : '0') << '>';
    return oss.str();
  }

  // Build "<DIR X:0.123 Y:-0.456 Z:0.789>"
  std::string makeDirMsg(const geometry_msgs::Vector3& v)
  {
    std::ostringstream oss;
    oss << '<'
        << "DIR"
        << " X:" << std::fixed << std::setprecision(3) << v.x
        << " Y:" << std::fixed << std::setprecision(3) << v.y
        << " Z:" << std::fixed << std::setprecision(3) << v.z
        << '>';
    return oss.str();
  }

  // Called whenever a new direction vector arrives
  void directionCallback(const geometry_msgs::Vector3::SharedPtr msg)
  {
    latest_direction_ = *msg;
  }

  // Received new distance; send ACT once on edge and DIR at 20 Hz while active
  void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    bool active = (msg->data > 0.02);
    if (active != prev_active_) {
      // only send ACT when the boolean changes
      std::string act_msg = makeActMsg(active);
      robustWrite(act_msg.c_str(), act_msg.size());
      RCLCPP_INFO_STREAM("Sent " << act_msg);
      prev_active_ = active;
    }

    if (active) {
      // send DIR at up to 20 Hz so Arduino can adjust strength/direction
      if ((rclcpp::Time::now() - last_send_time_) >= send_interval_) {
        last_send_time_ = rclcpp::Time::now();
        std::string dir_msg = makeDirMsg(latest_direction_);
        robustWrite(dir_msg.c_str(), dir_msg.size());
        RCLCPP_INFO_STREAM("Sent " << dir_msg);
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTrajecSender>());
  rclcpp::shutdown();
  return 0;
}
