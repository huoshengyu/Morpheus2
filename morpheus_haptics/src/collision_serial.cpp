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
#include <moveit_msgs/msg/contact_information.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcntl.h>      // File control (open)
#include <termios.h>    // Terminal control
#include <unistd.h>     // Write, close
#include <errno.h>      // Error number
#include <string.h>     // String operations
#include <iostream>
#include <iomanip>
#include <map>
#include <cmath>
#include <set>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>

using namespace std::chrono_literals;

class SerialCollisionSender : public rclcpp::Node
{
public:
    rclcpp::Subscription<moveit_msgs::msg::ContactInformation>::SharedPtr collision_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

    int serial_port_{-1};
    bool mock_mode_{false};
    std::string port_name_{"/dev/arduino"};
    std::string arm_type_{"ur5e"};
    std::string last_contact_link_{""};
    std::map<std::string, double> current_joint_state_;

    SerialCollisionSender() : Node("serial_collision_sender")
    {
        this->declare_parameter("serial_port", std::string("/dev/arduino"));
        this->declare_parameter("mock_mode", false);
        this->declare_parameter("arm_type", std::string("ur5e"));
        this->declare_parameter("contact_topic", std::string("collision/contact"));
        this->declare_parameter("joint_state_topic", std::string("/joint_states"));

        port_name_ = this->get_parameter("serial_port").as_string();
        mock_mode_ = this->get_parameter("mock_mode").as_bool();
        arm_type_ = this->get_parameter("arm_type").as_string();
        std::string contact_topic = this->get_parameter("contact_topic").as_string();
        std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();

        if (!mock_mode_)
        {
            initSerialPort();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Running SerialCollisionSender in MOCK mode (no hardware required)");
        }

        collision_sub = this->create_subscription<moveit_msgs::msg::ContactInformation>(
            contact_topic, 10,
            std::bind(&SerialCollisionSender::collisionCallback, this, std::placeholders::_1));

        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic, 10,
            std::bind(&SerialCollisionSender::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "SerialCollisionSender initialized for arm_type: %s, subscribing to %s",
                    arm_type_.c_str(), contact_topic.c_str());
    }

    ~SerialCollisionSender() override
    {
        if (serial_port_ >= 0)
        {
            close(serial_port_);
        }
    }

private:
    void initSerialPort()
    {
        serial_port_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (serial_port_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Unable to open serial port %s (%s). Falling back to MOCK mode.",
                        port_name_.c_str(), strerror(errno));
            mock_mode_ = true;
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port_, &tty) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "tcgetattr failed (%s). Falling back to MOCK mode.", strerror(errno));
            close(serial_port_);
            serial_port_ = -1;
            mock_mode_ = true;
            return;
        }

        // Configure serial port for 8N1 @ 115200 baud
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "tcsetattr failed (%s). Falling back to MOCK mode.", strerror(errno));
            close(serial_port_);
            serial_port_ = -1;
            mock_mode_ = true;
            return;
        }

        // DTR toggle for reset
        int flags;
        ioctl(serial_port_, TIOCMGET, &flags);
        flags &= ~TIOCM_DTR;
        ioctl(serial_port_, TIOCMSET, &flags);
        usleep(100000);
        flags |= TIOCM_DTR;
        ioctl(serial_port_, TIOCMSET, &flags);

        tcflush(serial_port_, TCIFLUSH);
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully at 115200 baud", port_name_.c_str());
    }

    std::string makeSerialMessage(char link_id, float y_dist, float z_dist)
    {
        std::ostringstream ss;
        ss << link_id << ","
           << std::fixed << std::setprecision(2) << y_dist << ","
           << std::fixed << std::setprecision(2) << z_dist << "\n";
        return ss.str();
    }

    char classifyLink(const std::string& link1, const std::string& link2)
    {
        auto matches = [&](const std::string& pattern) {
            return (link1.find(pattern) != std::string::npos || link2.find(pattern) != std::string::npos);
        };

        // Link classification for feedback mapping:
        // '1' = End effector / wrist / gripper
        // '2' = Forearm
        // '3' = Upper arm
        // '4' = Shoulder / base
        if (matches("gripper") || matches("tool0") || matches("wrist_3") || matches("ee_gripper"))
        {
            return '1';
        }
        else if (matches("forearm") || matches("wrist_1") || matches("wrist_2"))
        {
            return '2';
        }
        else if (matches("upper_arm") || matches("shoulder"))
        {
            return '3';
        }
        else if (matches("base"))
        {
            return '4';
        }
        return '1';
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state_.clear();
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            current_joint_state_[msg->name[i]] = msg->position[i];
        }
    }

    void collisionCallback(const moveit_msgs::msg::ContactInformation& msg)
    {
        std::string link1 = msg.contact_body_1;
        std::string link2 = msg.contact_body_2;
        char send_char = classifyLink(link1, link2);

        float distancex = msg.normal.x * msg.depth;
        float distancey = msg.normal.y * msg.depth;
        float distancez = msg.normal.z * msg.depth;

        if (msg.depth < 0.07f)
        {
            float angle = std::atan2(distancez, distancey) * 180.0f / M_PI;
            float ang = std::fmod(angle + 360.0f, 360.0f);

            if (ang < 45.0f || ang >= 315.0f)
            {
                distancez = 1.0f;
            }
            else if (ang >= 45.0f && ang < 135.0f)
            {
                distancey = 1.0f;
            }
            else if (ang >= 135.0f && ang < 225.0f)
            {
                distancez = 1.0f;
            }
            else if (ang >= 225.0f && ang < 315.0f)
            {
                distancey = 1.0f;
            }

            std::string full_msg = makeSerialMessage(send_char, distancey, distancez);
            if (!mock_mode_ && serial_port_ >= 0)
            {
                ssize_t bytes_written = write(serial_port_, full_msg.c_str(), full_msg.length());
                if (bytes_written != static_cast<ssize_t>(full_msg.length()))
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Serial write mismatch! Expected %zu, wrote %zd",
                                         full_msg.length(), bytes_written);
                }
            }
            else
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[MOCK HAPTICS] Contact: link1=%s link2=%s -> Msg: %s",
                                     link1.c_str(), link2.c_str(), full_msg.substr(0, full_msg.size() - 1).c_str());
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCollisionSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
