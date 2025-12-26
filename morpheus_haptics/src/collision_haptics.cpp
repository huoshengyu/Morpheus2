#include <ros/ros.h>
#include <moveit_msgs/ContactInformation.h>
#include <sensor_msgs/JointState.h>

#include <fcntl.h>          // open
#include <termios.h>        // termios
#include <unistd.h>         // write, close, usleep
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>      // TIOCMGET, TIOCMSET
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <map>
#include <set>
#include <cmath>

class SerialCollisionSender
{
public:
    SerialCollisionSender()
    {
        // --- Open & configure serial port ---
        std::string port_name;
        nh_.param<std::string>("serial_port", port_name, "/dev/arduino");
        serial_port_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_port_ < 0) {
            ROS_FATAL("Failed to open %s: %s", port_name.c_str(), strerror(errno));
            std::exit(1);
        }

        memset(&tty_, 0, sizeof tty_);
        if (tcgetattr(serial_port_, &tty_) != 0) {
            ROS_FATAL("tcgetattr: %s", strerror(errno));
            std::exit(1);
        }

        // 115200 8N1, no flow control, raw mode
        tty_.c_cflag &= ~PARENB;
        tty_.c_cflag &= ~CSTOPB;
        tty_.c_cflag &= ~CSIZE;
        tty_.c_cflag |= CS8;
        tty_.c_cflag &= ~CRTSCTS;
        tty_.c_cflag |= CREAD | CLOCAL;

        tty_.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
        tty_.c_iflag &= ~(IXON | IXOFF | IXANY
                          | IGNBRK | BRKINT | PARMRK | ISTRIP
                          | INLCR | IGNCR | ICRNL);
        tty_.c_oflag &= ~(OPOST | ONLCR);

        tty_.c_cc[VTIME] = 1;   // 0.1 s read timeout
        tty_.c_cc[VMIN]  = 0;   // return immediately

        cfsetispeed(&tty_, B115200);
        cfsetospeed(&tty_, B115200);

        if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0) {
            ROS_FATAL("tcsetattr: %s", strerror(errno));
            std::exit(1);
        }

        // --- Toggle DTR to reset Arduino ---
        int flags;
        ioctl(serial_port_, TIOCMGET, &flags);
        flags &= ~TIOCM_DTR;
        ioctl(serial_port_, TIOCMSET, &flags);
        usleep(100000);
        flags |= TIOCM_DTR;
        ioctl(serial_port_, TIOCMSET, &flags);

        // flush and wait 2s for boot
        tcflush(serial_port_, TCIFLUSH);
        ros::Duration(2.0).sleep();

        ROS_INFO("Serial port %s up at 115200 bps", port_name.c_str());

        // --- Start writer thread ---
        writer_thread_ = std::thread(&SerialCollisionSender::writerLoop, this);

        // --- Subscribers ---
        collision_sub_ = nh_.subscribe(
            "/vx300s/collision/nearest/contact", 10,
            &SerialCollisionSender::collisionCallback, this);
        joint_state_sub_ = nh_.subscribe(
            "/vx300s/joint_states", 10,
            &SerialCollisionSender::jointStateCallback, this);

        // --- Heartbeat timer: resend last packet at 50 Hz ---
        heartbeat_timer_ = nh_.createTimer(
            ros::Duration(0.02),
            &SerialCollisionSender::onHeartbeat, this);
    }

    ~SerialCollisionSender()
    {
        // signal writer thread to exit
        {
            std::lock_guard<std::mutex> lk(write_mutex_);
            running_ = false;
        }
        write_cv_.notify_one();
        writer_thread_.join();

        close(serial_port_);
    }

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber collision_sub_, joint_state_sub_;
    ros::Timer      heartbeat_timer_;

    // Serial
    int serial_port_;
    struct termios tty_;

    // Threaded writer
    std::thread            writer_thread_;
    std::mutex             write_mutex_;
    std::condition_variable write_cv_;
    std::queue<std::string> write_queue_;
    bool                   running_ = true;

    // Last packet for heartbeat
    std::string            last_packet_;

    // Joint state & sleep posture
    std::map<std::string, double> current_joint_state_;
    const std::map<std::string, double> sleep_joint_state_ = {
        {"elbow", 1.55}, {"forearm_roll", 0.0},
        {"shoulder", -1.85}, {"waist", 0.0},
        {"wrist_angle", 0.8}, {"wrist_rotate", 0.0}
    };
    std::string last_contact_link_;

    // Format an ASCII packet
    std::string make_serial_message(char mode, float y, float z)
    {
        char buf[32];
        int len = snprintf(buf, sizeof(buf),
                           "<M:%c Y:%+.3f Z:%+.3f>",
                           mode, y, z);
        return std::string(buf, len);
    }

    // Joint state callback
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_state_.clear();
        for (size_t i = 0; i < msg->name.size(); ++i)
            current_joint_state_[msg->name[i]] = msg->position[i];
    }

    // Check if robot is in sleep posture
    bool isInSleepState()
    {
        if (current_joint_state_.empty()) return false;
        const double tol = 0.08;
        for (auto const& kv : sleep_joint_state_) {
            auto it = current_joint_state_.find(kv.first);
            if (it == current_joint_state_.end()
             || fabs(it->second - kv.second) > tol) {
                return false;
            }
        }
        return true;
    }

    // Collision callback: build & enqueue packet
    void collisionCallback(const moveit_msgs::ContactInformation& msg)
    {
        if (isInSleepState()) {
            ROS_INFO("Sleeping: skipping feedback");
            return;
        }

        std::string contact = msg.contact_body_1 + "_" + msg.contact_body_2;
        if (contact == last_contact_link_) return;
        last_contact_link_ = contact;

        // Determine mode
        static const std::set<std::string> eff = {
            "vx300s/gripper_bar_link","vx300s/gripper_link",
            "vx300s/gripper_prop_link","vx300s/right_finger_link",
            "vx300s/left_finger_link"
        };
        static const std::set<std::string> lower = {
            "vx300s/wrist_link","vx300s/lower_forearm_link"
        };
        char mode='0';
        if      (eff.count(msg.contact_body_1)||eff.count(msg.contact_body_2)) mode='1';
        else if (lower.count(msg.contact_body_1)||lower.count(msg.contact_body_2)) mode='2';
        else if (msg.contact_body_1=="vx300s/upper_forearm_link"
              || msg.contact_body_2=="vx300s/upper_forearm_link") mode='3';
        else if (msg.contact_body_1=="vx300s/upper_arm_link"
              || msg.contact_body_2=="vx300s/upper_arm_link") mode='4';

        float dy = msg.normal.y * msg.depth;
        float dz = msg.normal.z * msg.depth;

        last_packet_ = make_serial_message(mode, dy, dz);
        {
            std::lock_guard<std::mutex> lk(write_mutex_);
            write_queue_.push(last_packet_);
        }
        write_cv_.notify_one();
    }

    // Heartbeat: resend last_packet_ at fixed rate
    void onHeartbeat(const ros::TimerEvent&)
    {
        if (last_packet_.empty()) return;
        {
            std::lock_guard<std::mutex> lk(write_mutex_);
            write_queue_.push(last_packet_);
        }
        write_cv_.notify_one();
    }

    // Writer thread: drain queue and write
    void writerLoop()
    {
        while (true) {
            std::unique_lock<std::mutex> lk(write_mutex_);
            write_cv_.wait(lk, [&]{ return !write_queue_.empty() || !running_; });
            if (!running_ && write_queue_.empty()) break;

            std::string pkt = std::move(write_queue_.front());
            write_queue_.pop();
            lk.unlock();
            ROS_INFO_STREAM("[ → Arduino ] " << pkt);
            ssize_t w = write(serial_port_, pkt.c_str(), pkt.size());
            if (w != (ssize_t)pkt.size()) {
                ROS_WARN("Serial write mismatch: %zd/%zu", w, pkt.size());
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_collision_sender");
    SerialCollisionSender node;
    ros::spin();
    return 0;
}
