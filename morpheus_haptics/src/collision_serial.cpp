#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ContactInformation.h>

#include <fcntl.h>      // File control (open)
#include <termios.h>    // Terminal control
#include <unistd.h>     // Write, close
#include <errno.h>      // Error number
#include <string.h>     // String operations
#include <iostream>     // IO Stream
#include <sensor_msgs/JointState.h>
#include <map>
#include <cmath>
#include <sys/ioctl.h>  // ✅ Add this line to fix your build
#include <thread>

class SerialCollisionSender
{
public:
    ros::NodeHandle nh;
    ros::Subscriber collision_sub;
    ros::Subscriber joint_state_sub;

    int serial_port;
    struct termios tty;
    std::string last_contact_link = ""; // <--- NEW: Remember last contact
    std::map<std::string, double> current_joint_state;
    
    std::map<std::string, double> sleep_joint_state = {
        {"elbow", 1.55},
        {"forearm_roll", 0.0},
        {"shoulder", -1.85},
        {"waist", 0.0},
        {"wrist_angle", 0.8},
        {"wrist_rotate", 0.0}
    };

    SerialCollisionSender()
    {
            std::string port_name;
            nh.param<std::string>("serial_port", port_name, "/dev/arduino"); // Use ROS param if available
            serial_port = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
        if (serial_port < 0)
        {
            ROS_ERROR("Error %i from open: %s", errno, strerror(errno));
            exit(1);
        }

        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_port, &tty) != 0)
        {
            ROS_ERROR("Error %i from tcgetattr: %s", errno, strerror(errno));
            exit(1);
        }

        // Setup serial parameters
        tty.c_cflag &= ~PARENB; // No parity bit
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;// No flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore control lines

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT, SUSP

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; 

        tty.c_cc[VTIME] = 10;    // Wait for up to 1 second (10 deciseconds)
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            ROS_ERROR("Error %i from tcsetattr: %s", errno, strerror(errno));
            exit(1);
        }
            // === Force reset Arduino via DTR toggle ===
        int flags;
        ioctl(serial_port, TIOCMGET, &flags);     // Get current modem bits
        flags &= ~TIOCM_DTR;                      // Clear DTR
        ioctl(serial_port, TIOCMSET, &flags);
        usleep(100000);                           // Wait 100 ms
        flags |= TIOCM_DTR;                       // Set DTR
        ioctl(serial_port, TIOCMSET, &flags);

        // === Flush garbage and wait for Arduino to boot ===
        tcflush(serial_port, TCIFLUSH);          // Clean buffer
        ros::Duration(2.0).sleep();               // Wait for Arduino to be ready

        ROS_INFO("Serial port configured and Arduino reset successfully!");

        ROS_INFO("Serial port configured successfully!");

        // Subscribe to nearest collision contact
        collision_sub = nh.subscribe("/vx300s/collision/yaw/contact", 10, &SerialCollisionSender::collisionCallback, this);
        joint_state_sub = nh.subscribe("/vx300s/joint_states", 10, &SerialCollisionSender::jointStateCallback, this);
    }

    ~SerialCollisionSender()
    {
        close(serial_port);
    }

    // === Joint State Callback ===
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_state.clear();
        for (size_t i = 0; i < msg->name.size(); ++i)
            current_joint_state[msg->name[i]] = msg->position[i];
    }
    // === Check if robot is in Sleep posture ===
    bool isInSleepState()
    {
        if (current_joint_state.empty()) {
            return false;
        }

        double tolerance = 0.08;

        for (const auto& it : sleep_joint_state) {
            const std::string& joint = it.first;
            double target_val = it.second;

            auto jt = current_joint_state.find(joint);
            if (jt != current_joint_state.end()) {
                double current_val = jt->second;
                double diff = std::fabs(current_val - target_val);
                if (diff > tolerance) {
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }

    std::string make_serial_message(char mode, float y, float z)
    {
        std::ostringstream oss;
        oss << "<M:" << mode
            << " Y:" << std::showpos << std::fixed << std::setprecision(3) << y
            << " Z:" << std::showpos << std::fixed << std::setprecision(3) << z
            << ">";
        return oss.str();
    }

    void collisionCallback(const moveit_msgs::ContactInformation& msg)
    {
        static const std::set<std::string> ignored_links = {
            "hera_obstacle_course_easy_pick_goal",
            "hera_obstacle_course_hard_pick_goal",
            "hera_obstacle_course_place_goal",
            "hera_table"
        };
    if (ignored_links.count(msg.contact_body_1) ||
        ignored_links.count(msg.contact_body_2))
    {
        // ROS_INFO_STREAM("Ignoring collision with goal link "
        //                  << msg.contact_body_1 << " / "
        //                  << msg.contact_body_2);
        return;                         // ← nothing is sent to the Arduino
    }

        if (isInSleepState()) {
            // ROS_INFO_STREAM("Robot is in Sleep state. Skipping haptic feedback.");
            return;
        }
        std::string link1 = msg.contact_body_1;
        std::string link2 = msg.contact_body_2;

        static const std::set<std::string> end_effector_links = {
            "vx300s/gripper_bar_link", "vx300s/gripper_link",
            "vx300s/gripper_prop_link", "vx300s/right_finger_link", "vx300s/left_finger_link"
        };

        static const std::set<std::string> lower_arm_links = {
            "vx300s/wrist_link", "vx300s/lower_forearm_link"
        };
        char send_char = '0';  // default

        // while (link1 != last_contact_link && link2 != last_contact_link)
        // {
            last_contact_link = link1 + "_" + link2;
            if (end_effector_links.count(link1) || end_effector_links.count(link2)) {
                ROS_INFO_STREAM("Collision with end effector. Sending '1'");
                ROS_INFO_STREAM("which link could be collided" 
                                    << link1 << "/"
                                    << link2);

                send_char = '1';
                // write(serial_port, &send_char, 1);
                // write(serial_port, full_message.c_str(), full_message.length());
            }
            else if (lower_arm_links.count(link1) || lower_arm_links.count(link2) || link1 == "vx300s/upper_forearm_link" || link2 == "vx300s/upper_forearm_link") {
                ROS_INFO_STREAM("Collision with lower arm. Sending '2'");
                send_char = '2';
                ROS_INFO_STREAM("which link could be collided" 
                    << link1 << "/"
                    << link2);

                // write(serial_port, &send_char, 1);
                // write(serial_port, full_message.c_str(), full_message.length());
            }
            // else if (link1 == "vx300s/upper_forearm_link" || link2 == "vx300s/upper_forearm_link") {
            //     ROS_INFO_STREAM("Collision with upper forearm. Sending '3'");
            //     send_char = '3';
            //     // send_char = '0';
            //     // write(serial_port, &send_char, 1);
            //     // write(serial_port, full_message.c_str(), full_message.length());
            // }
            else if (link1 == "vx300s/upper_arm_link" || link2 == "vx300s/upper_arm_link") {
                ROS_INFO_STREAM("Collision with upper arm. Sending '4'");
                send_char = '3';
                ROS_INFO_STREAM("which link could be collided" 
                    << link1 << "/"
                    << link2);

                // write(serial_port, &send_char,s 1);
                // write(serial_port, full_message.c_str(), full_message.length());
            }   

            float distancex = msg.normal.x * msg.depth; //forward, backward
            float distancey = msg.normal.y * msg.depth; //right, left
            float distancez = msg.normal.z * msg.depth; //up, down
            // float angle = std::atan2(distancez,distancey) * 180.0f / M_PI; 
            // ROS_INFO_STREAM("What is the angle:" << msg.depth);

            if (msg.depth < 0.07f){
                float angle = std::atan2(distancez, distancey) * 180.0f / M_PI;  
                float angle_x= std::atan2(distancex, distancez) * 180.0f / M_PI;  
                // 2) normalize to [0,360)
                float ang = std::fmod(angle + 360.0f, 360.0f);

                // Region 1: right (–22.5°…+22.5° ⇒ 337.5…360 or 0…22.5)
                if (ang < 45.0f || ang >= 315.0f) {
                    distancez = 1.0f;
                    // std::string full_msg = make_serial_message(send_char,distancey,distancez);
                }
                // Region 2: up (+67.5…112.5)
                else if (ang >= 45.0f && ang < 135.0f) {
                    distancey = 1.0f;
                    // std::string full_msg = make_serial_message(send_char, distancey,distancez);
                }
                // Region 3: left (+157.5…202.5)
                else if (ang >= 135.0f && ang < 225.0f) {
                    distancez = 1.0f;
                    // std::string full_msg = make_serial_message(send_char, distancey,distancez);
                }
                // Region 4: down (+247.5…292.5)
                else if (ang >= 225.0f && ang < 315.0f) {
                    distancey = 1.0f;
                    // std::string full_msg = make_serial_message(send_char, distancey,distancez);
                }
                std::string full_msg = make_serial_message(send_char,distancey,distancez);
                ssize_t bytes_written = write(serial_port,full_msg.c_str(),full_msg.length());
                std::this_thread::yield();
                if (bytes_written != (ssize_t)full_msg.length()) {
                    ROS_WARN_STREAM("Serial write mismatch! Expected " << full_msg.length()
                                    << ", wrote " << bytes_written);
                } else {
                    ROS_INFO_STREAM("Sent to Arduino: " << full_msg);
                }
            }

            // std::string full_msg = make_serial_message(send_char, distancey, distancez);      
            // tcflush(serial_port, TCIOFLUSH);
            // ssize_t bytes_written = write(serial_port, full_msg.c_str(), full_msg.length());
            // usleep(5000);
            // ssize_t bytes_written = write(serial_port, full_msg.c_str(),full_msg.length());
            // std::this_thread::yield();
            // ✅ Optional debug check
        
            // if (bytes_written != (ssize_t)full_msg.length()) {
            //     ROS_WARN_STREAM("Serial write mismatch! Expected " << full_msg.length()
            //                     << ", wrote " << bytes_written);
            // } else {
            //     ROS_INFO_STREAM("Sent to Arduino: " << full_msg);
            // }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_collision_sender");
    SerialCollisionSender node;
    ros::spin();
    return 0;
}
