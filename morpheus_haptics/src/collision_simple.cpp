#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/contact_information.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <map>
#include <cmath>
#include <set>
#include <string>

using namespace std::chrono_literals;

class CollisionFeedbackPublisher : public rclcpp::Node
{
public:
    rclcpp::Subscription<moveit_msgs::msg::ContactInformation>::SharedPtr collision_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr haptic_pub;

    std::map<std::string, double> current_joint_state;

    std::map<std::string, double> sleep_joint_state = {
        {"elbow", 1.55}, {"forearm_roll", 0.0}, {"shoulder", -1.85},
        {"waist", 0.0}, {"wrist_angle", 0.8}, {"wrist_rotate", 0.0}
    };

    CollisionFeedbackPublisher() : Node("collision_feedback_publisher")
    {
        collision_sub = this->create_subscription<moveit_msgs::msg::ContactInformation>(
            "/vx300s/collision/yaw/contact", 10, 
            std::bind(&CollisionFeedbackPublisher::collisionCallback, this, std::placeholders::_1));
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/vx300s/joint_states", 10, 
            std::bind(&CollisionFeedbackPublisher::jointStateCallback, this, std::placeholders::_1));
        haptic_pub = this->create_publisher<geometry_msgs::msg::Vector3>("/vx300s/collision/yaw/yaw_distance", 10);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state.clear();
        for (size_t i = 0; i < msg->name.size(); ++i)
            current_joint_state[msg->name[i]] = msg->position[i];
    }

    bool isInSleepState()
    {
        if (current_joint_state.empty()) return false;
        double tol = 0.08;
        for (const auto& it : sleep_joint_state)
        {
            auto jt = current_joint_state.find(it.first);
            if (jt == current_joint_state.end() || std::abs(jt->second - it.second) > tol)
                return false;
        }
        return true;
    }

    void collisionCallback(const moveit_msgs::msg::ContactInformation& msg)
    {
        if (isInSleepState()) {
            RCLCPP_INFO(this->get_logger(),"In sleep pose, skipping haptic feedback.");
            return;
        }

        std::string link1 = msg.contact_body_1;
        std::string link2 = msg.contact_body_2;

        std::string mode = "0";
        static const std::set<std::string> end_effector_links = {
            "vx300s/gripper_bar_link", "vx300s/gripper_link",
            "vx300s/gripper_prop_link", "vx300s/right_finger_link", "vx300s/left_finger_link"
        };
        static const std::set<std::string> lower_arm_links = {
            "vx300s/wrist_link", "vx300s/lower_forearm_link"
        };

        if (end_effector_links.count(link1) || end_effector_links.count(link2))
            mode = "1";
        else if (lower_arm_links.count(link1) || lower_arm_links.count(link2))
            mode = "2";
        else if (link1 == "vx300s/upper_forearm_link" || link2 == "vx300s/upper_forearm_link")
            mode = "3";
        else if (link1 == "vx300s/upper_arm_link" || link2 == "vx300s/upper_arm_link")
            mode = "4";

        float distance_y = std::round(msg.normal.y * msg.depth * 1000.0f) / 1000.0f;
        float distance_z = std::round(msg.normal.z * msg.depth * 1000.0f) / 1000.0f;
        geometry_msgs::msg::Vector3 feedback;
        feedback.x = std::stof(mode);  // Encode mode in x
        feedback.y = distance_y;
        feedback.z = distance_z;
        rclcpp::sleep_for(100ms);  // Delay for 0.1 seconds (100 ms)
        haptic_pub->publish(feedback);

        RCLCPP_INFO_STREAM(this->get_logger(),
                        "Published HapticCommand: M:" << mode
                        << " Y:" << distance_y
                        << " Z:" << distance_z);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionFeedbackPublisher>());
    rclcpp::shutdown();
    return 0;
}
