#include <ros/ros.h>
#include <moveit_msgs/ContactInformation.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <cmath>
#include <set>
#include <string>

class CollisionFeedbackPublisher
{
public:
    ros::NodeHandle nh;
    ros::Subscriber collision_sub;
    ros::Subscriber joint_state_sub;
    ros::Publisher haptic_pub;

    std::map<std::string, double> current_joint_state;

    std::map<std::string, double> sleep_joint_state = {
        {"elbow", 1.55}, {"forearm_roll", 0.0}, {"shoulder", -1.85},
        {"waist", 0.0}, {"wrist_angle", 0.8}, {"wrist_rotate", 0.0}
    };

    CollisionFeedbackPublisher()
    {
        collision_sub = nh.subscribe("/vx300s/collision/yaw/contact", 10, &CollisionFeedbackPublisher::collisionCallback, this);
        joint_state_sub = nh.subscribe("/vx300s/joint_states", 10, &CollisionFeedbackPublisher::jointStateCallback, this);
        haptic_pub = nh.advertise<geometry_msgs::Vector3>("/vx300s/collision/yaw/yaw_distance", 10);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
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

    void collisionCallback(const moveit_msgs::ContactInformation& msg)
    {
        if (isInSleepState()) {
            ROS_INFO("In sleep pose, skipping haptic feedback.");
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
        geometry_msgs::Vector3 feedback;
        feedback.x = std::stof(mode);  // Encode mode in x
        feedback.y = distance_y;
        feedback.z = distance_z;
        ros::Duration(0.1).sleep();  // Delay for 0.1 seconds (100 ms)
        haptic_pub.publish(feedback);

        ROS_INFO_STREAM("Published HapticCommand: M:" << mode
                        << " Y:" << distance_y
                        << " Z:" << distance_z);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_feedback_publisher");
    CollisionFeedbackPublisher node;
    ros::spin();
    return 0;
}
