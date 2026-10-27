#ifndef MORPHEUS_SPAWNER__SPAWNER_HPP_
#define MORPHEUS_SPAWNER__SPAWNER_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Shapes
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

// Eigen
#include <tf2_eigen/tf2_eigen.hpp>

// Import other files from module
#include "collision_object.hpp"
#include "attached_collision_object.hpp"
#include "morpheus_msgs/srv/spawner_msg_service.hpp"
#include "morpheus_msgs/srv/spawner_preset_service.hpp"
#include "morpheus_msgs/srv/attacher_msg_service.hpp"
#include "morpheus_msgs/srv/attacher_index_service.hpp"

class SpawnerNode : public rclcpp::Node
{
public:
    explicit SpawnerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SpawnerNode();

    std::shared_ptr<rclcpp::Node> initialize();

    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_publisher_; // Unnecessary, as objects can be spawned via the planning scene interface

    rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attached_collision_object_publisher_;
    std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_collision_object_vector_;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_fake_publisher_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_fake_monitor_;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;

    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr spawner_msg_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spawner_preset_subscription_;
    rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attacher_msg_subscription_;
    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr attacher_index_subscription_;

    rclcpp::Service<morpheus_msgs::srv::SpawnerMsgService>::SharedPtr spawner_msg_service_;
    rclcpp::Service<morpheus_msgs::srv::SpawnerPresetService>::SharedPtr spawner_preset_service_;
    rclcpp::Service<morpheus_msgs::srv::AttacherMsgService>::SharedPtr attacher_msg_service_;
    rclcpp::Service<morpheus_msgs::srv::AttacherIndexService>::SharedPtr attacher_index_service_;

    // Declare variables for identifying robot vs environment
    std::string arm_group_;

    void spawner_msg_subscription_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
    void spawner_preset_subscription_callback(const std_msgs::msg::String::SharedPtr msg);
    void attacher_msg_subscription_callback(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr msg);
    void attacher_index_subscription_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);

    bool spawner_msg_service_callback(std::shared_ptr<morpheus_msgs::srv::SpawnerMsgService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::SpawnerMsgService::Response> res);
    bool spawner_preset_service_callback(std::shared_ptr<morpheus_msgs::srv::SpawnerPresetService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::SpawnerPresetService::Response> res);
    bool attacher_msg_service_callback(std::shared_ptr<morpheus_msgs::srv::AttacherMsgService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::AttacherMsgService::Response> res);
    bool attacher_index_service_callback(std::shared_ptr<morpheus_msgs::srv::AttacherIndexService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::AttacherIndexService::Response> res);

    void spawn(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object);
    void spawn(const int& index = 0);
    void despawn(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object);
    void despawn(const int& index = 0);
    void attach(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr& attached_collision_object, const std::string& link_name = "tool0");
    void attach(const int& index = 0, const std::string& link_name = "tool0");
    void detach(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr& attached_collision_object);
    void detach(const int& index = 0);

    void attach_fake(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr& attached_collision_object, const std::string& link_name = "tool0");
    void attach_fake(const int& index = 0, const std::string& link_name = "tool0");
    void detach_fake(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr& attached_collision_object);
    void detach_fake(const int& index = 0);

    void publish(const moveit_msgs::msg::CollisionObject::SharedPtr& collision_object);
    void publish(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr& attached_collision_object);
    void publish(const moveit_msgs::msg::CollisionObject::SharedPtr& collision_object, const moveit_msgs::msg::AttachedCollisionObject::SharedPtr& attached_collision_object);

    void update();
    void get_params_from_preset(const std::string&, std::string&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Quaterniond&);
    void spawn_from_preset(const std::string&);

    moveit_msgs::msg::CollisionObject create(const std::string& mesh_path, 
          const Eigen::Vector3d& scale,
          const Eigen::Vector3d& position,
          const Eigen::Quaterniond& quaternion);
    void save(const moveit_msgs::msg::CollisionObject::SharedPtr& collision_object);

private:

};

#endif  // MORPHEUS_SPAWNER__SPAWNER_HPP_