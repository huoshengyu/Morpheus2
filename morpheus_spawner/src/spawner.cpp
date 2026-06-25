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

// Name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

class SpawnerNode : public rclcpp::Node
{
  public:
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

    explicit SpawnerNode() : Node("morpheus_spawner")
    {
      // Declare parameters
      this->declare_parameter("arm_group", "arm");

      // Get parameters from ros server, if possible
      arm_group_ = this->get_parameter("arm_group").as_string();

      // Set PlanningSceneInterface in namespace
      planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(std::string(this->get_namespace()));

      // Initialize publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 1);

      // Initialize publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      attached_collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>("attached_collision_object", 1);
    
      // Initialize planning scene diff publisher
      planning_scene_fake_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene_fake", 1);

      // Initialize planning scene diff publisher
      planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

      // Initialize a subscription to receive collision object messages to spawn
      spawner_msg_subscription_ = this->create_subscription<moveit_msgs::msg::CollisionObject>("spawner/spawner_msg_queue", 1, std::bind(&SpawnerNode::spawner_msg_subscription_callback, this, std::placeholders::_1));

      // Initialize a subscription to receive names of object presets to spawn
      spawner_preset_subscription_ = this->create_subscription<std_msgs::msg::String>("spawner/spawner_preset_queue", 1, std::bind(&SpawnerNode::spawner_preset_subscription_callback, this, std::placeholders::_1));

      // Initialize a subscription to receive indices of collision objects to attach/detach
      attacher_msg_subscription_ = this->create_subscription<moveit_msgs::msg::AttachedCollisionObject>("spawner/attacher_msg_queue", 1, std::bind(&SpawnerNode::attacher_msg_subscription_callback, this, std::placeholders::_1));

      // Initialize a subscription to receive indices of collision objects to attach/detach
      attacher_index_subscription_ = this->create_subscription<moveit_msgs::msg::CollisionObject>("spawner/attacher_index_queue", 1, std::bind(&SpawnerNode::attacher_index_subscription_callback, this, std::placeholders::_1));

      // Initialize collision object message spawner service
      spawner_msg_service_ = this->create_service<morpheus_msgs::srv::SpawnerMsgService>("spawner_msg", std::bind(&SpawnerNode::spawner_msg_service_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      // Initialize collision object preset spawner service
      spawner_preset_service_ = this->create_service<morpheus_msgs::srv::SpawnerPresetService>("spawner_preset", std::bind(&SpawnerNode::spawner_preset_service_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      // Initialize collision object message attacher service
      attacher_msg_service_ = this->create_service<morpheus_msgs::srv::AttacherMsgService>("attacher_msg", std::bind(&SpawnerNode::attacher_msg_service_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      // Initialize collision object index attacher service
      attacher_index_service_ = this->create_service<morpheus_msgs::srv::AttacherIndexService>("attacher_index", std::bind(&SpawnerNode::attacher_index_service_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "SpawnerNode ready");
    }

    // Initialize components which rely on shared_from_this() and thus cannot be called in the node's constructor
    std::shared_ptr<rclcpp::Node> initialize()
    {
      // Initialize PlanningSceneMonitor
      planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), ROBOT_DESCRIPTION);
      planning_scene_fake_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), ROBOT_DESCRIPTION);
      
      // Start the PlanningSceneMonitor
      planning_scene_monitor_->startSceneMonitor("move_group/monitored_planning_scene"); // Get scene updates from topic
      planning_scene_monitor_->requestPlanningSceneState();

      // Initialize a move group interface so objects can be attached
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), arm_group_);
      return shared_from_this();
    }

    moveit_msgs::msg::CollisionObject create(const std::string& mesh_path, 
          const Eigen::Vector3d& scale = {1, 1, 1},
          const Eigen::Vector3d& position = {0, 0, 0},
          const Eigen::Quaterniond& quaternion = {1, 0, 0, 0})
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Creating object");
      std::stringstream position_ss;
      position_ss << "Position:";
      for (double val : position)
      {
        position_ss << " ";
        position_ss << std::to_string(val);
      }
      RCLCPP_INFO_STREAM(this->get_logger(), position_ss.str());
      std::stringstream quaternion_ss;
      quaternion_ss << "Quaternion:";
      for (double val : quaternion.coeffs())
      {
        quaternion_ss << " ";
        quaternion_ss << std::to_string(val);
      }
      RCLCPP_INFO_STREAM(this->get_logger(), quaternion_ss.str());

      // Create collision object
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
      collision_object.header.stamp = this->now();
      collision_object.id = "collision_object_" + std::to_string(attached_collision_object_vector_.size()); // Unique id for each object, based on number of objects already in vector
      
      shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, scale);
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(mesh, shape_msg);
      shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
      collision_object.meshes.push_back(mesh_msg);

      collision_object.pose.position = tf2::toMsg(position);
      collision_object.pose.orientation = tf2::toMsg(quaternion);

      collision_object.operation = collision_object.ADD;

      RCLCPP_INFO_STREAM(this->get_logger(), "Creating object complete");

      return collision_object;
    }

    void save(const moveit_msgs::msg::CollisionObject& collision_object)
    {
      // Create attached collision object to be saved
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
      attached_collision_object.object.header = collision_object.header;
      attached_collision_object.object = collision_object;
      attached_collision_object.link_name = "tool0";
      attached_collision_object.object.operation = collision_object.ADD;

      // Save both versions of object to vector (attached object contains collision object)
      attached_collision_object_vector_.push_back(attached_collision_object);
    }

    void spawn(const moveit_msgs::msg::CollisionObject& collision_object)
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Spawning object");

      // Publish planning scene diff
      planning_scene_interface_->addCollisionObjects({collision_object});

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Spawning object complete");
    }

    void spawn(const int& index = 0)
    {
      // Select collision object to spawn
      moveit_msgs::msg::CollisionObject collision_object = attached_collision_object_vector_[index].object;

      // Call version of spawn() that uses specific collision object
      spawn(collision_object);
    }

    void despawn(const moveit_msgs::msg::CollisionObject& collision_object)
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Despawning object");

      // Publish planning scene diff
      planning_scene_interface_->removeCollisionObjects({collision_object.id});
      //publish(collision_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Despawning object complete");
    }

    void despawn(const int& index = 0)
    {
      // Select collision object to despawn
      moveit_msgs::msg::CollisionObject collision_object = attached_collision_object_vector_[index].object;

      // Call version of despawn() that uses specific collision object
      despawn(collision_object);
    }

    void attach(const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object, const std::string& link_name = "tool0")
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object");
      
      // Initialize new message to avoid altering the original
      moveit_msgs::msg::AttachedCollisionObject attach_object(attached_collision_object);
      attach_object.object.header.stamp = this->now();
      attach_object.link_name = link_name;
      attach_object.object.operation = attached_collision_object.object.ADD;
      
      // Publish planning scene diff
      publish(attach_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object complete");
    }

    void attach(const int& index = 0, const std::string& link_name = "tool0")
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = attached_collision_object_vector_[index];
      
      // Call version of attach() that uses specific collision object
      attach(attached_collision_object, link_name);
    }

    void attach_fake(const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object, const std::string& link_name = "tool0")
    {
      // Stopgap solution for attached collision object geometry being ignored by collision engine
      // Marks collision object for continuous position updating
      // Publishes objects so that collision node can track whether they are in robot or not

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object");

      // Find relative transformation from link name to collision object (requires up to date planning scene state)
      moveit_msgs::msg::CollisionObject object;
      planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCollisionObjectMsg(object, attached_collision_object.object.id); // Get object from scene
      Eigen::Affine3d object_transform;
      tf2::fromMsg(object.pose, object_transform);
      Eigen::Affine3d link_transform = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState().getGlobalLinkTransform(link_name); // Get link pose
      Eigen::Affine3d relative_transform = link_transform.inverse() * object_transform; // Concatenate inverse to get relative transform

      // Convert eigen pose back to geometry_msgs::msg::Pose
      geometry_msgs::msg::Pose relative_pose;
      relative_pose = tf2::toMsg(relative_transform);

      // Modify object to be in frame of attach link
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object_fake(attached_collision_object);
      attached_collision_object_fake.object.header.stamp = this->now();
      attached_collision_object_fake.link_name = link_name;
      attached_collision_object_fake.object.header.frame_id = link_name;
      attached_collision_object_fake.object.pose = relative_pose;
      attached_collision_object_fake.object.operation = attached_collision_object_fake.object.ADD;

      // Update planning scene
      planning_scene_monitor::LockedPlanningSceneRW(planning_scene_fake_monitor_)->processAttachedCollisionObjectMsg(attached_collision_object_fake);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object complete");
    }

    void attach_fake(const int& index = 0, const std::string& link_name = "tool0")
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = attached_collision_object_vector_[index];
      
      // Call version of attach() that uses specific collision object
      attach_fake(attached_collision_object, link_name);
    }

    void detach(const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object)
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object");
      
      // Initialize new message to avoid altering the original
      moveit_msgs::msg::AttachedCollisionObject detach_object(attached_collision_object);
      detach_object.object.header.stamp = this->now();
      detach_object.object.operation = attached_collision_object.object.REMOVE;

      // Publish planning scene diff
      publish(detach_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object complete");
    }

    void detach(const int& index = 0)
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = attached_collision_object_vector_[index];
      
      // Call version of detach() that uses specific collision object
      detach(attached_collision_object);
    }

    void detach_fake(const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object)
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object");
      
      // Initialize new message to avoid altering the original
      moveit_msgs::msg::AttachedCollisionObject detach_object(attached_collision_object);
      detach_object.object.header.stamp = this->now();
      detach_object.object.operation = attached_collision_object.object.REMOVE;

      // Update planning scene
      planning_scene_monitor::LockedPlanningSceneRW(planning_scene_fake_monitor_)->processAttachedCollisionObjectMsg(detach_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object complete");
    }

    void detach_fake(const int& index = 0)
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = attached_collision_object_vector_[index];
      
      // Call version of detach() that uses specific collision object
      detach_fake(attached_collision_object);
    }

    // Publish an incoming collision object message without edits
    void publish(const moveit_msgs::msg::CollisionObject& collision_object)
    {
      // Publish planning scene diff
      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;
      //planning_scene_diff_publisher_->publish(planning_scene);
      planning_scene_interface_->applyPlanningScene(planning_scene);

      // collision_object_publisher_->publish(collision_object);

      // Process message
      // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      // locked_planning_scene->usePlanningSceneMsg(planning_scene);
    }

    void publish(const moveit_msgs::msg::CollisionObject& collision_object, const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object)
    {
      // Publish planning scene diff
      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;
      // planning_scene_diff_publisher_->publish(planning_scene);
      planning_scene_interface_->applyPlanningScene(planning_scene);

      // collision_object_publisher_->publish(collision_object);
      // attached_collision_object_publisher_->publish(attached_collision_object);

      // Process message
      // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      // locked_planning_scene->usePlanningSceneMsg(planning_scene);
      
    }

    void publish(const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object)
    { 
      // Publish planning scene diff
      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;
      // planning_scene_diff_publisher_->publish(planning_scene);
      planning_scene_interface_->applyPlanningScene(planning_scene);

      // attached_collision_object_publisher_->publish(attached_collision_object);

      // Process message
      // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      // locked_planning_scene->usePlanningSceneMsg(planning_scene);
    }

    void update()
    {
      // Construct PlanningScene message from PlanningScene
      moveit_msgs::msg::PlanningScene planning_scene_fake_msg;
      planning_scene_monitor::LockedPlanningSceneRO(planning_scene_fake_monitor_)->getPlanningSceneMsg(planning_scene_fake_msg);

      // Repeatedly update positions of fake attached objects
      for (moveit_msgs::msg::AttachedCollisionObject attached_object : planning_scene_fake_msg.robot_state.attached_collision_objects)
      {
        // Respawn object to reset its position
        std::vector<moveit_msgs::msg::CollisionObject> collision_object_vector;
        collision_object_vector.push_back(attached_object.object);
        planning_scene_interface_->addCollisionObjects(collision_object_vector);
      }

      // Send planning scene message with list of fake attached objects
      planning_scene_fake_publisher_->publish(planning_scene_fake_msg);
    }

    // Spin node to continue handling services (should usually be called from main())
    void spin()
    {
      // Loop collision requests and publish at specified rate
      rclcpp::Rate loop_rate(15);
      while (rclcpp::ok())
      {
        std::exception_ptr eptr; // record exceptions in case handling is needed
        try
        {
          update();
        }
        catch (...) 
        {
          eptr = std::current_exception(); // capture
        }
        loop_rate.sleep();
      }

      // Block node from closing before ros shutdown
      // rclcpp::waitForShutdown(); // Equivalent to while(rclcpp::ok()), unnecessary
    }

    // Define function to retrieve a set of rosparams from under a preset name
    void get_params_from_preset(const std::string& preset_name,
                            std::string& mesh_path,
                            Eigen::Vector3d& scale,
                            Eigen::Vector3d& position,
                            Eigen::Quaterniond& quaternion)
    {
      // Namespace containing collision object preset params
      std::string prefix = "/collision_object_presets/";
      // Get full path of preset
      std::string preset_path = prefix + preset_name;

      // Get name of the param which holds the mesh path
      std::string mesh_id = preset_path + "/mesh_path";
      // Retrieve the mesh path from param
      RCLCPP_INFO(this->get_logger(), "Requesting mesh param: %s", mesh_id.c_str());
      mesh_path = this->get_parameter(mesh_id).as_string();
      RCLCPP_INFO(this->get_logger(), "Mesh path: %s", mesh_path.c_str());
      // Specify that the path is relative to the morpheus_description package
      mesh_path = "file://" + ament_index_cpp::get_package_share_directory("morpheus_description") + mesh_path;

      // Get the name of the param which holds the scale
      std::string scale_id = preset_path + "/scale";
      //Retrieve the scale from param
      RCLCPP_INFO(this->get_logger(), "Requesting scale param: %s", scale_id.c_str());
      std::vector<double> scale_param;
      scale = Eigen::Vector3d(this->get_parameter(scale_id).as_double_array().data());
      RCLCPP_INFO(this->get_logger(), "Scale: [%f, %f, %f]", scale[0], scale[1], scale[2]);

      // Get the names of the params holding the object's coordinates
      std::string position_id = preset_path + "/position";
      std::string quaternion_id = preset_path + "/quaternion";
      // Retrieve the coordinates from param
      RCLCPP_INFO(this->get_logger(), "Requesting position param: %s", position_id.c_str());
      position = Eigen::Vector3d(this->get_parameter(position_id).as_double_array().data());
      RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", position[0], position[1], position[2]);
      RCLCPP_INFO(this->get_logger(), "Requesting quaternion param: %s", quaternion_id.c_str());
      quaternion = Eigen::Map<const Eigen::Quaternion<double>, 0>(this->get_parameter(quaternion_id).as_double_array().data());
      RCLCPP_INFO(this->get_logger(), "Quaternion: [%f, %f, %f, %f]", quaternion.coeffs()[0], quaternion.coeffs()[1], quaternion.coeffs()[2], quaternion.coeffs()[3]);
    }

    void spawn_from_preset(const std::string& preset_name)
    {
      // Initialize variables to hold the retrieved params
      std::string mesh_path = "";
      Eigen::Vector3d scale, position;
      Eigen::Quaterniond quaternion;
      get_params_from_preset(preset_name, mesh_path, scale, position, quaternion);

      // Create the collision object
      moveit_msgs::msg::CollisionObject collision_object = create(mesh_path, scale, position, quaternion);

      // Save the object so the node has a persistent object to recall by index
      save(collision_object);
      
      // Spawn the created object
      spawn(attached_collision_object_vector_.size() - 1);
    }

    // Define the callback for spawning whole collision object messages
    void spawner_msg_subscription_callback(moveit_msgs::msg::CollisionObject msg)
    {
      // Save the incoming collision object to attached_collision_object_vector_
      save(msg);

      // Spawn the received object
      spawn(attached_collision_object_vector_.size() - 1);
    }

    // Define the callback for spawning preset objects by name
    void spawner_preset_subscription_callback(std_msgs::msg::String msg)
    {
      spawn_from_preset(msg.data);
    }

    // Define the callback for processing whole attached collision object messages
    void attacher_msg_subscription_callback(moveit_msgs::msg::AttachedCollisionObject msg)
    {
      // Just publish the message
      publish(msg);
    }

    // Define the callback for attaching/detaching objects by index
    void attacher_index_subscription_callback(moveit_msgs::msg::CollisionObject msg)
    {
      // Use a collision object message only for its id (as index) and operation
      // If req->operation == collision_object.ADD, then attach
      if (msg.operation == 0)
      {
        attach_fake(std::stoi(msg.id));
      }
      // If req->operation == collision_object.REMOVE, then detach
      else if (msg.operation == 1)
      {
        detach_fake(std::stoi(msg.id));
      }
    }

    // Define the service call function
    bool spawner_msg_service_callback(std::shared_ptr<morpheus_msgs::srv::SpawnerMsgService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::SpawnerMsgService::Response> res)
    {
      // Save the incoming collision object to attached_collision_object_vector_
      save(req->data);

      // Spawn the created object
      spawn(attached_collision_object_vector_.size() - 1);

      // Return the processed attached collision object
      res->data = attached_collision_object_vector_.back();
      return true;
    }

    // Define the service call function
    bool spawner_preset_service_callback(std::shared_ptr<morpheus_msgs::srv::SpawnerPresetService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::SpawnerPresetService::Response> res)
    {
      spawn_from_preset(req->preset_name);

      // Return the processed attached collision object
      res->data = attached_collision_object_vector_.back();
      return true;
    }

    // Define the service call function
    bool attacher_msg_service_callback(std::shared_ptr<morpheus_msgs::srv::AttacherMsgService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::AttacherMsgService::Response> res)
    {
      // Just publish the message
      publish(req->data);
      publish(res->data);
      return static_cast<bool>(res);
    }

    // Define the service call function
    bool attacher_index_service_callback(std::shared_ptr<morpheus_msgs::srv::AttacherIndexService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::AttacherIndexService::Response> res)
    {
      // If req->operation == collision_object.ADD, then attach
      if (req->operation == 0)
      {
        attach_fake(req->index);
      }
      // If req->operation == collision_object.REMOVE, then detach
      else if (req->operation == 1)
      {
        detach_fake(req->index);
      }
      return static_cast<bool>(res);
    }

  private:
    
};

Eigen::Quaterniond eulerToQuaternion(Eigen::Vector3d& euler)
{ 
  // Convert euler angles to Eigen::Quaterniond
  Eigen::Quaterniond quaternion;
  quaternion = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
  return quaternion;
}

int main(int argc, char** argv)
{
  // Start ROS node
  rclcpp::init(argc, argv);
  auto spawner_node = std::make_shared<SpawnerNode>();
  spawner_node->initialize();
  //rclcpp::spin(spawner_node);
  //rclcpp::shutdown();
  //return 0;
  
  // Parse arguments
  std::vector<std::string> arguments(argv, argv + argc);
  std::string mesh_path;
  Eigen::Vector3d position;
  // std::fill(position.begin(), position.end(), 0);
  Eigen::Vector3d euler;
  // std::fill(euler.begin(), euler.end(), 0);
  std::string mode = "none";
  for (std::size_t i = 0; i < arguments.size(); i++) {
    std::string s = arguments[i];
    //RCLCPP_INFO_STREAM(this->get_logger(), s);
    if (s == "-mesh_path") {
      mesh_path = arguments[i+1];
    }
    if (s == "-x") {
      position[0] = std::stod(arguments[i+1]);
    }
    if (s == "-y") {
      position[1] = std::stod(arguments[i+1]);
    }
    if (s == "-z") {
      position[2] = std::stod(arguments[i+1]);
    }
    if (s == "-R") {
      euler[0] = std::stod(arguments[i+1]);
    }
    if (s == "-P") {
      euler[1] = std::stod(arguments[i+1]);
    }
    if (s == "-Y") {
      euler[2] = std::stod(arguments[i+1]);
    }
    if (s == "-mode") {
      mode = arguments[i+1];
    }
  }

  Eigen::Quaterniond quaternion = eulerToQuaternion(euler);

  Eigen::Vector3d scale = {0.0254, 0.0254, 0.0254};

  if (mode == "spawn") {
    moveit_msgs::msg::CollisionObject collision_object = spawner_node->create(mesh_path, scale=scale, position=position, quaternion=quaternion);
    spawner_node->spawn(spawner_node->attached_collision_object_vector_.size() - 1);
  }
  if (mode == "despawn") {
    spawner_node->despawn(spawner_node->attached_collision_object_vector_.size() - 1);
  }
  if (mode == "attach") {
    spawner_node->attach(spawner_node->attached_collision_object_vector_.size() - 1);
  }
  if (mode == "detach") {
    spawner_node->detach(spawner_node->attached_collision_object_vector_.size() - 1);
  }

  moveit_msgs::msg::CollisionObject sphere_object;
  sphere_object.header.frame_id = spawner_node->move_group_interface_->getPlanningFrame();
  sphere_object.id = "test_sphere";
  shape_msgs::msg::SolidPrimitive sphere_primitive;
  sphere_primitive.type = sphere_primitive.SPHERE;
  sphere_primitive.dimensions.resize(1);
  sphere_primitive.dimensions[0] = 0.2;
  sphere_object.primitives.resize(1);
  sphere_object.primitives[0] = sphere_primitive;
  sphere_object.pose.position.x = 0.0;
  sphere_object.pose.position.y = 0.85;
  sphere_object.pose.position.z = 0.95;
  sphere_object.pose.orientation.w = 1.0;
  sphere_object.operation = sphere_object.ADD;
  //spawner_node->save(sphere_object);
  //spawner_node->spawn(sphere_object);

  moveit_msgs::msg::CollisionObject mesh_object;
  mesh_object.header.frame_id = spawner_node->move_group_interface_->getPlanningFrame();
  mesh_object.id = "test_mesh";
  std::string test_mesh_path = "file:///root/ros2_ws/src/morpheus_description/meshes/collision/teapot.obj";
  const Eigen::Vector3d scale_eigen(0.05, 0.05, 0.05); // mm/inch
  shapes::Mesh* m = shapes::createMeshFromResource(test_mesh_path, scale_eigen);
  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  mesh_object.meshes.resize(2);
  mesh_object.meshes[0] = mesh_msg;
  mesh_object.meshes[1] = mesh_msg;

  //mesh_object.primitives.resize(1);
  //mesh_object.primitives[0] = sphere_primitive;

  mesh_object.pose.position.x = 0.3;
  mesh_object.pose.position.y = 0.35;
  mesh_object.pose.position.z = 0.95;
  mesh_object.pose.orientation.w = 1.0;
  
  mesh_object.mesh_poses.resize(2);
  mesh_object.mesh_poses[0].position.z = 0.2;
  mesh_object.mesh_poses[0].orientation.w = 1;
  mesh_object.mesh_poses[1].position.z = -0.2;
  mesh_object.mesh_poses[1].orientation.w = 1;

  //mesh_object.primitive_poses.resize(1);
  //mesh_object.primitive_poses[0].position.z = -0.2;
  //mesh_object.primitive_poses[0].orientation.w = 1;

  mesh_object.operation = mesh_object.ADD;
  RCLCPP_INFO_STREAM(spawner_node->get_logger(), "Test spawning object");
  spawner_node->save(mesh_object);
  spawner_node->spawn(0);
  //spawner_node->attach(0);

  //RCLCPP_INFO_STREAM(spawner_node->get_logger(), "Spawner Node spinning");

  spawner_node->spin();

  return 0;
}
