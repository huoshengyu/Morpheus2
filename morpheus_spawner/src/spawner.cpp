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
#include "collision_object.h"
#include "attached_collision_object.h"
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
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_move_group_interface;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> g_planning_scene_interface;

    rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr g_collision_object_publisher; // Unnecessary, as objects can be spawned via the planning scene interface

    rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr g_attached_collision_object_publisher;
    std::vector<moveit_msgs::msg::AttachedCollisionObject> g_attached_collision_object_vector;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr g_planning_scene_fake_publisher;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_fake_monitor;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr g_planning_scene_diff_publisher;

    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr g_spawner_msg_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_spawner_preset_subscription;
    rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr g_attacher_msg_subscription;
    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr g_attacher_index_subscription;

    rclcpp::Service<morpheus_msgs::srv::SpawnerMsgService>::SharedPtr g_spawner_msg_service;
    rclcpp::Service<morpheus_msgs::srv::SpawnerPresetService>::SharedPtr g_spawner_preset_service;
    rclcpp::Service<morpheus_msgs::srv::AttacherMsgService>::SharedPtr g_attacher_msg_service;
    rclcpp::Service<morpheus_msgs::srv::AttacherIndexService>::SharedPtr g_attacher_index_service;

    SpawnerNode() : Node("morpheus_spawner")
    {
      // Joints to plan for, from srdf file
      std::string g_planning_group;

      // Get arm and gripper groups from ros server, if possible
      if (this->get_parameter("~arm_group", g_planning_group))
      {
          RCLCPP_INFO(this->get_logger(), "Using arm_group from parameter server");
      }
      else
      {
        g_planning_group = "arm";
          RCLCPP_INFO(this->get_logger(), "Using planning group 'arm'");
      }

      // Set PlanningSceneInterface in namespace
      g_planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(std::string(this->get_namespace()));
            
      // Instantiate PlanningSceneMonitor
      g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), ROBOT_DESCRIPTION);
      g_planning_scene_fake_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), ROBOT_DESCRIPTION);
      
      // Start the PlanningSceneMonitor
      g_planning_scene_monitor->startSceneMonitor("move_group/monitored_planning_scene"); // Get scene updates from topic
      // g_planning_scene_monitor->startSceneMonitor("/planning_scene");
      // g_planning_scene_monitor->startWorldGeometryMonitor("/collision_object", "/planning_scene_world");
      // g_planning_scene_monitor->startStateMonitor("/joint_states", "/attached_collision_object");
      g_planning_scene_monitor->requestPlanningSceneState();

      // Instantiate a move group interface so objects can be attached
      g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), g_planning_group);

      // Planning scene interface does not need further instantiating

      // Instantiate publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      g_collision_object_publisher = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 1);

      // Instantiate publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      g_attached_collision_object_publisher = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>("attached_collision_object", 1);
    
      // Instantiate planning scene diff publisher
      g_planning_scene_fake_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene_fake", 1);

      // Instantiate planning scene diff publisher
      g_planning_scene_diff_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

      // Instantiate a subscription to receive collision object messages to spawn
      g_spawner_msg_subscription = this->create_subscription<moveit_msgs::msg::CollisionObject>("spawner/spawner_msg_queue", 1, std::bind(&SpawnerNode::spawner_msg_subscription_callback, this, std::placeholders::_1));

      // Instantiate a subscription to receive names of object presets to spawn
      g_spawner_preset_subscription = this->create_subscription<std_msgs::msg::String>("spawner/spawner_preset_queue", 1, std::bind(&SpawnerNode::spawner_preset_subscription_callback, this, std::placeholders::_1));

      // Instantiate a subscription to receive indices of collision objects to attach/detach
      g_attacher_msg_subscription = this->create_subscription<moveit_msgs::msg::AttachedCollisionObject>("spawner/attacher_msg_queue", 1, std::bind(&SpawnerNode::attacher_msg_subscription_callback, this, std::placeholders::_1));

      // Instantiate a subscription to receive indices of collision objects to attach/detach
      g_attacher_index_subscription = this->create_subscription<moveit_msgs::msg::CollisionObject>("spawner/attacher_index_queue", 1, std::bind(&SpawnerNode::attacher_index_subscription_callback, this, std::placeholders::_1));

      // Instantiate collision object message spawner service
      g_spawner_msg_service = this->create_service<morpheus_msgs::srv::SpawnerMsgService>("spawner_msg", std::bind(&SpawnerNode::spawner_msg_service_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      // Instantiate collision object preset spawner service
      g_spawner_preset_service = this->create_service<morpheus_msgs::srv::SpawnerPresetService>("spawner_preset", std::bind(&SpawnerNode::spawner_preset_service_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      // Instantiate collision object message attacher service
      g_attacher_msg_service = this->create_service<morpheus_msgs::srv::AttacherMsgService>("attacher_msg", std::bind(&SpawnerNode::attacher_msg_service_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      // Instantiate collision object index attacher service
      g_attacher_index_service = this->create_service<morpheus_msgs::srv::AttacherIndexService>("attacher_index", std::bind(&SpawnerNode::attacher_index_service_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "SpawnerNode ready");
    }

    moveit_msgs::msg::CollisionObject create(std::string mesh_path, 
          std::vector<double> scale = {1, 1, 1},
          std::vector<double> position = {0, 0, 0},
          std::vector<double> quaternion = {0, 0, 0, 1})
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
      for (double val : quaternion)
      {
        quaternion_ss << " ";
        quaternion_ss << std::to_string(val);
      }
      RCLCPP_INFO_STREAM(this->get_logger(), quaternion_ss.str());

      // Create collision object
      moveit_msgs::msg::CollisionObject collision_object =
        create(mesh_path,
              scale,
              position,
              quaternion);

      RCLCPP_INFO_STREAM(this->get_logger(), "Creating object complete");

      return collision_object;
    }

    void save(moveit_msgs::msg::CollisionObject collision_object)
    {
      // Create attached collision object to be saved
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
      attached_collision_object.object.header = collision_object.header;
      attached_collision_object.object = collision_object;
      attached_collision_object.link_name = "camera_link";
      attached_collision_object.object.operation = collision_object.ADD;

      // Save both versions of object to vector (attached object contains collision object)
      g_attached_collision_object_vector.push_back(attached_collision_object);
    }

    void spawn(moveit_msgs::msg::CollisionObject collision_object)
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Spawning object");

      // Create vector of objects to be added
      std::vector<moveit_msgs::msg::CollisionObject> collision_object_vector;
      collision_object_vector.push_back(collision_object);

      // Publish planning scene diff
      g_planning_scene_interface->addCollisionObjects(collision_object_vector);
      //publish(collision_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Spawning object complete");
    }

    void spawn(int index = 0)
    {
      // Select collision object to spawn
      moveit_msgs::msg::CollisionObject collision_object = g_attached_collision_object_vector[index].object;

      // Call version of spawn() that uses specific collision object
      spawn(collision_object);
    }

    void despawn(moveit_msgs::msg::CollisionObject collision_object)
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Despawning object");

      // Create vector of ids to be removed
      std::vector<std::string> object_ids;
      object_ids.push_back(collision_object.id);

      // Publish planning scene diff
      g_planning_scene_interface->removeCollisionObjects(object_ids);
      //publish(collision_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Despawning object complete");
    }

    void despawn(int index = 0)
    {
      // Select collision object to despawn
      moveit_msgs::msg::CollisionObject collision_object = g_attached_collision_object_vector[index].object;

      // Call version of despawn() that uses specific collision object
      despawn(collision_object);
    }

    void attach(moveit_msgs::msg::AttachedCollisionObject attached_collision_object, std::string link_name = "camera_link")
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object");
      
      // Instantiate message strictly for attaching object, to avoid resetting whole object
      moveit_msgs::msg::AttachedCollisionObject attach_object;
      attach_object.object.header = attached_collision_object.object.header;
      attach_object.object.id = attached_collision_object.object.id;
      attach_object.link_name = link_name;
      attach_object.object.operation = attached_collision_object.object.ADD;
      
      // Publish planning scene diff
      publish(attach_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object complete");
    }

    void attach(int index = 0, std::string link_name = "camera_link")
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = g_attached_collision_object_vector[index];
      
      // Call version of attach() that uses specific collision object
      attach(attached_collision_object, link_name);
    }

    void attach_fake(moveit_msgs::msg::AttachedCollisionObject attached_collision_object, std::string link_name = "camera_link")
    {
      // Stopgap soltution for attached collision object geometry being ignored by collision engine
      // Marks collision object for continuous position updating
      // Publishes objects so that collision node can track whether they are in robot or not

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object");

      // Find relative transformation from link name to collision object (requires up to date planning scene state)
      moveit_msgs::msg::CollisionObject object;
      planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCollisionObjectMsg(object, attached_collision_object.object.id); // Get object from scene
      Eigen::Affine3d object_transform;
      tf2::fromMsg(object.pose, object_transform);
      Eigen::Affine3d link_transform = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState().getGlobalLinkTransform(link_name); // Get link pose
      Eigen::Affine3d relative_transform = link_transform.inverse() * object_transform; // Concatenate inverse to get relative transform

      // Convert eigen pose back to geometry_msgs::msg::Pose
      geometry_msgs::msg::Pose relative_pose;
      relative_pose = tf2::toMsg(relative_transform);

      // Modify object to be in frame of attach link
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object_fake;
      attached_collision_object_fake.link_name = link_name;
      attached_collision_object_fake.object = attached_collision_object.object;
      attached_collision_object_fake.object.header.frame_id = link_name;
      attached_collision_object_fake.object.pose = relative_pose;

      // Save attached collision object for future updating of object position
      attached_collision_object_fake.object.operation = attached_collision_object_fake.object.ADD;
      planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_fake_monitor)->processAttachedCollisionObjectMsg(attached_collision_object_fake);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object complete");
    }

    void attach_fake(int index = 0, std::string link_name = "camera_link")
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = g_attached_collision_object_vector[index];
      
      // Call version of attach() that uses specific collision object
      attach_fake(attached_collision_object, link_name);
    }

    void detach(moveit_msgs::msg::AttachedCollisionObject attached_collision_object, std::string link_name = "camera_link")
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object");
      
      // Instantiate message strictly for detaching object, to avoid resetting whole object
      moveit_msgs::msg::AttachedCollisionObject detach_object;
      detach_object.object.id = attached_collision_object.object.id;
      detach_object.link_name = link_name;
      detach_object.object.operation = attached_collision_object.object.REMOVE;
      
      // Publish planning scene diff
      publish(detach_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object complete");
    }

    void detach(int index = 0, std::string link_name = "camera_link")
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = g_attached_collision_object_vector[index];
      
      // Call version of detach() that uses specific collision object
      detach(attached_collision_object);
    }

    void detach_fake(moveit_msgs::msg::AttachedCollisionObject attached_collision_object, std::string link_name = "camera_link")
    {
      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object");
      
      // Instantiate message strictly for detaching object, to avoid resetting whole object
      moveit_msgs::msg::AttachedCollisionObject detach_object;
      detach_object.object.id = attached_collision_object.object.id;
      detach_object.link_name = link_name;
      detach_object.object.operation = attached_collision_object.object.REMOVE;

      // Save attached collision object for future updating of object position
      planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_fake_monitor)->processAttachedCollisionObjectMsg(detach_object);

      // Debug printouts
      RCLCPP_INFO_STREAM(this->get_logger(), "Detaching object complete");
    }

    void detach_fake(int index = 0, std::string link_name = "camera_link")
    {
      // Select attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_collision_object = g_attached_collision_object_vector[index];
      
      // Call version of detach() that uses specific collision object
      detach_fake(attached_collision_object, link_name);
    }

    // Publish an incoming collision object message without edits
    void publish(moveit_msgs::msg::CollisionObject collision_object)
    {
      // Publish planning scene diff
      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;
      //g_planning_scene_diff_publisher->publish(planning_scene);
      g_planning_scene_interface->applyPlanningScene(planning_scene);

      // g_collision_object_publisher->publish(collision_object);

      // Process message
      // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(g_planning_scene_monitor);
      // locked_planning_scene->usePlanningSceneMsg(planning_scene);
    }

    void publish(moveit_msgs::msg::CollisionObject collision_object, moveit_msgs::msg::AttachedCollisionObject attached_collision_object)
    {
      // Publish planning scene diff
      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;
      // g_planning_scene_diff_publisher->publish(planning_scene);
      g_planning_scene_interface->applyPlanningScene(planning_scene);

      // g_collision_object_publisher->publish(collision_object);
      // g_attached_collision_object_publisher->publish(attached_collision_object);

      // Process message
      // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(g_planning_scene_monitor);
      // locked_planning_scene->usePlanningSceneMsg(planning_scene);
      
    }

    void publish(moveit_msgs::msg::AttachedCollisionObject attached_collision_object)
    { 
      // Publish planning scene diff
      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;
      // g_planning_scene_diff_publisher->publish(planning_scene);
      g_planning_scene_interface->applyPlanningScene(planning_scene);

      // g_attached_collision_object_publisher->publish(attached_collision_object);

      // Process message
      // planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(g_planning_scene_monitor);
      // locked_planning_scene->usePlanningSceneMsg(planning_scene);
    }

    void update()
    {
      // Construct PlanningScene message from PlanningScene
      moveit_msgs::msg::PlanningScene planning_scene_fake_msg;
      planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_fake_monitor)->getPlanningSceneMsg(planning_scene_fake_msg);

      // Repeatedly update positions of fake attached objects
      for (moveit_msgs::msg::AttachedCollisionObject attached_object : planning_scene_fake_msg.robot_state.attached_collision_objects)
      {
        // Respawn object to reset its position
        std::vector<moveit_msgs::msg::CollisionObject> collision_object_vector;
        collision_object_vector.push_back(attached_object.object);
        g_planning_scene_interface->addCollisionObjects(collision_object_vector);
      }

      // Send planning scene message with list of fake attached objects
      g_planning_scene_fake_publisher->publish(planning_scene_fake_msg);
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
    void get_preset_params(std::string preset_name,
                            std::string &mesh_path,
                            std::vector<double> &scale_vec,
                            std::vector<double> &pos_vec,
                            std::vector<double> &quat_vec)
    {
      // Namespace containing collision object preset params
      std::string prefix = "/collision_object_presets/";
      // Get full path of preset
      std::string preset_path = prefix + preset_name;

      // Get name of the param which holds the mesh path
      std::string mesh_id = preset_path + "/mesh_path";
      // Retrieve the mesh path from param
      this->get_parameter(mesh_id, mesh_path);
      // Specify that the path is relative to the morpheus_description package
      RCLCPP_INFO_STREAM(this->get_logger(), "Requesting mesh param: " + mesh_id);
      mesh_path = "file://" + ament_index_cpp::get_package_share_directory("morpheus_description") + mesh_path;
      RCLCPP_INFO_STREAM(this->get_logger(), "Mesh path: " + mesh_path);

      // Get the name of the param which holds the scale
      std::string scale_id = preset_path + "/scale";
      //Retrieve the scale from param
      RCLCPP_INFO_STREAM(this->get_logger(), "Requesting scale param: " + scale_id);
      this->get_parameter(scale_id, scale_vec);
      RCLCPP_INFO_STREAM(this->get_logger(), "Scale map: {{x, " + std::to_string(scale_vec[0]) + "}, {y, " + std::to_string(scale_vec[1]) + "}, {z, " + std::to_string(scale_vec[2]) + "}}");

      // Get the names of the params holding the object's coordinates
      std::string pos_id = preset_path + "/position";
      std::string quat_id = preset_path + "/quaternion";
      // Retrieve the coordinates from param
      RCLCPP_INFO_STREAM(this->get_logger(), "Requesting position param: " + pos_id);
      this->get_parameter(pos_id, pos_vec);
      RCLCPP_INFO_STREAM(this->get_logger(), "Position map: {{x, " + std::to_string(pos_vec[0]) + "}, {y, " + std::to_string(pos_vec[1]) + "}, {z, " + std::to_string(pos_vec[2]) + "}}");
      RCLCPP_INFO_STREAM(this->get_logger(), "Requesting quaternion param: " + quat_id);
      this->get_parameter(quat_id, quat_vec);
      RCLCPP_INFO_STREAM(this->get_logger(), "Quaternion map: {{x, " + std::to_string(quat_vec[0]) + "}, {y, " + std::to_string(quat_vec[1]) + "}, {z, " + std::to_string(quat_vec[2]) + "}, {w, " + std::to_string(quat_vec[3]) + "}}");
    }

    void spawn_from_preset(std::string preset_name)
    {
      // Instantiate a string to hold the mesh path
      std::string mesh_path = "";
      // Instantiate a map to hold the scale
      std::vector<double> scale_vec;
      // Instantiate maps to hold the coordinates
      std::vector<double> pos_vec, quat_vec;

      get_preset_params(preset_name, mesh_path, scale_vec, pos_vec, quat_vec);

      // Create the collision object (and save it to g_attached_collision_object_vector)
      moveit_msgs::msg::CollisionObject collision_object = create(mesh_path, scale_vec, pos_vec, quat_vec);

      // Save the object so the node has a persistent object to recall by index
      save(collision_object);
      
      // Spawn the created object
      spawn(g_attached_collision_object_vector.size() - 1);
    }

    // Define the callback for spawning whole collision object messages
    void spawner_msg_subscription_callback(moveit_msgs::msg::CollisionObject msg)
    {
      // Save the incoming collision object to g_attached_collision_object_vector
      save(msg);

      // Spawn the received object
      spawn(g_attached_collision_object_vector.size() - 1);
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
      // Save the incoming collision object to g_attached_collision_object_vector
      save(req->data);

      // Spawn the created object
      spawn(g_attached_collision_object_vector.size() - 1);

      // Return the processed attached collision object
      res->data = g_attached_collision_object_vector.back();
      return true;
    }

    // Define the service call function
    bool spawner_preset_service_callback(std::shared_ptr<morpheus_msgs::srv::SpawnerPresetService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::SpawnerPresetService::Response> res)
    {
      spawn_from_preset(req->preset_name);

      // Return the processed attached collision object
      res->data = g_attached_collision_object_vector.back();
      return true;
    }

    // Define the service call function
    bool attacher_msg_service_callback(std::shared_ptr<morpheus_msgs::srv::AttacherMsgService::Request> req,
                                  std::shared_ptr<morpheus_msgs::srv::AttacherMsgService::Response> res)
    {
      // Just publish the message
      publish(req->data);
      return true;
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
      return true;
    }

  private:
    
};

std::vector<double> eulerToQuaternion(std::vector<double> euler)
{ 
  // Convert euler angles to quaternion
  Eigen::Quaternionf quaternion_eigen;
  quaternion_eigen = Eigen::AngleAxisf(euler[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(euler[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(euler[2], Eigen::Vector3f::UnitZ());
  std::vector<double> quaternion;
  quaternion.resize(4);
  for (int i = 0; i < quaternion_eigen.coeffs().size(); i++)
  {
    double val = quaternion_eigen.coeffs()[i];
    quaternion[i] = val;
  }
  return quaternion;
}

int main(int argc, char** argv)
{
  // Start ROS node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpawnerNode>());
  rclcpp::shutdown();
  return 0;
  
  // Parse arguments
  std::vector<std::string> arguments(argv, argv + argc);
  std::string mesh_path;
  std::vector<double> position;
  position.resize(3);
  // std::fill(position.begin(), position.end(), 0);
  std::vector<double> euler;
  euler.resize(3);
  // std::fill(euler.begin(), euler.end(), 0);
  std::string mode = "none";
  int index = 0;
  for (int i = 0; i < arguments.size(); i++) {
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
    if (s == "-index") {
      index = std::stoi(arguments[i+1]);
    }
  }

  std::vector<double> quaternion = eulerToQuaternion(euler);

  SpawnerNode spawner_node = SpawnerNode();

  std::vector<double> scale = {0.0254, 0.0254, 0.0254};

  if (mode == "spawn") {
    moveit_msgs::msg::CollisionObject collision_object = spawner_node.create(mesh_path, scale=scale, position=position, quaternion=quaternion);
    spawner_node.spawn(spawner_node.g_attached_collision_object_vector.size() - 1);
  }
  if (mode == "despawn") {
    spawner_node.despawn(spawner_node.g_attached_collision_object_vector.size() - 1);
  }
  if (mode == "attach") {
    spawner_node.attach(spawner_node.g_attached_collision_object_vector.size() - 1);
  }
  if (mode == "detach") {
    spawner_node.detach(spawner_node.g_attached_collision_object_vector.size() - 1);
  }

  moveit_msgs::msg::CollisionObject sphere_object;
  sphere_object.header.frame_id = spawner_node.g_move_group_interface->getPlanningFrame();
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
  //spawner_node.save(sphere_object);
  //spawner_node.spawn(sphere_object);

  moveit_msgs::msg::CollisionObject mesh_object;
  mesh_object.header.frame_id = spawner_node.g_move_group_interface->getPlanningFrame();
  mesh_object.id = "test_mesh";
  std::string test_mesh_path = "file:///root/catkin_ws/src/morpheus_description/meshes/components/collision/teapot.stl";
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
  //RCLCPP_INFO_STREAM(this->get_logger(), "Spawning object");
  moveit_msgs::msg::AttachedCollisionObject mesh_attach;
  mesh_attach.object = mesh_object;
  //spawner_node.save(mesh_object);
  //spawner_node.spawn(mesh_object);
  //spawner_node.attach(mesh_attach);

  //RCLCPP_INFO_STREAM(this->get_logger(), "Spawner Node spinning");

  spawner_node.spin();

  return 0;
}
