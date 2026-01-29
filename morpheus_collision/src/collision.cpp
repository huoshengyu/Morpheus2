// General
#include <algorithm>
#include <chrono>

// ROS
#include <rclcpp/rclcpp.hpp>

// Moveit
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/collision_detection_bullet/collision_env_bullet.hpp>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.hpp>
#include <moveit/collision_distance_field/collision_env_hybrid.hpp>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.hpp>
#include <moveit/collision_distance_field/collision_env_distance_field.hpp>
#include <moveit/collision_distance_field/collision_detector_allocator_distance_field.hpp>
#include <moveit/collision_detection/collision_detector_allocator.hpp>
#include <moveit/collision_detection/collision_tools.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Messages
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <morpheus_msgs/msg/contact_map.hpp>

// Shapes
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_state/attached_body.hpp>

// Eigen
#include <tf2_eigen/tf2_eigen.hpp>

// Name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Names of groups in srdf which encompass the robot itself (and not the environment)
static const std::string ARM_GROUP_DEFAULT = "arm";
static const std::string GRIPPER_GROUP_DEFAULT = "gripper";

namespace collision
{

};

class CollisionNode : public rclcpp::Node
{
    public:
        // Declare interfaces for interacting with the planning scene
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

        // Declare publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contactmap_string_publisher_;
        rclcpp::Publisher<morpheus_msgs::msg::ContactMap>::SharedPtr contactmap_msg_publisher_;
        rclcpp::Publisher<moveit_msgs::msg::ContactInformation>::SharedPtr nearest_contact_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nearest_distance_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr nearest_direction_publisher_;
        rclcpp::Publisher<morpheus_msgs::msg::ContactMap>::SharedPtr yaw_contactmap_msg_publisher_;
        rclcpp::Publisher<moveit_msgs::msg::ContactInformation>::SharedPtr yaw_contact_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_distance_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr yaw_direction_publisher_;
        rclcpp::Publisher<morpheus_msgs::msg::ContactMap>::SharedPtr relative_contactmap_msg_publisher_;
        rclcpp::Publisher<moveit_msgs::msg::ContactInformation>::SharedPtr relative_contact_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr relative_distance_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr relative_direction_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr directional_distance_publisher_; //testing before full integration with arduino
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr yaw_directional_distance_publisher_; //testing before full integration with arduino

        // Declare collision info variables
        collision_detection::CollisionResult c_res_;
        collision_detection::CollisionRequest c_req_;
        std::vector<collision_detection::Contact> sorted_contacts_;
        std::vector<collision_detection::Contact> yaw_contacts_;
        std::vector<collision_detection::Contact> relative_contacts_;

        // Declare collision visualization variables
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
        visualization_msgs::msg::MarkerArray collision_points_;
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
        std::vector<collision_detection::Contact>::size_type max_markers_ = 20;

        // Declare variables for identifying robot vs environment
        std::vector<std::string> robot_link_vector_;
        std::string arm_group_;
        std::string gripper_group_;
        
        // Declare interfaces for retrieving robot link models and other info
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_interface_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_interface_;
        
        // Declare variables for tracking attached collision objects
        rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_fake_subscription_;
        std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_collision_object_vector_;

        CollisionNode() : Node("collision_node")
        {
            // Initialize ROS node

            // Create a RobotModelLoader to load the robot's URDF and SRDF
            // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            // robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

            // Create a PlanningScene object and set the robot model
            // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

            // Create a PlanningSceneMonitor around the PlanningScene
            // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
            //     new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

            // Get arm and gripper groups from ros server, if possible
            if (this->get_parameter("~arm_group", arm_group_))
            {
                RCLCPP_INFO(this->get_logger(), "Using arm_group from parameter server");
            }
            else
            {
                arm_group_ = ARM_GROUP_DEFAULT;
                RCLCPP_INFO(this->get_logger(), "Using ARM_GROUP_DEFAULT");
            }
            if (this->get_parameter("~gripper_group", gripper_group_))
            {
                RCLCPP_INFO(this->get_logger(), "Using gripper_group from parameter server");
            }
            else
            {
                gripper_group_ = GRIPPER_GROUP_DEFAULT;
                RCLCPP_INFO(this->get_logger(), "Using GRIPPER_GROUP_DEFAULT");
            }

            // Start move group interfaces for retrieving robot links
            arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), arm_group_);
            gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), gripper_group_);

            // Set PlanningSceneInterface in namespace
            planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(std::string(this->get_namespace()));

            // Instantiate PlanningSceneMonitor
            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), ROBOT_DESCRIPTION);
            
            // Start the PlanningSceneMonitor
            planning_scene_monitor_->startSceneMonitor("move_group/monitored_planning_scene"); // Get scene updates from topic

            // Ensure the PlanningSceneMonitor is ready
            if (planning_scene_monitor_->requestPlanningSceneState("get_planning_scene"))
            {
                RCLCPP_INFO(this->get_logger(), "Planning Scene Monitor is active and ready.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set up Planning Scene Monitor.");
            }

            // Request the PlanningScene itself and change collision detection engine to Bullet
            // planning_scene::PlanningScenePtr planning_scene;
            // Get read/write pointer to planning_scene
            try
            {   
                // Change the PlanningScene's collision detector to Bullet
                // Bullet supports distance vectors, as well as distances to multiple obstacles
                planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
                
                if (strcmp((planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCollisionDetectorName()).c_str(), "Bullet") == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Planning Scene is active and ready.");
                }    
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Collision detector incorrect");
                    RCLCPP_INFO_STREAM(this->get_logger(), planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCollisionDetectorName());
                    std::string collision_detector_name = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCollisionDetectorName();
                    throw collision_detector_name;
                }
            }
            catch (std::string collision_detector_name)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to retrieve PlanningScene.");
            }

            // Prepare collision result and request objects
            c_req_.contacts = true;
            c_req_.distance = true;
            c_req_.max_contacts = 1;
            c_req_.max_contacts_per_pair = 1;
            
            /*
            // Edit the allowed collision matrix to focus only on robot-obstacle collisions
            collision_detection::AllowedCollisionMatrix allowed_collision_matrix = 
                planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->getAllowedCollisionMatrix();
            allowed_collision_matrix.setEntry(true); // Allow all collisions
            allowed_collision_matrix.setEntry("teapot", false); // Register collisions involving teapot
            */

            // Create collision publishers
            contactmap_string_publisher_ = this->create_publisher<std_msgs::msg::String>("collision/contactmap/string", 0);
            contactmap_msg_publisher_ = this->create_publisher<morpheus_msgs::msg::ContactMap>("collision/contactmap/msg", 0);
            nearest_contact_publisher_ = this->create_publisher<moveit_msgs::msg::ContactInformation>("collision/nearest/contact", 0);
            nearest_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("collision/nearest/distance", 0);
            nearest_direction_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("collision/nearest/direction", 0);
            yaw_contactmap_msg_publisher_ = this->create_publisher<morpheus_msgs::msg::ContactMap>("collision/yaw/contactmap/msg", 0);
            yaw_contact_publisher_ = this->create_publisher<moveit_msgs::msg::ContactInformation>("collision/yaw/contact", 0);
            yaw_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("collision/yaw/distance", 0);
            yaw_direction_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("collision/yaw/direction", 0);
            relative_contactmap_msg_publisher_ = this->create_publisher<morpheus_msgs::msg::ContactMap>("collision/relative/contactmap/msg", 0);
            relative_contact_publisher_ = this->create_publisher<moveit_msgs::msg::ContactInformation>("collision/relative/contact", 0);
            relative_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("collision/relative/distance", 0);
            relative_direction_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("collision/relative/direction", 0);
            directional_distance_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("collision/nearest/directional_distance", 0); //testing before full integration with arduino
            yaw_directional_distance_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("collision/yaw/yaw_distance", 0); //testing before full integration with arduino

            
            // Create a marker array publisher for publishing shapes to Rviz
            marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 0);
            
            // Subscribe to fake planning scene for recognizing "fake" attached collision objects
            planning_scene_fake_subscription_ = this->create_subscription<moveit_msgs::msg::PlanningScene>("planning_scene_fake", 1, std::bind(&CollisionNode::planning_scene_fake_callback, this, std::placeholders::_1));

            // Set robot link vector to include all parts of the robot
            // Get all links in the robot arm
            std::vector<std::string> arm_link_vector = arm_interface_->getLinkNames();
            robot_link_vector_.insert(robot_link_vector_.end(), arm_link_vector.begin(), arm_link_vector.end());

            // Get all links in the robot gripper
            std::vector<std::string> gripper_link_vector = gripper_interface_->getLinkNames();
            robot_link_vector_.insert(robot_link_vector_.end(), gripper_link_vector.begin(), gripper_link_vector.end());

            // Instantiate visual tools for visualizing markers in Rviz
            visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(shared_from_this(), "world","moveit_visual_markers"));
        }

        void spin()
        {
            // Create asynchronous spinner to allow callbacks while looping
            //rclcpp::AsyncSpinner spinner(2); // Use 2 threads
            //spinner.start();

            // Loop collision requests and publish at specified rate
            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok())
            {
                update();
                publish();
                visualize(c_res_.contacts);
                // Get all contact vectors which correspond to robot<->obstacle pairs
                //for (int i : contact_map)
                //{

                //}
                loop_rate.sleep();
            }
        }

        void update()
        {
            // Update the planning scene monitor, in case new collision objects have been added
            // planning_scene_monitor_->requestPlanningSceneState();

            // Update the collision result, based on the collision request
            c_res_.clear();
            // Get locked planning scene to ensure scene does not change during update
            planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->checkCollision(c_req_, c_res_);

            // Get sorted contacts so that the top N can be selected
            sorted_contacts_ = get_sorted_contacts(c_res_);
            // Transform contacts based on yaw, which is assumed to be the first joint of the robot
            yaw_contacts_.clear();
            std::string yaw_joint = arm_interface_->getActiveJoints()[0];
            const double* yaw = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState().getJointPositions(yaw_joint);
            Eigen::Affine3d yaw_tf; // Must be first declared, then assigned since there is no implicit constructor from AngleAxis
            yaw_tf = Eigen::AngleAxisd(-*yaw, Eigen::Vector3d(0.0,0.0,1.0));
            for (auto contact : sorted_contacts_)
            {
                yaw_contacts_.push_back(transformContact(contact, yaw_tf));
            }
            // Transform contacts to the frame of the link they are associated with
            relative_contacts_.clear();
            for (auto contact : sorted_contacts_)
            {
                relative_contacts_.push_back(contactToRelativeContact(contact));
            }

            /*
            rclcpp::Time update_time = planning_scene_monitor_->getLastUpdateTime(); // Get last update time
            std::stringstream update_time_ss; // Instantiate stringstream for concatenation
            update_time_ss << update_time.sec << "." << update_time.nsec; // Concatenate seconds.nanoseconds
            std::string update_time_str = update_time_ss.str(); // Convert to std::string
            RCLCPP_INFO(update_time_str.c_str()); // Convert to const char*
            */
        }

        void publish()
        {

            // Publish contact map as text string
            std_msgs::msg::String contacts_msg;
            contacts_msg.data = contactMapToString(c_res_.contacts);
            contactmap_string_publisher_->publish(contacts_msg);

            // Publish contact map (sorted and reduced) as custom msg type
            morpheus_msgs::msg::ContactMap contactmap_msg;
            for (auto const& contact : sorted_contacts_)
            {
                morpheus_msgs::msg::StringPair msg_key;
                msg_key.first = contact.body_name_1;
                msg_key.second = contact.body_name_2;

                moveit_msgs::msg::ContactInformation msg_value;
                collision_detection::contactToMsg(contact, msg_value);

                contactmap_msg.keys.push_back(msg_key);
                contactmap_msg.values.push_back(msg_value);
            }
            contactmap_msg_publisher_->publish(contactmap_msg);

            // Publish link contact map just like regular contact map
            morpheus_msgs::msg::ContactMap yaw_contactmap_msg;
            for (auto const& contact : yaw_contacts_)
            {
                morpheus_msgs::msg::StringPair msg_key;
                msg_key.first = contact.body_name_1;
                msg_key.second = contact.body_name_2;

                moveit_msgs::msg::ContactInformation msg_value;
                collision_detection::contactToMsg(contact, msg_value);

                yaw_contactmap_msg.keys.push_back(msg_key);
                yaw_contactmap_msg.values.push_back(msg_value);
            }
            yaw_contactmap_msg_publisher_->publish(yaw_contactmap_msg);

            // Publish link contact map just like regular contact map
            morpheus_msgs::msg::ContactMap relative_contactmap_msg;
            for (auto const& contact : relative_contacts_)
            {
                morpheus_msgs::msg::StringPair msg_key;
                msg_key.first = contact.body_name_1;
                msg_key.second = contact.body_name_2;

                moveit_msgs::msg::ContactInformation msg_value;
                collision_detection::contactToMsg(contact, msg_value);

                relative_contactmap_msg.keys.push_back(msg_key);
                relative_contactmap_msg.values.push_back(msg_value);
            }
            relative_contactmap_msg_publisher_->publish(relative_contactmap_msg);

            // Get nearest contact, break if none exist
            if (sorted_contacts_.size() > 0)
            {
                collision_detection::Contact nearest_contact = sorted_contacts_[0];

                // Publish contact object associated with nearest collision
                moveit_msgs::msg::ContactInformation nearest_msg;
                collision_detection::contactToMsg(nearest_contact, nearest_msg);
                nearest_contact_publisher_->publish(nearest_msg);

                // Publish distance associated with nearest contact
                std_msgs::msg::Float64 distance_msg;
                distance_msg.data = nearest_contact.depth;
                nearest_distance_publisher_->publish(distance_msg);

                // Publish direction associated with nearest contact
                geometry_msgs::msg::Vector3 direction_msg;
                direction_msg.x = nearest_contact.normal[0];
                direction_msg.y = nearest_contact.normal[1];
                direction_msg.z = nearest_contact.normal[2];
                nearest_direction_publisher_->publish(direction_msg);
                geometry_msgs::msg::Point vec_point = contactToPoint(nearest_contact); //testing before full integration with arduino

                geometry_msgs::msg::Vector3 directional_distance_msg;//testing before full integration with arduino
                directional_distance_msg.x = vec_point.x;//testing before full integration with arduino
                directional_distance_msg.y = vec_point.y;//testing before full integration with arduino
                directional_distance_msg.z = vec_point.z;//testing before full integration with arduino
                directional_distance_publisher_->publish(directional_distance_msg);//testing before full integration with arduino
            }

            // Get nearest yaw contact, break if none exist
            if (yaw_contacts_.size() > 0)
            {
                collision_detection::Contact yaw_contact = yaw_contacts_[0];

                // Publish contact object associated with yaw collision
                moveit_msgs::msg::ContactInformation yaw_msg;
                collision_detection::contactToMsg(yaw_contact, yaw_msg);
                yaw_contact_publisher_->publish(yaw_msg);

                // Publish distance associated with yaw contact
                std_msgs::msg::Float64 distance_msg;
                distance_msg.data = yaw_contact.depth;
                yaw_distance_publisher_->publish(distance_msg);

                // Publish direction associated with yaw contact
                geometry_msgs::msg::Vector3 direction_msg;
                direction_msg.x = yaw_contact.normal[0];
                direction_msg.y = yaw_contact.normal[1];
                direction_msg.z = yaw_contact.normal[2];
                yaw_direction_publisher_->publish(direction_msg);
                geometry_msgs::msg::Point yaw_vec_point = contactToPoint(yaw_contact); //testing before full integration with arduino

                geometry_msgs::msg::Vector3 yaw_distance_msg;//testing before full integration with arduino
                yaw_distance_msg.x = yaw_vec_point.x;//testing before full integration with arduino
                yaw_distance_msg.y = yaw_vec_point.y;//testing before full integration with arduino
                yaw_distance_msg.z = yaw_vec_point.z;//testing before full integration with arduino
                yaw_directional_distance_publisher_->publish(yaw_distance_msg);//testing before full integration with arduino
            }

            // Get nearest relative contact, break if none exist
            if (relative_contacts_.size() > 0)
            {
                collision_detection::Contact relative_contact = relative_contacts_[0];

                // Publish contact object associated with relative collision
                moveit_msgs::msg::ContactInformation relative_msg;
                collision_detection::contactToMsg(relative_contact, relative_msg);
                relative_contact_publisher_->publish(relative_msg);

                // Publish distance associated with relative contact
                std_msgs::msg::Float64 distance_msg;
                distance_msg.data = relative_contact.depth;
                relative_distance_publisher_->publish(distance_msg);

                // Publish direction associated with relative contact
                geometry_msgs::msg::Vector3 direction_msg;
                direction_msg.x = relative_contact.normal[0];
                direction_msg.y = relative_contact.normal[1];
                direction_msg.z = relative_contact.normal[2];
                relative_direction_publisher_->publish(direction_msg);
            }
        }

        std::vector<collision_detection::Contact> get_sorted_contacts(collision_detection::CollisionResult res)
        {
            std::vector<collision_detection::Contact> sorted_contacts;
            for (auto const& key_value : res.contacts)
            {
                // Split into keys and values for readability
                auto key = key_value.first;
                auto contact = key_value.second[0]; // Only get nearest contact

                // Enforce condition
                if (isRobotObstacleContact(contact))
                {
                    // Add nearest contact between pair of links
                    sorted_contacts.push_back(setContactDirection(contact));
                }
            }
            std::sort(sorted_contacts.begin(), sorted_contacts.end(), compareContacts);
            return sorted_contacts;
        }

        bool isRobotLink(std::string link)
        {
            return (std::find(robot_link_vector_.begin(), robot_link_vector_.end(), link) != robot_link_vector_.end());
        }

        bool isRobotAttached(collision_detection::BodyTypes::Type type)
        {
            return (type == collision_detection::BodyTypes::ROBOT_ATTACHED);
        }
        
        bool isRobotObstacleContact(collision_detection::Contact contact)
        {
            // Query locked planning scene's allowed collision matrix to see if contact objects can collide
            collision_detection::AllowedCollision::Type allowed_collision_type;
            bool has_entry = false;
            try
            {
                has_entry = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getAllowedCollisionMatrix().getEntry(contact.body_name_1, contact.body_name_2, allowed_collision_type);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
            }
            
            // If objects can collide and exactly one is a world object (i.e. not robot link or robot attached), return true. 
            // Else, return false.
            if 
            (
                (
                    !(has_entry) or
                    (allowed_collision_type == collision_detection::AllowedCollision::NEVER)
                ) and
                (
                    (
                        (isRobotAttached(contact.body_type_1)) or
                        (isRobotLink(contact.body_name_1))
                    ) !=
                    (
                        (isRobotAttached(contact.body_type_2)) or 
                        (isRobotLink(contact.body_name_2))
                    )
                )
            )
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        std::string contactMapToString(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Convert the key list to a string
            std::stringstream key_value_list_str;
            // Iterate over key-value pairs of contact_map
            for (const auto& key_value : contact_map) 
            {
                // Separate keys from values for readability
                const std::pair<std::string, std::string>& key = key_value.first;
                auto contact = key_value.second[0];

                // Enforce condition
                if (isRobotObstacleContact(contact))
                {
                    // First add the keys, each of which is a pair of link names, for links in contact
                    key_value_list_str << "Contact: (" << key.first << ", " << key.second << "), Vector: [";
                    // Optionally add the depth, normal, and position associated of the Contact object, i.e. a distance vector
                    // for (const collision_detection::Contact& contact : value) 
                    // {
                    //     key_value_list_str << "{depth: " << contact.depth << ", normal: " << contact.normal << ", pos: " << contact.pos << "}, ";
                    // }
                    // Just publish the first (smallest) depth so we can see the pairwise nearest distances
                    key_value_list_str << contact.depth;
                    // End entry
                    key_value_list_str << "]" << '\\';
                }
            }

            // Convert result to string type
            std::string result = key_value_list_str.str();

            return result;
        }

        std::pair<bool, collision_detection::Contact> getNearestContact()
        {
            std::pair<bool, collision_detection::Contact> pair;
            try
            {
                pair.second = sorted_contacts_[0];
                pair.first = true;
                throw 0;
            }
            catch (...)
            {
                collision_detection::Contact empty_contact;
                pair.second = empty_contact;
                pair.first = false;
            }
            return pair;
        }

        // If contact points from Obstacle to Robot, flip it
        collision_detection::Contact setContactDirection(collision_detection::Contact contact)
        {
            if (
                !(
                    (isRobotAttached(contact.body_type_1)) or 
                    (isRobotLink(contact.body_name_1))
                ) 
                and
                (
                    (isRobotAttached(contact.body_type_2)) or 
                    (isRobotLink(contact.body_name_2))
                )
            )
            {
                std::swap(contact.body_name_1, contact.body_name_2);
                std::swap(contact.body_type_1, contact.body_type_2);
                contact.pos = contact.pos + (contact.normal * contact.depth);
                contact.normal = contact.normal * (-1);
            }
            return contact;
        }

        geometry_msgs::msg::Point contactToPoint(collision_detection::Contact contact)
        {
            Eigen::Vector3d vector = contact.normal * contact.depth;
            geometry_msgs::msg::Point point;
            point.x = vector[0];
            point.y = vector[1];
            point.z = vector[2];
            return point;
        }

        collision_detection::Contact transformContact(collision_detection::Contact contact, Eigen::Affine3d tf)
        {
            // Returns a copy of contact where:
            // contact.pos is given in the transformed reference frame
            // contact.normal is given in the transformed reference frame
            // contact.depth is the depth of collision as usual

            // Transform the contact position and normal to the link frame
            Eigen::Vector3d contact_pos_tf_frame = tf * contact.pos;
            Eigen::Vector3d contact_normal_tf_frame = tf.linear() * contact.normal;

            // Create a new contact object to be returned
            collision_detection::Contact out;
            out.body_name_1 = contact.body_name_1;
            out.body_name_2 = contact.body_name_2;
            out.body_type_1 = contact.body_type_1;
            out.body_type_2 = contact.body_type_2;
            out.depth = contact.depth;
            out.pos = contact_pos_tf_frame;
            out.normal = contact_normal_tf_frame;

            return out;
        }

        collision_detection::Contact contactToRelativeContact(collision_detection::Contact contact)
        {
            // Returns a copy of contact where:
            // contact.pos is given in the link's reference frame
            // contact.normal is given in the link's reference frame
            // contact.depth is the position of the contact relative to the height of the link model's bounding box

            // Get transformation matrix from world frame to link frame
            Eigen::Affine3d link_tf = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState().getGlobalLinkTransform(contact.body_name_1).inverse();

            // Transform the contact position and normal to the link frame
            Eigen::Vector3d contact_pos_link_frame = link_tf * contact.pos;
            Eigen::Vector3d contact_normal_link_frame = link_tf.linear() * contact.normal;

            // Get the length of the link by finding the z_distance of the transform of the next link
            float z_extent = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getRobotModel()->getLinkModel(contact.body_name_1)->getShapeExtentsAtOrigin()[2];

            // Create a new contact object to be returned
            collision_detection::Contact out;
            out.body_name_1 = contact.body_name_1;
            out.body_name_2 = contact.body_name_2;
            out.body_type_1 = contact.body_type_1;
            out.body_type_2 = contact.body_type_2;
            out.depth = contact.depth; // This is where z_extent goes if wanted instead of depth
            out.pos = contact_pos_link_frame;
            out.normal = contact_normal_link_frame;

            return out;
        }

        void visualize(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::msg::MarkerArray markers;
            // The function below works for any contact map, but can only create sphere markers. The implementation further below is very similar.
            /* 
            collision_detection::getCollisionMarkersFromContacts(markers, "world", contact_map, color,
                                                                rclcpp::Duration(),  // remain until deleted
                                                                0.01);            // radius
            */

            // Iterate over key-value pairs of contact_map
            std::map<std::string, unsigned> ns_counts;
            // Record the lowest (max_markers) distance values and return only the nearest (max_markers) collisions

            // Select nearest n contacts
            std::vector<collision_detection::Contact>::size_type num_markers = std::min(max_markers_, sorted_contacts_.size());
            std::vector<collision_detection::Contact> nearest_n_contacts(sorted_contacts_.begin(), sorted_contacts_.begin() + num_markers);
            // std::vector<collision_detection::Contact> nearest_n_relative(relative_contacts_.begin(), relative_contacts_.begin() + num_markers);

            // Define colors for visualization
            std_msgs::msg::ColorRGBA color_near;
            color_near.r = 1.0;
            color_near.g = 0.0;
            color_near.b = 0.0;
            color_near.a = 0.5;
            std_msgs::msg::ColorRGBA color_far;
            color_far.r = 0.0;
            color_far.g = 1.0;
            color_far.b = 0.0;
            color_far.a = 0.5;
            std::vector<std_msgs::msg::ColorRGBA> colors = {color_near, color_far};
            // Visualize nearest n contacts
            contactVectorToMarkerArray(nearest_n_contacts, markers, "world", colors, ns_counts);

            /*
            // Visualize nearest n relative contacts (for validation, should output same as above)
            std_msgs::msg::ColorRGBA color_near_rel;
            color_near_rel.r = 1.0;
            color_near_rel.g = 0.0;
            color_near_rel.b = 1.0;
            color_near_rel.a = 0.5;
            std_msgs::msg::ColorRGBA color_far;
            color_far_rel.r = 1.0;
            color_far_rel.g = 1.0;
            color_far_rel.b = 0.0;
            color_far_rel.a = 0.5;
            std::vector<std_msgs::msg::ColorRGBA> colors_rel = {color_near_rel, color_far_rel};
            contactVectorToMarkerArray(nearest_n_relative, markers, "", colors_rel, ns_counts); // Empty frame_id means use link frames
            */

            publishMarkers(markers);
        }

        visualization_msgs::msg::Marker contactToMarker(collision_detection::Contact& contact, std::string& frame_id, std_msgs::msg::ColorRGBA& color, rclcpp::Duration& lifetime, std::map<std::string, unsigned>& ns_counts)
        {
            Eigen::Vector3d p0 = contact.pos; // p0 is the position reported by the Contact
            Eigen::Vector3d p1; // p1 is p0 + depth * normal
            Eigen::Vector3d n = contact.normal; // get normal vector for readability
            double d = contact.depth; // get depth value for readibility
            for (int i = 0; i < p0.size(); i++) // p1 is p0 + depth * normal
            {
                p1[i] = p0[i] + d * n[i];
            }

            std::vector<Eigen::Vector3d> vec;
            vec.push_back(p0);
            vec.push_back(p1);
            
            std::vector<geometry_msgs::msg::Point> points; // Put points in the array type accepted by Marker
            for (int i = 0; i < vec.size(); i++)
            {
                geometry_msgs::msg::Point point;
                point.x = vec[i][0];
                point.y = vec[i][1];
                point.z = vec[i][2];
                points.push_back(point);
            }

            std::string ns_name = contact.body_name_1 + "=" + contact.body_name_2; // String name
            if (ns_counts.find(ns_name) == ns_counts.end())
                ns_counts[ns_name] = 0;
            else
                ns_counts[ns_name]++;
            visualization_msgs::msg::Marker mk; // Instantiate marker
            mk.header.stamp = this->get_clock()->now(); // Timestamp
            mk.header.frame_id = frame_id; // Reference frame id
            mk.ns = ns_name; // String name
            mk.id = ns_counts[ns_name]; // Unique number id
            mk.type = visualization_msgs::msg::Marker::ARROW; // Arrow marker shape
            mk.action = visualization_msgs::msg::Marker::ADD; // Add shape to Rviz
            mk.points = points; // Start and end points of arrow
            mk.scale.x = 0.01; // Arrow shaft diameter
            mk.scale.y = 0.02; // Arrow head diameter
            mk.scale.z = 0.02; // Arrow head length
            mk.color = color; // Color specified above
            mk.lifetime = rclcpp::Duration(1,0); // Remain until deleted
            mk.lifetime = lifetime; // Remain for n sec or until replaced
            return mk;
        }

        void contactVectorToMarkerArray(std::vector<collision_detection::Contact>& contact_vector, visualization_msgs::msg::MarkerArray& markers, std::string frame_id, std::vector<std_msgs::msg::ColorRGBA>& colors, std::map<std::string, unsigned>& ns_counts)
        {
            for (auto contact : contact_vector)
            {
                // Set the reference frame per link if given empty
                std::string marker_frame = frame_id;
                if (marker_frame == "")
                {
                    marker_frame = contact.body_name_1;
                }

                double d = contact.depth; // get depth value for readibility

                // Use depth value to calculate a color along the spectrum given
                double color_d = std::max(0.0001, d); // Don't divide by 0
                double color_proportion = std::max(0.0, std::min(1.0, 1 - 1.0 * std::sqrt(0.050 / color_d))); // Use depth to determine color. Increases with distance For collisions, red should max out around 50 mm from collision, and shouldn't decay too fast.
                int color_index_1 = std::min(static_cast<int>(std::floor(color_proportion * colors.size())), static_cast<int>(colors.size()) - 1);
                int color_index_2 = color_index_1 + 1;
                double color_interp_factor = color_proportion * colors.size() - color_index_1;
                std_msgs::msg::ColorRGBA color_1 = colors[color_index_1];
                std_msgs::msg::ColorRGBA color_2 = colors[color_index_2];
                std_msgs::msg::ColorRGBA color;
                color.r = color_1.r * (1 - color_interp_factor) + color_2.r * color_interp_factor;
                color.g = color_1.g * (1 - color_interp_factor) + color_2.g * color_interp_factor;
                color.b = color_1.b * (1 - color_interp_factor) + color_2.b * color_interp_factor;
                color.a = color_1.a * (1 - color_interp_factor) + color_2.a * color_interp_factor;

                rclcpp::Duration lifetime(1,0);
                
                visualization_msgs::msg::Marker mk = contactToMarker(contact, marker_frame, color, lifetime, ns_counts);
                markers.markers.push_back(mk); // Add to MarkerArray markers
            }
        }

        void publishMarkers(visualization_msgs::msg::MarkerArray& markers)
        {
            // Delete old markers
            if (!collision_points_.markers.empty())
            {
                for (auto& marker : collision_points_.markers)
                marker.action = visualization_msgs::msg::Marker::DELETE;

                // marker_array_publisher_->publish(collision_points_);
            }

            // Move new markers into collision_points_
            std::swap(collision_points_.markers, markers.markers);

            // Draw new markers (if there are any)
            if (!collision_points_.markers.empty())
                marker_array_publisher_->publish(collision_points_);
        }

        void planning_scene_fake_callback(moveit_msgs::msg::PlanningScene msg)
        {
            attached_collision_object_vector_ = msg.robot_state.attached_collision_objects;
        }

        void test()
        {
            moveit_msgs::msg::CollisionObject mesh_object;
            mesh_object.header.frame_id = "world";
            mesh_object.id = "test_mesh";
            std::string test_mesh_path = "file:///root/catkin_ws/src/morpheus_description/meshes/components/collision/block.obj";
            const Eigen::Vector3d scale_eigen(0.1, 0.1, 0.1); // mm/inch
            shapes::Mesh* m = shapes::createMeshFromResource(test_mesh_path, scale_eigen);
            shape_msgs::msg::Mesh mesh_msg;
            shapes::ShapeMsg shape_msg;
            shapes::constructMsgFromShape(m, shape_msg);
            mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
            mesh_object.meshes.resize(1);
            mesh_object.meshes[0] = mesh_msg;
            mesh_object.pose.position.x = 0.3;
            mesh_object.pose.position.y = 0.35;
            mesh_object.pose.position.z = 0.8;
            mesh_object.pose.orientation.w = 1.0;
            mesh_object.operation = mesh_object.ADD;
            moveit_msgs::msg::AttachedCollisionObject mesh_attach;
            mesh_attach.object = mesh_object;
            mesh_attach.link_name = "wrist_3_link";
            
            // Publish planning scene diff
            moveit_msgs::msg::PlanningScene planning_scene;
            planning_scene.world.collision_objects.push_back(mesh_object);
            planning_scene.is_diff = true;
            planning_scene.robot_state.is_diff = true;

            // g_collision_object_publisher->publish(collision_object);

            // Process message
            RCLCPP_INFO_STREAM(this->get_logger(), "Spawning object");
            try {
                // rclcpp::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::msg::ApplyPlanningScene>("apply_planning_scene");
                // planning_scene_diff_client.waitForExistence();
                // moveit_msgs::msg::ApplyPlanningScene srv;
                // srv.request.scene = planning_scene;
                // planning_scene_diff_client.call(srv);

                //planning_scene_interface_->applyPlanningScene(planning_scene);
                //planning_scene_monitor_->newPlanningSceneMessage(planning_scene);
                //planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->usePlanningSceneMsg(planning_scene);
                //planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processCollisionObjectMsg(mesh_object);
                RCLCPP_INFO_STREAM(this->get_logger(), "Spawn succeeded");
            } catch (...) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Spawn failed");
            }

            std::vector<moveit_msgs::msg::CollisionObject> print_object;
            planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCollisionObjectMsgs(print_object);
            
            // Publish planning scene diff
            moveit_msgs::msg::PlanningScene planning_scene_attach;
            planning_scene_attach.robot_state.attached_collision_objects.push_back(mesh_attach);
            planning_scene_attach.is_diff = true;
            planning_scene_attach.robot_state.is_diff = true;

            moveit_msgs::msg::PlanningScene print_planning_scene;
            planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getPlanningSceneMsg(print_planning_scene);
            //RCLCPP_INFO_STREAM(this->get_logger(), print_planning_scene);
            
            // Process message
            RCLCPP_INFO_STREAM(this->get_logger(), "Attaching object");
            try {
                //rclcpp::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::msg::ApplyPlanningScene>("apply_planning_scene");
                //planning_scene_diff_client.waitForExistence();
                //moveit_msgs::msg::ApplyPlanningScene srv;
                //srv.request.scene = planning_scene_attach;
                //planning_scene_diff_client.call(srv);

                //planning_scene_interface_.applyPlanningScene(planning_scene_attach);
                //planning_scene_monitor_->newPlanningSceneMessage(planning_scene_attach);
                //planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->usePlanningSceneMsg(planning_scene_attach);
                planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_)->processAttachedCollisionObjectMsg(mesh_attach);
                RCLCPP_INFO_STREAM(this->get_logger(), "Attach succeeded");
            } catch (...) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Attach failed");
            }

            std::vector<moveit_msgs::msg::AttachedCollisionObject> print_attach;
            planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getAttachedCollisionObjectMsgs(print_attach);
            
            for (moveit_msgs::msg::CollisionObject obj : print_object)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), obj.id);
            }
            for (moveit_msgs::msg::AttachedCollisionObject att : print_attach)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), att.link_name);
            }
            
            moveit_msgs::msg::PlanningScene print_attach_planning_scene;
            planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getPlanningSceneMsg(print_attach_planning_scene);
            RCLCPP_INFO_STREAM(this->get_logger(), print_attach_planning_scene.name);
        }
        
    private:
        // Define a comparator for sorting contacts by depth
        static const bool compareContacts (const collision_detection::Contact a, const collision_detection::Contact b)
        {
            return a.depth < b.depth;
        }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionNode>());
    rclcpp::shutdown();
    return 0;
}
