// General
#include <algorithm>
#include <chrono>

// ROS
#include <ros/ros.h>

// Moveit
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_distance_field/collision_env_hybrid.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/collision_distance_field/collision_detector_allocator_distance_field.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <morpheus_msgs/ContactMap.h>

// Shapes
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_state/attached_body.h>

// Eigen
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Names of groups in srdf which encompass the robot itself (and not the environment)
static const std::string ARM_GROUP_DEFAULT = "arm";
static const std::string GRIPPER_GROUP_DEFAULT = "gripper";

namespace collision
{

};

class CollisionNode
{
    public:
        // Declare interfaces for interacting with the planning scene
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        moveit::planning_interface::PlanningSceneInterface g_planning_scene_interface;

        // Declare publishers
        ros::Publisher g_contactmap_string_publisher;
        ros::Publisher g_contactmap_msg_publisher;
        ros::Publisher g_nearest_contact_publisher;
        ros::Publisher g_nearest_distance_publisher;
        ros::Publisher g_nearest_direction_publisher;
        ros::Publisher g_yaw_contactmap_msg_publisher;
        ros::Publisher g_yaw_contact_publisher;
        ros::Publisher g_yaw_distance_publisher;
        ros::Publisher g_yaw_direction_publisher;
        ros::Publisher g_relative_contactmap_msg_publisher;
        ros::Publisher g_relative_contact_publisher;
        ros::Publisher g_relative_distance_publisher;
        ros::Publisher g_relative_direction_publisher;
        ros::Publisher g_directional_distance_publisher; //testing before full integration with arduino
        ros::Publisher yaw_directional_distance_publisher; //testing before full integration with arduino

        // Declare collision info variables
        collision_detection::CollisionResult g_c_res;
        collision_detection::CollisionRequest g_c_req;
        std::vector<collision_detection::Contact> g_sorted_contacts;
        std::vector<collision_detection::Contact> g_yaw_contacts;
        std::vector<collision_detection::Contact> g_relative_contacts;

        // Declare collision visualization variables
        ros::Publisher* g_marker_array_publisher = nullptr;
        visualization_msgs::MarkerArray g_collision_points;
        moveit_visual_tools::MoveItVisualToolsPtr g_visual_tools;
        std::vector<collision_detection::Contact>::size_type g_max_markers = 20;

        // Declare variables for identifying robot vs environment
        std::vector<std::string> g_robot_link_vector;
        std::string g_arm_group;
        std::string g_gripper_group;
        
        // Declare interfaces for retrieving robot link models and other info
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_arm_interface;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_gripper_interface;
        
        // Declare variables for tracking attached collision objects
        ros::Subscriber g_planning_scene_fake_subscriber;
        std::vector<moveit_msgs::AttachedCollisionObject> g_attached_collision_object_vector;

        // Declare ROS node handle
        ros::NodeHandle nh;

        CollisionNode(int argc, char** argv)
        {
            // Initialize ROS node
            ros::AsyncSpinner spinner(0);
            spinner.start();

            // Create a RobotModelLoader to load the robot's URDF and SRDF
            // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            // robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

            // Create a PlanningScene object and set the robot model
            // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

            // Create a PlanningSceneMonitor around the PlanningScene
            // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
            //     new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

            // Get arm and gripper groups from ros server, if possible
            if (ros::param::get("~arm_group", g_arm_group))
            {
                ROS_INFO("Using arm_group from parameter server");
            }
            else
            {
                g_arm_group = ARM_GROUP_DEFAULT;
                ROS_INFO("Using ARM_GROUP_DEFAULT");
            }
            if (ros::param::get("~gripper_group", g_gripper_group))
            {
                ROS_INFO("Using gripper_group from parameter server");
            }
            else
            {
                g_gripper_group = GRIPPER_GROUP_DEFAULT;
                ROS_INFO("Using GRIPPER_GROUP_DEFAULT");
            }

            // Start move group interfaces for retrieving robot links
            g_arm_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(g_arm_group);
            g_gripper_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(g_gripper_group);

            // Set PlanningSceneInterface in namespace
            g_planning_scene_interface = *new moveit::planning_interface::PlanningSceneInterface(ros::this_node::getNamespace());

            // Instantiate PlanningSceneMonitor
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("move_group/monitored_planning_scene"); // Get scene updates from topic

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("get_planning_scene"))
            {
                ROS_INFO("Planning Scene Monitor is active and ready.");
            }
            else
            {
                ROS_ERROR("Failed to set up Planning Scene Monitor.");
            }

            // Request the PlanningScene itself and change collision detection engine to Bullet
            // planning_scene::PlanningScenePtr planning_scene;
            // Get read/write pointer to planning_scene
            try
            {   
                // Change the PlanningScene's collision detector to Bullet
                // Bullet supports distance vectors, as well as distances to multiple obstacles
                planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_monitor)->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), 
                                                    true /* exclusive */);
                
                if (strcmp((planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getActiveCollisionDetectorName()).c_str(), "Bullet") == 0)
                {
                    ROS_INFO("Planning Scene is active and ready.");
                }    
                else
                {
                    ROS_INFO("Collision detector incorrect");
                    ROS_INFO_STREAM(planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getActiveCollisionDetectorName());
                    std::string collision_detector_name = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getActiveCollisionDetectorName();
                    throw collision_detector_name;
                }
            }
            catch (std::string collision_detector_name)
            {
                ROS_ERROR("Failed to retrieve PlanningScene.");
            }

            // Prepare collision result and request objects
            g_c_req.contacts = true;
            g_c_req.distance = true;
            g_c_req.max_contacts = 1;
            g_c_req.max_contacts_per_pair = 1;
            
            /*
            // Edit the allowed collision matrix to focus only on robot-obstacle collisions
            collision_detection::AllowedCollisionMatrix allowed_collision_matrix = 
                planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_monitor)->getAllowedCollisionMatrix();
            allowed_collision_matrix.setEntry(true); // Allow all collisions
            allowed_collision_matrix.setEntry("teapot", false); // Register collisions involving teapot
            */

            // Create collision publishers
            g_contactmap_string_publisher = nh.advertise<std_msgs::String>("collision/contactmap/string", 0);
            g_contactmap_msg_publisher = nh.advertise<morpheus_msgs::ContactMap>("collision/contactmap/msg", 0);
            g_nearest_contact_publisher = nh.advertise<moveit_msgs::ContactInformation>("collision/nearest/contact", 0);
            g_nearest_distance_publisher = nh.advertise<std_msgs::Float64>("collision/nearest/distance", 0);
            g_nearest_direction_publisher = nh.advertise<geometry_msgs::Vector3>("collision/nearest/direction", 0);
            g_directional_distance_publisher = nh.advertise<geometry_msgs::Vector3>("collision/nearest/directional_distance", 0); //testing before full integration with arduino
            g_yaw_contactmap_msg_publisher = nh.advertise<morpheus_msgs::ContactMap>("collision/yaw/contactmap/msg", 0);
            g_yaw_contact_publisher = nh.advertise<moveit_msgs::ContactInformation>("collision/yaw/contact", 0);
            g_yaw_distance_publisher = nh.advertise<std_msgs::Float64>("collision/yaw/distance", 0);
            g_yaw_direction_publisher = nh.advertise<geometry_msgs::Vector3>("collision/yaw/direction", 0);
            g_relative_contactmap_msg_publisher = nh.advertise<morpheus_msgs::ContactMap>("collision/relative/contactmap/msg", 0);
            g_relative_contact_publisher = nh.advertise<moveit_msgs::ContactInformation>("collision/relative/contact", 0);
            g_relative_distance_publisher = nh.advertise<std_msgs::Float64>("collision/relative/distance", 0);
            g_relative_direction_publisher = nh.advertise<geometry_msgs::Vector3>("collision/relative/direction", 0);
            yaw_directional_distance_publisher = nh.advertise<geometry_msgs::Vector3>("collision/yaw/yaw_distance", 0); //testing before full integration with arduino

            
            // Create a marker array publisher for publishing shapes to Rviz
            g_marker_array_publisher =
                new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0));
            
            // Subscribe to fake planning scene for recognizing "fake" attached collision objects
            g_planning_scene_fake_subscriber = nh.subscribe("planning_scene_fake", 1, &CollisionNode::planning_scene_fake_callback, this);

            // Set robot link vector to include all parts of the robot
            // Get all links in the robot arm
            std::vector<std::string> arm_link_vector = g_arm_interface->getLinkNames();
            g_robot_link_vector.insert(g_robot_link_vector.end(), arm_link_vector.begin(), arm_link_vector.end());

            // Get all links in the robot gripper
            std::vector<std::string> gripper_link_vector = g_gripper_interface->getLinkNames();
            g_robot_link_vector.insert(g_robot_link_vector.end(), gripper_link_vector.begin(), gripper_link_vector.end());

            // Instantiate visual tools for visualizing markers in Rviz
            g_visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("world","moveit_visual_markers"));
        }

        void spin()
        {
            // Create asynchronous spinner to allow callbacks while looping
            ros::AsyncSpinner spinner(2); // Use 2 threads
            spinner.start();

            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                update();
                publish();
                visualize(g_c_res.contacts);
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
            // g_planning_scene_monitor->requestPlanningSceneState();

            // Update the collision result, based on the collision request
            g_c_res.clear();
            // Get locked planning scene to ensure scene does not change during update
            planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->checkCollision(g_c_req, g_c_res);

            // Get sorted contacts so that the top N can be selected
            g_sorted_contacts = get_sorted_contacts(g_c_res);
            // Transform contacts based on yaw, which is assumed to be the first joint of the robot
            g_yaw_contacts.clear();
            std::string yaw_joint = g_arm_interface->getVariableNames()[0];
            const double* yaw = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState().getJointPositions(yaw_joint);
            Eigen::Affine3d yaw_tf; // Must be first declared, then assigned since there is no implicit constructor from AngleAxis
            yaw_tf = Eigen::AngleAxisd(-*yaw, Eigen::Vector3d(0.0,0.0,1.0));
            for (auto contact : g_sorted_contacts)
            {
                g_yaw_contacts.push_back(transformContact(contact, yaw_tf));
            }
            // Transform contacts to the frame of the link they are associated with
            g_relative_contacts.clear();
            for (auto contact : g_sorted_contacts)
            {
                g_relative_contacts.push_back(contactToRelativeContact(contact));
            }

            /*
            ros::Time update_time = g_planning_scene_monitor->getLastUpdateTime(); // Get last update time
            std::stringstream update_time_ss; // Instantiate stringstream for concatenation
            update_time_ss << update_time.sec << "." << update_time.nsec; // Concatenate seconds.nanoseconds
            std::string update_time_str = update_time_ss.str(); // Convert to std::string
            ROS_INFO(update_time_str.c_str()); // Convert to const char*
            */
        }

        void publish()
        {

            // Publish contact map as text string
            std_msgs::String contacts_msg;
            contacts_msg.data = contactMapToString(g_c_res.contacts);
            g_contactmap_string_publisher.publish(contacts_msg);

            // Publish contact map (sorted and reduced) as custom msg type
            morpheus_msgs::ContactMap contactmap_msg;
            for (auto const& contact : g_sorted_contacts)
            {
                morpheus_msgs::StringPair msg_key;
                msg_key.first = contact.body_name_1;
                msg_key.second = contact.body_name_2;

                moveit_msgs::ContactInformation msg_value;
                collision_detection::contactToMsg(contact, msg_value);

                contactmap_msg.keys.push_back(msg_key);
                contactmap_msg.values.push_back(msg_value);
            }
            g_contactmap_msg_publisher.publish(contactmap_msg);

            // Publish link contact map just like regular contact map
            morpheus_msgs::ContactMap yaw_contactmap_msg;
            for (auto const& contact : g_yaw_contacts)
            {
                morpheus_msgs::StringPair msg_key;
                msg_key.first = contact.body_name_1;
                msg_key.second = contact.body_name_2;

                moveit_msgs::ContactInformation msg_value;
                collision_detection::contactToMsg(contact, msg_value);

                yaw_contactmap_msg.keys.push_back(msg_key);
                yaw_contactmap_msg.values.push_back(msg_value);
            }
            g_yaw_contactmap_msg_publisher.publish(yaw_contactmap_msg);

            // Publish link contact map just like regular contact map
            morpheus_msgs::ContactMap relative_contactmap_msg;
            for (auto const& contact : g_relative_contacts)
            {
                morpheus_msgs::StringPair msg_key;
                msg_key.first = contact.body_name_1;
                msg_key.second = contact.body_name_2;

                moveit_msgs::ContactInformation msg_value;
                collision_detection::contactToMsg(contact, msg_value);

                relative_contactmap_msg.keys.push_back(msg_key);
                relative_contactmap_msg.values.push_back(msg_value);
            }
            g_relative_contactmap_msg_publisher.publish(relative_contactmap_msg);

            // Get nearest contact, break if none exist
            if (g_sorted_contacts.size() > 0)
            {
                collision_detection::Contact nearest_contact = g_sorted_contacts[0];

                // Publish contact object associated with nearest collision
                moveit_msgs::ContactInformation nearest_msg;
                collision_detection::contactToMsg(nearest_contact, nearest_msg);
                g_nearest_contact_publisher.publish(nearest_msg);

                // Publish distance associated with nearest contact
                std_msgs::Float64 distance_msg;
                distance_msg.data = nearest_contact.depth;
                g_nearest_distance_publisher.publish(distance_msg);

                // Publish direction associated with nearest contact
                geometry_msgs::Vector3 direction_msg;
                direction_msg.x = nearest_contact.normal[0];
                direction_msg.y = nearest_contact.normal[1];
                direction_msg.z = nearest_contact.normal[2];
                g_nearest_direction_publisher.publish(direction_msg);
                geometry_msgs::Point vec_point = contactToPoint(nearest_contact); //testing before full integration with arduino

                geometry_msgs::Vector3 directional_distance_msg;//testing before full integration with arduino
                directional_distance_msg.x = vec_point.x;//testing before full integration with arduino
                directional_distance_msg.y = vec_point.y;//testing before full integration with arduino
                directional_distance_msg.z = vec_point.z;//testing before full integration with arduino
                g_directional_distance_publisher.publish(directional_distance_msg);//testing before full integration with arduino
            }

            // Get nearest yaw contact, break if none exist
            if (g_yaw_contacts.size() > 0)
            {
                collision_detection::Contact yaw_contact = g_yaw_contacts[0];

                // Publish contact object associated with yaw collision
                moveit_msgs::ContactInformation yaw_msg;
                collision_detection::contactToMsg(yaw_contact, yaw_msg);
                g_yaw_contact_publisher.publish(yaw_msg);

                // Publish distance associated with yaw contact
                std_msgs::Float64 distance_msg;
                distance_msg.data = yaw_contact.depth;
                g_yaw_distance_publisher.publish(distance_msg);

                // Publish direction associated with yaw contact
                geometry_msgs::Vector3 direction_msg;
                direction_msg.x = yaw_contact.normal[0];
                direction_msg.y = yaw_contact.normal[1];
                direction_msg.z = yaw_contact.normal[2];
                g_yaw_direction_publisher.publish(direction_msg);
                geometry_msgs::Point yaw_vec_point = contactToPoint(yaw_contact); //testing before full integration with arduino

                geometry_msgs::Vector3 yaw_distance_msg;//testing before full integration with arduino
                yaw_distance_msg.x = yaw_vec_point.x;//testing before full integration with arduino
                yaw_distance_msg.y = yaw_vec_point.y;//testing before full integration with arduino
                yaw_distance_msg.z = yaw_vec_point.z;//testing before full integration with arduino
                yaw_directional_distance_publisher.publish(yaw_distance_msg);//testing before full integration with arduino
            }

            // Get nearest relative contact, break if none exist
            if (g_relative_contacts.size() > 0)
            {
                collision_detection::Contact relative_contact = g_relative_contacts[0];

                // Publish contact object associated with relative collision
                moveit_msgs::ContactInformation relative_msg;
                collision_detection::contactToMsg(relative_contact, relative_msg);
                g_relative_contact_publisher.publish(relative_msg);

                // Publish distance associated with relative contact
                std_msgs::Float64 distance_msg;
                distance_msg.data = relative_contact.depth;
                g_relative_distance_publisher.publish(distance_msg);

                // Publish direction associated with relative contact
                geometry_msgs::Vector3 direction_msg;
                direction_msg.x = relative_contact.normal[0];
                direction_msg.y = relative_contact.normal[1];
                direction_msg.z = relative_contact.normal[2];
                g_relative_direction_publisher.publish(direction_msg);
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
            return (std::find(g_robot_link_vector.begin(), g_robot_link_vector.end(), link) != g_robot_link_vector.end());
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
                has_entry = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getAllowedCollisionMatrix().getEntry(contact.body_name_1, contact.body_name_2, allowed_collision_type);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM(e.what());
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
                pair.second = g_sorted_contacts[0];
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

        geometry_msgs::Point contactToPoint(collision_detection::Contact contact)
        {
            Eigen::Vector3d vector = contact.normal * contact.depth;
            geometry_msgs::Point point;
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
            Eigen::Affine3d link_tf = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState().getGlobalLinkTransform(contact.body_name_1).inverse();

            // Transform the contact position and normal to the link frame
            Eigen::Vector3d contact_pos_link_frame = link_tf * contact.pos;
            Eigen::Vector3d contact_normal_link_frame = link_tf.linear() * contact.normal;

            // Get the length of the link by finding the z_distance of the transform of the next link
            float z_extent = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getRobotModel()->getLinkModel(contact.body_name_1)->getShapeExtentsAtOrigin()[2];

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
            visualization_msgs::MarkerArray markers;
            // The function below works for any contact map, but can only create sphere markers. The implementation further below is very similar.
            /* 
            collision_detection::getCollisionMarkersFromContacts(markers, "world", contact_map, color,
                                                                ros::Duration(),  // remain until deleted
                                                                0.01);            // radius
            */

            // Iterate over key-value pairs of contact_map
            std::map<std::string, unsigned> ns_counts;
            // Record the lowest (max_markers) distance values and return only the nearest (max_markers) collisions

            // Select nearest n contacts
            std::vector<collision_detection::Contact>::size_type num_markers = std::min(g_max_markers, g_sorted_contacts.size());
            std::vector<collision_detection::Contact> nearest_n_contacts(g_sorted_contacts.begin(), g_sorted_contacts.begin() + num_markers);
            // std::vector<collision_detection::Contact> nearest_n_relative(g_relative_contacts.begin(), g_relative_contacts.begin() + num_markers);

            // Define colors for visualization
            std_msgs::ColorRGBA color_near;
            color_near.r = 1.0;
            color_near.g = 0.0;
            color_near.b = 0.0;
            color_near.a = 0.5;
            std_msgs::ColorRGBA color_far;
            color_far.r = 0.0;
            color_far.g = 1.0;
            color_far.b = 0.0;
            color_far.a = 0.5;
            std::vector<std_msgs::ColorRGBA> colors = {color_near, color_far};
            // Visualize nearest n contacts
            contactVectorToMarkerArray(nearest_n_contacts, markers, "world", colors, ns_counts);

            /*
            // Visualize nearest n relative contacts (for validation, should output same as above)
            std_msgs::ColorRGBA color_near_rel;
            color_near_rel.r = 1.0;
            color_near_rel.g = 0.0;
            color_near_rel.b = 1.0;
            color_near_rel.a = 0.5;
            std_msgs::ColorRGBA color_far;
            color_far_rel.r = 1.0;
            color_far_rel.g = 1.0;
            color_far_rel.b = 0.0;
            color_far_rel.a = 0.5;
            std::vector<std_msgs::ColorRGBA> colors_rel = {color_near_rel, color_far_rel};
            contactVectorToMarkerArray(nearest_n_relative, markers, "", colors_rel, ns_counts); // Empty frame_id means use link frames
            */

            publishMarkers(markers);
        }

        visualization_msgs::Marker contactToMarker(collision_detection::Contact& contact, std::string& frame_id, std_msgs::ColorRGBA& color, ros::Duration& lifetime, std::map<std::string, unsigned>& ns_counts)
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
            
            std::vector<geometry_msgs::Point> points; // Put points in the array type accepted by Marker
            for (int i = 0; i < vec.size(); i++)
            {
                geometry_msgs::Point point;
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
            visualization_msgs::Marker mk; // Instantiate marker
            mk.header.stamp = ros::Time::now(); // Timestamp
            mk.header.frame_id = frame_id; // Reference frame id
            mk.ns = ns_name; // String name
            mk.id = ns_counts[ns_name]; // Unique number id
            mk.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
            mk.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
            mk.points = points; // Start and end points of arrow
            mk.scale.x = 0.01; // Arrow shaft diameter
            mk.scale.y = 0.02; // Arrow head diameter
            mk.scale.z = 0.02; // Arrow head length
            mk.color = color; // Color specified above
            // mk.lifetime = ros::Duration(); // Remain until deleted
            mk.lifetime = lifetime; // Remain for n sec or until replaced
            return mk;
        }

        void contactVectorToMarkerArray(std::vector<collision_detection::Contact>& contact_vector, visualization_msgs::MarkerArray& markers, std::string frame_id, std::vector<std_msgs::ColorRGBA>& colors, std::map<std::string, unsigned>& ns_counts)
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
                std_msgs::ColorRGBA color_1 = colors[color_index_1];
                std_msgs::ColorRGBA color_2 = colors[color_index_2];
                std_msgs::ColorRGBA color;
                color.r = color_1.r * (1 - color_interp_factor) + color_2.r * color_interp_factor;
                color.g = color_1.g * (1 - color_interp_factor) + color_2.g * color_interp_factor;
                color.b = color_1.b * (1 - color_interp_factor) + color_2.b * color_interp_factor;
                color.a = color_1.a * (1 - color_interp_factor) + color_2.a * color_interp_factor;

                ros::Duration lifetime(1.0);
                
                visualization_msgs::Marker mk = contactToMarker(contact, marker_frame, color, lifetime, ns_counts);
                markers.markers.push_back(mk); // Add to MarkerArray markers
            }
        }

        void publishMarkers(visualization_msgs::MarkerArray& markers)
        {
            // Delete old markers
            if (!g_collision_points.markers.empty())
            {
                for (auto& marker : g_collision_points.markers)
                marker.action = visualization_msgs::Marker::DELETE;

                // g_marker_array_publisher->publish(g_collision_points);
            }

            // Move new markers into g_collision_points
            std::swap(g_collision_points.markers, markers.markers);

            // Draw new markers (if there are any)
            if (!g_collision_points.markers.empty())
                g_marker_array_publisher->publish(g_collision_points);
        }

        void planning_scene_fake_callback(moveit_msgs::PlanningScene msg)
        {
            g_attached_collision_object_vector = msg.robot_state.attached_collision_objects;
        }

        void test()
        {
            moveit_msgs::CollisionObject mesh_object;
            mesh_object.header.frame_id = "world";
            mesh_object.id = "test_mesh";
            std::string test_mesh_path = "file:///root/catkin_ws/src/morpheus_description/meshes/components/collision/block.obj";
            const Eigen::Vector3d scale_eigen(0.1, 0.1, 0.1); // mm/inch
            shapes::Mesh* m = shapes::createMeshFromResource(test_mesh_path, scale_eigen);
            shape_msgs::Mesh mesh;
            shapes::ShapeMsg mesh_msg;
            shapes::constructMsgFromShape(m, mesh_msg);
            mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
            mesh_object.meshes.resize(1);
            mesh_object.meshes[0] = mesh;
            mesh_object.pose.position.x = 0.3;
            mesh_object.pose.position.y = 0.35;
            mesh_object.pose.position.z = 0.8;
            mesh_object.pose.orientation.w = 1.0;
            mesh_object.operation = mesh_object.ADD;
            moveit_msgs::AttachedCollisionObject mesh_attach;
            mesh_attach.object = mesh_object;
            mesh_attach.link_name = "wrist_3_link";
            
            // Publish planning scene diff
            moveit_msgs::PlanningScene planning_scene;
            planning_scene.world.collision_objects.push_back(mesh_object);
            planning_scene.is_diff = true;
            planning_scene.robot_state.is_diff = true;

            // g_collision_object_publisher.publish(collision_object);

            // Process message
            ROS_INFO_STREAM("Spawning object");
            try {
                ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
                planning_scene_diff_client.waitForExistence();
                moveit_msgs::ApplyPlanningScene srv;
                srv.request.scene = planning_scene;
                planning_scene_diff_client.call(srv);

                //g_planning_scene_interface.applyPlanningScene(planning_scene);
                //g_planning_scene_monitor->newPlanningSceneMessage(planning_scene);
                //planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_monitor)->usePlanningSceneMsg(planning_scene);
                planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_monitor)->processCollisionObjectMsg(mesh_object);
                ROS_INFO_STREAM("Spawn succeeded");
            } catch (...) {
                ROS_INFO_STREAM("Spawn failed");
            }

            std::vector<moveit_msgs::CollisionObject> print_object;
            planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCollisionObjectMsgs(print_object);
            
            // Publish planning scene diff
            moveit_msgs::PlanningScene planning_scene_attach;
            planning_scene_attach.robot_state.attached_collision_objects.push_back(mesh_attach);
            planning_scene_attach.is_diff = true;
            planning_scene_attach.robot_state.is_diff = true;

            moveit_msgs::PlanningScene print_planning_scene;
            planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getPlanningSceneMsg(print_planning_scene);
            //ROS_INFO_STREAM(print_planning_scene);
            
            // Process message
            ROS_INFO_STREAM("Attaching object");
            try {
                ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
                planning_scene_diff_client.waitForExistence();
                moveit_msgs::ApplyPlanningScene srv;
                srv.request.scene = planning_scene_attach;
                planning_scene_diff_client.call(srv);

                //g_planning_scene_interface.applyPlanningScene(planning_scene_attach);
                //g_planning_scene_monitor->newPlanningSceneMessage(planning_scene_attach);
                //planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_monitor)->usePlanningSceneMsg(planning_scene_attach);
                planning_scene_monitor::LockedPlanningSceneRW(g_planning_scene_monitor)->processAttachedCollisionObjectMsg(mesh_attach);
                ROS_INFO_STREAM("Attach succeeded");
            } catch (...) {
                ROS_INFO_STREAM("Attach failed");
            }

            std::vector<moveit_msgs::AttachedCollisionObject> print_attach;
            planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getAttachedCollisionObjectMsgs(print_attach);
            
            for (moveit_msgs::CollisionObject obj : print_object)
            {
                ROS_INFO_STREAM(obj);
            }
            for (moveit_msgs::AttachedCollisionObject att : print_attach)
            {
                ROS_INFO_STREAM(att);
            }
            
            moveit_msgs::PlanningScene print_attach_planning_scene;
            planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getPlanningSceneMsg(print_attach_planning_scene);
            ROS_INFO_STREAM(print_attach_planning_scene);
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
    ros::init(argc, argv, "collision");
    CollisionNode collision_node(argc, argv);

    //collision_node.test();

    collision_node.spin();
    ROS_INFO_STREAM("Stop");
    return 0;
}
