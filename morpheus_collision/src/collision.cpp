#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <morpheus_msgs/msg/contact_map.hpp>

// name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Set of names of links included in the robot for collision detection (a param name, so it can be changed externally)
static const std::vector<std::string> A_BOT_LINK_VECTOR_DEFAULT
{
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tcp_link",
    "tcp_collision_link",
    "d415_mount_link",
    "camera_link",
    "coupler",
    "cable_protector",
    "gripper_body",
    "left_outer_knuckle",
    "left_outer_finger",
    "left_inner_finger",
    "left_inner_finger_pad",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "right_outer_finger",
    "right_inner_finger",
    "right_inner_finger_pad",
    "right_inner_knuckle"
};

// Set of names of links excluded from the obstacles for collision detection (a param name, so it can be changed externally)
static const std::vector<std::string> ALLOWED_COLLISION_VECTOR_DEFAULT
{
    "base_link",
    "base_link_inertia",
    "pedestal",
    "simple_pedestal"
};

namespace collision
{

};

class CollisionNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        ros::Publisher g_contactmap_string_publisher;
        ros::Publisher g_contactmap_msg_publisher;
        ros::Publisher g_nearest_contact_publisher;
        ros::Publisher g_nearest_distance_publisher;
        ros::Publisher g_nearest_direction_publisher;
        collision_detection::CollisionResult g_c_res;
        collision_detection::CollisionRequest g_c_req;
        std::vector<collision_detection::Contact> g_sorted_contacts;

        ros::Publisher* g_marker_array_publisher = nullptr;
        visualization_msgs::msg::MarkerArray g_collision_points;
        // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
        std::vector<collision_detection::Contact>::size_type g_max_markers = 10;

        std::vector<std::string> A_BOT_LINK_VECTOR;
        std::vector<std::string> ALLOWED_COLLISION_VECTOR;

        CollisionNode(int argc, char** argv)
        {
            // Initialize ROS node
            rclcpp::init(argc, argv);
            auto nh = rclcpp::Node::make_shared("collision");
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

            // Get robot and obstacle vectors from ros server, if possible
            if (ros::param::get("/A_BOT_LINK_VECTOR", A_BOT_LINK_VECTOR))
            {
                RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Using A_BOT_LINK_VECTOR from parameter server");
            }
            else
            {
                A_BOT_LINK_VECTOR = A_BOT_LINK_VECTOR_DEFAULT;
                RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Using A_BOT_LINK_VECTOR_DEFAULT");
            }
            if (ros::param::get("/ALLOWED_COLLISION_VECTOR", ALLOWED_COLLISION_VECTOR))
            {
                RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Using ALLOWED_COLLISION_VECTOR from parameter server");
            }
            else
            {
                ALLOWED_COLLISION_VECTOR = ALLOWED_COLLISION_VECTOR_DEFAULT;
                RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Using ALLOWED_COLLISION_VECTOR_DEFAULT");
            }

            // Retrieve preexisting PlanningSceneMonitor, if possible
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);

            // Set update callback
            // g_planning_scene_monitor->addUpdateCallback(planningSceneMonitorCallback);

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene"))
            {
                RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Planning Scene Monitor is active and ready.");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("MorpheusCollision"), "Failed to set up Planning Scene Monitor.");
            }

            // Request the PlanningScene itself and change collision detection engine to Bullet
            // planning_scene::PlanningScenePtr planning_scene;
            // Get read/write pointer to planning_scene
            try
            {   
                // Change the PlanningScene's collision detector to Bullet
                // Bullet supports distance vectors, as well as distances to multiple obstacles
                g_planning_scene_monitor->getPlanningScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), 
                                                    true /* exclusive */);
                
                if (strcmp((g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName()).c_str(), "Bullet") == 0)
                {
                    RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Planning Scene is active and ready.");
                }    
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Collision detector incorrect");
                    // RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName());
                    std::string collision_detector_name = g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName();
                    throw collision_detector_name;
                }
            }
            catch (std::string collision_detector_name)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MorpheusCollision"), "Failed to retrieve PlanningScene.");
            }
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene"); // Get scene updates from topic
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("/joint_states");

            // Prepare collision result and request objects
            g_c_req.contacts = true;
            g_c_req.distance = true;
            g_c_req.max_contacts = 1;
            g_c_req.max_contacts_per_pair = 1;
            
            /*
            // Edit the allowed collision matrix to focus only on robot-obstacle collisions
            collision_detection::AllowedCollisionMatrix allowed_collision_matrix = 
                g_planning_scene_monitor->getPlanningScene().getAllowedCollisionMatrix();
            allowed_collision_matrix.setEntry(true); // Allow all collisions
            allowed_collision_matrix.setEntry("teapot", false); // Register collisions involving teapot
            */

            // Create collision publishers
            g_contactmap_string_publisher = nh.advertise<std_msgs::msg::String>("collision/contactmap/string", 0);
            g_contactmap_msg_publisher = nh.advertise<morpheus_msgs::msg::ContactMap>("collision/contactmap/msg", 0);
            g_nearest_contact_publisher = nh.advertise<moveit_msgs::msg::ContactInformation>("collision/nearest/contact", 0);
            g_nearest_distance_publisher = nh.advertise<std_msgs::msg::Float64>("collision/nearest/distance", 0);
            g_nearest_direction_publisher = nh.advertise<geometry_msgs::msg::Vector3>("collision/nearest/direction", 0);

            
            // Create a marker array publisher for publishing shapes to Rviz
            g_marker_array_publisher =
                new ros::Publisher(nh.advertise<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 0));
            
            // Instantiate visual tools for visualizing markers in Rviz
            // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, "world", "/moveit_visual_tools");

            // Add callback which dictates behavior after each scene update
            // planning_scene_monitor->addUpdateCallback
            
            // Get a list of all links in the robot so we can check them for collisions


            // ros::shutdown();
        }

        void spin()
        {
            // Loop collision requests and publish at specified rate
            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok())
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

            // Spin the ROS node
            rclcpp::spin(node);
        }

        void update()
        {
            // Update the planning scene monitor, in case new collision objects have been added
            // g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene");
            // Update the collision result, based on the collision request
            g_c_res.clear();
            g_planning_scene_monitor->getPlanningScene()->checkCollision(g_c_req, g_c_res);
            g_sorted_contacts = get_sorted_contacts();


            /*
            rclcpp::Time update_time = g_planning_scene_monitor->getLastUpdateTime(); // Get last update time
            std::stringstream update_time_ss; // Instantiate stringstream for concatenation
            update_time_ss << update_time.sec << "." << update_time.nsec; // Concatenate seconds.nanoseconds
            std::string update_time_str = update_time_ss.str(); // Convert to std::string
            RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), update_time_str.c_str()); // Convert to const char*
            */
        }

        void publish()
        {

            // Publish contact map as text string
            std_msgs::msg::String contacts_msg;
            contacts_msg.data = contactMapToString(g_c_res.contacts);
            g_contactmap_string_publisher.publish(contacts_msg);

            // Publish contact map as special msg type
            morpheus_msgs::msg::ContactMap contactmap_msg;
            for (auto const& key_value : g_c_res.contacts)
            {
                auto key = key_value.first;
                auto value = key_value.second[0]; // Only get nearest contact

                morpheus_msgs::msg::StringPair msg_key;
                msg_key.first = key.first;
                msg_key.second = key.second;

                moveit_msgs::msg::ContactInformation msg_value;
                msg_value = contactToContactInformation(value);

                contactmap_msg.keys.push_back(msg_key);
                contactmap_msg.values.push_back(msg_value);
            }
            g_contactmap_msg_publisher.publish(contactmap_msg);

            // Publish contact object associated with nearest collision
            collision_detection::Contact nearest_contact = g_sorted_contacts[0];
            moveit_msgs::msg::ContactInformation nearest_msg = contactToContactInformation(nearest_contact);
            g_nearest_contact_publisher.publish(nearest_msg);

            // Publish distance associated with nearest contact
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = nearest_contact.depth;
            g_nearest_distance_publisher.publish(distance_msg);

            // Publish direction associated with nearest contact
            geometry_msgs::msg::Vector3 direction_msg;
            direction_msg.x = nearest_contact.normal[0];
            direction_msg.y = nearest_contact.normal[1];
            direction_msg.z = nearest_contact.normal[2];
            g_nearest_direction_publisher.publish(direction_msg);

        }

        std::vector<collision_detection::Contact> get_sorted_contacts()
        {
            std::vector<collision_detection::Contact> sorted_contacts;
            for (auto const& key_value : g_c_res.contacts)
            {
                // Split into keys and values for readability
                auto key = key_value.first;
                auto value = key_value.second[0]; // Only get nearest contact

                // Enforce condition
                if (isRobotObstaclePair(key))
                {
                    // Add nearest contact between pair of links
                    sorted_contacts.push_back(setContactDirection(value));
                }
            }
            std::sort(sorted_contacts.begin(), sorted_contacts.end(), compareContacts);
            return sorted_contacts;
        }

        bool isRobotObstaclePair(std::pair<std::string, std::string> pair)
        {
            if 
            (
                !(std::find(ALLOWED_COLLISION_VECTOR.begin(), ALLOWED_COLLISION_VECTOR.end(), pair.first) != ALLOWED_COLLISION_VECTOR.end()) and
                !(std::find(ALLOWED_COLLISION_VECTOR.begin(), ALLOWED_COLLISION_VECTOR.end(), pair.second) != ALLOWED_COLLISION_VECTOR.end()) and
                (
                    (
                        (std::find(A_BOT_LINK_VECTOR.begin(), A_BOT_LINK_VECTOR.end(), pair.first) != A_BOT_LINK_VECTOR.end()) and
                        !(std::find(A_BOT_LINK_VECTOR.begin(), A_BOT_LINK_VECTOR.end(), pair.second) != A_BOT_LINK_VECTOR.end())
                    )
                    or 
                    (
                        !(std::find(A_BOT_LINK_VECTOR.begin(), A_BOT_LINK_VECTOR.end(), pair.first) != A_BOT_LINK_VECTOR.end()) and
                        (std::find(A_BOT_LINK_VECTOR.begin(), A_BOT_LINK_VECTOR.end (), pair.second) != A_BOT_LINK_VECTOR.end())
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
                const std::vector<collision_detection::Contact>& value = key_value.second;

                // Enforce condition
                if (isRobotObstaclePair(key))
                {
                    // First add the keys, each of which is a pair of link names, for links in contact
                    key_value_list_str << "Contact: (" << key.first << ", " << key.second << "), Vector: [";
                    // Optionally add the depth, normal, and position associated of the Contact object, i.e. a distance vector
                    // for (const collision_detection::Contact& contact : value) 
                    // {
                    //     key_value_list_str << "{depth: " << contact.depth << ", normal: " << contact.normal << ", pos: " << contact.pos << "}, ";
                    // }
                    // Just publish the first (smallest) depth so we can see the pairwise nearest distances
                    const collision_detection::Contact& contact = value[0];
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
            if (!(std::find(A_BOT_LINK_VECTOR.begin(), A_BOT_LINK_VECTOR.end(), contact.body_name_1) != A_BOT_LINK_VECTOR.end()) and
                (std::find(A_BOT_LINK_VECTOR.begin(), A_BOT_LINK_VECTOR.end(), contact.body_name_2) != A_BOT_LINK_VECTOR.end()))
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

        moveit_msgs::msg::ContactInformation contactToContactInformation(collision_detection::Contact contact)
        {
            // Initialize and add header
            moveit_msgs::msg::ContactInformation ci;
            ci.header.stamp = rclcpp::Time::now(); // Timestamp
            ci.header.frame_id = "world"; // Reference frame id

            // Convert contact coordinates to geometry_msgs::msg::Point
            geometry_msgs::msg::Point contact_point;
            contact_point.x = contact.pos[0];
            contact_point.y = contact.pos[1];
            contact_point.z = contact.pos[2];
            ci.position = contact_point;

            // Convert contact normal to geometry_msgs::msg::Vector3 (which should always be a normalized vector)
            geometry_msgs::msg::Vector3 contact_vector;
            contact_vector.x = contact.normal[0];
            contact_vector.y = contact.normal[1];
            contact_vector.z = contact.normal[2];
            ci.normal = contact_vector;

            // Transfer other info
            ci.depth = contact.depth;
            ci.contact_body_1 = contact.body_name_1;
            ci.body_type_1 = contact.body_type_1;
            ci.contact_body_2 = contact.body_name_2;

            return ci;
        }

        void visualize(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Set a color for the visualization markers
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 0.5;

            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::msg::MarkerArray markers;
            // The function below works for any contact map, but can only create sphere markers
            /* 
            collision_detection::getCollisionMarkersFromContacts(markers, "world", contact_map, color,
                                                                rclcpp::Duration(),  // remain until deleted
                                                                0.01);            // radius
            */

            // Iterate over key-value pairs of contact_map
            std::map<std::string, unsigned> ns_counts;
            // Record the lowest (max_markers) distance values and return only the nearest (max_markers) collisions

            // Select nearest n contacts
            std::vector<collision_detection::Contact>::size_type num_markers = std::min(g_max_markers, g_sorted_contacts.size());
            std::vector<collision_detection::Contact> nearest_n_contacts(g_sorted_contacts.begin(), g_sorted_contacts.begin() + num_markers);

            // Visualize nearest n contacts
            for (auto contact : nearest_n_contacts)
            {
                std::vector<Eigen::Vector3d> vec;
                Eigen::Vector3d p0 = contact.pos; // p0 is the position reported by the Contact
                Eigen::Vector3d p1; // p1 is p0 + depth * normal
                double d = contact.depth; // get depth value for readibility
                double color_d = std::max(0.0001, d); // Don't divide by 0
                color.r = std::min(1.0, 1.0 * std::sqrt(0.050 / color_d)); // Use depth to determine color. Red should max out around 50 mm from collision, and shouldn't decay too fast.
                color.g = 1 - color.r;
                Eigen::Vector3d n = contact.normal; // get normal vector for readability
                for (int i = 0; i < p0.size(); i++) // p1 is p0 + depth * normal
                {
                    p1[i] = p0[i] + d * n[i];
                }
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
                mk.header.stamp = rclcpp::Time::now(); // Timestamp
                mk.header.frame_id = "world"; // Reference frame id
                mk.ns = ns_name; // String name
                mk.id = ns_counts[ns_name]; // Unique number id
                mk.type = visualization_msgs::msg::Marker::ARROW; // Arrow marker shape
                mk.action = visualization_msgs::msg::Marker::ADD; // Add shape to Rviz
                mk.points = points; // Start and end points of arrow
                mk.scale.x = 0.01; // Arrow shaft diameter
                mk.scale.y = 0.02; // Arrow head diameter
                mk.scale.z = 0.02; // Arrow head length
                mk.color = color; // Color specified above
                // mk.lifetime = rclcpp::Duration(); // Remain until deleted
                mk.lifetime = rclcpp::Duration(0.5); // Remain for 0.5 sec or until replaced
                markers.markers.push_back(mk); // Add to MarkerArray markers
            }

            publishMarkers(markers);
        }

        void publishMarkers(visualization_msgs::msg::MarkerArray& markers)
        {
            // delete old markers
            if (!g_collision_points.markers.empty())
            {
                for (auto& marker : g_collision_points.markers)
                marker.action = visualization_msgs::msg::Marker::DELETE;

                // g_marker_array_publisher->publish(g_collision_points);
            }

            // move new markers into g_collision_points
            std::swap(g_collision_points.markers, markers.markers);

            // draw new markers (if there are any)
            if (!g_collision_points.markers.empty())
                g_marker_array_publisher->publish(g_collision_points);
        }
        
    private:
        // Define a callback to update to be called when the PlanningSceneMonitor receives an update
        static void planningSceneMonitorCallback(const moveit_msgs::msg::PlanningScene::ConstSharedPtr& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
        {
            RCLCPP_INFO(rclcpp::get_logger("MorpheusCollision"), "Updating...");
        }

        // Define a comparator for sorting contacts by depth
        static const bool compareContacts (const collision_detection::Contact a, const collision_detection::Contact b)
        {
            return a.depth < b.depth;
        }

};

int main(int argc, char** argv)
{
    CollisionNode collision_node(argc, argv);
    collision_node.spin();
    return 0;
}
