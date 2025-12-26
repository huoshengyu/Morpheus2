#include <filesystem>
#include <functional>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <morpheus_msgs/ContactMap.h>
#include <morpheus_msgs/StringPair.h>

// name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Vector of names of links included in the robot for collision detection
static const std::vector<std::string> A_BOT_LINK_VECTOR
{
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tcp_link", // tcp_link does not have collision. May cause misalignment in data if included.
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

// Vector of names of links included in the obstacles for collision detection
static const std::vector<std::string> OBSTACLE_VECTOR
{
    "teapot",
    "cylinder",
    "test_wall_1",
    "wall",
    "foam"
};

// Get directory of data folder. Note: cwd is /root/.ros/ by default
static const std::string data_dir = "/root/catkin_ws/src/morpheus/morpheus_data/data/";

class DataNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        std::chrono::high_resolution_clock::time_point t1;
        std::ofstream g_file;
        std::string g_filename;
        std::stringstream g_next_line;

        ros::Subscriber g_contactmap_subscriber;
        ros::Subscriber g_nearest_collision_subscriber;
        inline static morpheus_msgs::ContactMap g_latest_contactmap;
        inline static bool received_contactmap;
        inline static moveit_msgs::ContactInformation g_nearest_collision;
        inline static bool received_nearest_collision;

        std::vector<std::string> g_robot_links;
        std::vector<std::string> g_obstacle_links;
        std::vector<std::string> g_column_label_vector;
        std::map<std::string, int> g_column_label_index_map;

        DataNode(int argc, char** argv)
        {
            // Select links to track
            g_robot_links = A_BOT_LINK_VECTOR;
            g_obstacle_links = OBSTACLE_VECTOR;
            
            // Initialize ROS node
            ros::init(argc, argv, "data");
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();
            
            // Retrieve preexisting PlanningSceneMonitor, if possible
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene"))
            {
                ROS_INFO("Planning Scene Monitor is active and ready.");
            }
            else
            {
                ROS_ERROR("Failed to set up Planning Scene Monitor.");
            }
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene"); // Get scene updates from topic
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("/joint_states");

            g_contactmap_subscriber = nh.subscribe("collision/contactmap", 1, contactMapCallback);
            g_nearest_collision_subscriber = nh.subscribe("collision/nearest", 1, nearestCollisionCallback);

            // Write a header for the file
            // TODO: make header strings dynamic, based on rostopics or arguments
            std::stringstream header;
            std::string robot = "UR5e";
            std::string task = "test1";
            std::string id = "0";
            std::time_t datetime_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            struct tm * timeinfo;
            timeinfo = localtime (&datetime_t);
            char datetime_buffer [80];
            strftime(datetime_buffer,80,"%F_%Hh%Mm%Ss",timeinfo);
            header << robot << "_" << task << "_" << id << "_" << datetime_buffer;
            g_filename = header.str();

            // Open file
            ROS_INFO_STREAM("Creating file with name: " << g_filename);
            std::stringstream filepath;
            filepath << data_dir << g_filename << ".csv";
            g_file = std::ofstream(filepath.str());
            if ( (g_file.rdstate() & std::ofstream::failbit ) != 0 )
                std::cerr << "Error opening file\n";
            // g_file << header.str() << std::endl;

            // Store column labels
            // Time
            g_column_label_vector.push_back("time");
            // Contact information for nearest contact
            g_column_label_vector.push_back("nearest_collision_contact_body_1");
            g_column_label_vector.push_back("nearest_collision_body_type_1");
            g_column_label_vector.push_back("nearest_collision_contact_body_2");
            g_column_label_vector.push_back("nearest_collision_body_type_2");
            g_column_label_vector.push_back("nearest_collision_position_x");
            g_column_label_vector.push_back("nearest_collision_position_y");
            g_column_label_vector.push_back("nearest_collision_position_z");
            g_column_label_vector.push_back("nearest_collision_normal_x");
            g_column_label_vector.push_back("nearest_collision_normal_y");
            g_column_label_vector.push_back("nearest_collision_normal_z");
            g_column_label_vector.push_back("nearest_collision_depth");
            // Robot joints for tracking joint angles
            std::map<std::string, double> current_state_values = g_planning_scene_monitor->getStateMonitor()->getCurrentStateValues();
            for (auto const& key_value : current_state_values)
            {
                g_column_label_vector.push_back(key_value.first);
            }
            // Robot links, subdivided by dimensions for tracking positions and rotations
            for (std::string link : g_robot_links)
            {
                g_column_label_vector.push_back(link + "_position_x");
                g_column_label_vector.push_back(link + "_position_y");
                g_column_label_vector.push_back(link + "_position_z");
                g_column_label_vector.push_back(link + "_quaternion_x");
                g_column_label_vector.push_back(link + "_quaternion_y");
                g_column_label_vector.push_back(link + "_quaternion_z");
                g_column_label_vector.push_back(link + "_quaternion_w");
            }
            // Robot links, subdivided by dimensions for tracking collision vectors
            for (std::string link : g_robot_links)
            {
                g_column_label_vector.push_back(link + "_collision_x");
                g_column_label_vector.push_back(link + "_collision_y");
                g_column_label_vector.push_back(link + "_collision_z");
            }

            // Index column labels
            std::stringstream column_label_ss;
            for (int i = 0; i < g_column_label_vector.size(); ++i)
            {
                g_column_label_index_map[g_column_label_vector[i]] = i;
                column_label_ss << g_column_label_vector[i] << ", ";
            }
            // Add labels to file
            column_label_ss << std::endl;
            g_file << column_label_ss.str();

            // Prepare for data collection
            received_contactmap = false;
            received_nearest_collision = false;
        }

        void spin()
        {
            // Start clock
            t1 = std::chrono::high_resolution_clock::now();

            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                // Spin once to invoke subscriber callback(s)
                ros::spinOnce();
                // Check if a new contactmap was received
                if (received_contactmap and received_nearest_collision)
                {
                    ROS_INFO_STREAM("Updating...");
                    update();
                    ROS_INFO_STREAM("Publishing...");
                    publish();
                }
                // If no new contact map, don't publish and print to terminal
                else
                {
                    if (!received_contactmap)
                    {
                        ROS_INFO_STREAM("Waiting for collision/contactmap topic...");
                    }
                    if (!received_nearest_collision)
                    {
                        ROS_INFO_STREAM("Waiting for collision/nearest topic...");
                    }   
                }
                
                loop_rate.sleep();
            }

            // Close when complete
            g_file.close();
        }

        void update()
        {
            g_next_line.str(""); // Reset next string to be printed
            auto robot_state_and_time = g_planning_scene_monitor->getStateMonitor()->getCurrentStateAndTime(); // Retrieve state and time
            auto robot_state_ptr = robot_state_and_time.first;
            auto robot_time = robot_state_and_time.second;

            // Get time since start
            // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> time_elapsed = t2 - t1;
            // g_next_line << time_elapsed.count() << ", ";
            g_next_line << robot_time.toSec() << ", ";

            // Get info about the nearest collision
            // Body 1 (should always be robot link)
            g_next_line << g_nearest_collision.contact_body_1 << ", ";
            // ROBOT_LINK=0, WORLD_OBJECT=1, ROBOT_ATTACHED=2
            g_next_line << g_nearest_collision.body_type_1 << ", ";
            // Body 2 (should always be world object)
            g_next_line << g_nearest_collision.contact_body_2 << ", ";
            // ROBOT_LINK=0, WORLD_OBJECT=1, ROBOT_ATTACHED=2
            g_next_line << g_nearest_collision.body_type_2 << ", ";
            // Contact Position
            g_next_line << g_nearest_collision.position.x << ", ";
            g_next_line << g_nearest_collision.position.y << ", ";
            g_next_line << g_nearest_collision.position.z << ", ";
            // Contact Normal
            g_next_line << g_nearest_collision.normal.x << ", ";
            g_next_line << g_nearest_collision.normal.y << ", ";
            g_next_line << g_nearest_collision.normal.z << ", ";
            // Contact depth
            g_next_line << g_nearest_collision.depth << ", ";

            // Get joint angles
            // Should include a column for every robot joint
            std::map<std::string, double> current_state_values = getCurrentStateValues();
            for (auto const& key_value : current_state_values)
            {
                g_next_line << key_value.second << ", ";
            }

            // Get link positions
            // Should include 3 columns for every robot link; 1 column per dimension
            std::vector<double> sorted_positions(7 * g_robot_links.size());
            for (std::size_t i = 0; i != g_robot_links.size(); ++i)
            {
                const Eigen::Affine3d & frame_transform = robot_state_ptr->getFrameTransform(g_robot_links[i]);
                auto translation = frame_transform.translation(); // Retrieve translation part
                auto rotation = frame_transform.rotation(); // Retrieve rotation part
                Eigen::Quaterniond quaternion(rotation);
                
                sorted_positions[7 * i + 0] = translation[0];
                sorted_positions[7 * i + 1] = translation[1];
                sorted_positions[7 * i + 2] = translation[2];
                sorted_positions[7 * i + 3] = quaternion.x();
                sorted_positions[7 * i + 4] = quaternion.y();
                sorted_positions[7 * i + 5] = quaternion.z();
                sorted_positions[7 * i + 6] = quaternion.w();
            }
            for (double distance : sorted_positions)
            {
                g_next_line << distance << ", ";
            }

            // Get contact map
            // Should include 3 columns for every robot link; 1 column per dimension
            std::vector<moveit_msgs::ContactInformation> contact_info_vector = g_latest_contactmap.values;
            std::vector<double> sorted_distances(3 * g_robot_links.size());

            // Add a collision distance for each robot link, keeping them sorted by the order in the link vector
            // Note: The collision node should only be reporting the nearest contacts to begin with, so no checking is done here
            for (moveit_msgs::ContactInformation contact_info : contact_info_vector)
            {
                std::string link_1 = contact_info.contact_body_1;
                std::string link_2 = contact_info.contact_body_2;
                
                // Need to check both links. Collision node does not swap these to always be robot-obstacle order.
                auto it = std::find(g_robot_links.begin(), g_robot_links.end(), link_1);
                if (it != g_robot_links.end())
                {
                    int index = it - g_robot_links.begin();
                    geometry_msgs::Vector3 collision_normal = contact_info.normal;
                    std::vector<double> collision_vector = {collision_normal.x, collision_normal.y, collision_normal.z};
                    std::transform(collision_vector.begin(), collision_vector.end(), collision_vector.begin(),
                        std::bind(std::multiplies<double>(), std::placeholders::_1, contact_info.depth));
                    sorted_distances[3 * index + 0] = collision_vector[0];
                    sorted_distances[3 * index + 1] = collision_vector[1];
                    sorted_distances[3 * index + 2] = collision_vector[2];
                }
                it = std::find(g_robot_links.begin(), g_robot_links.end(), link_2);
                if (it != g_robot_links.end())
                {
                    int index = it - g_robot_links.begin();
                    geometry_msgs::Vector3 collision_normal = contact_info.normal;
                    std::vector<double> collision_vector = {collision_normal.x, collision_normal.y, collision_normal.z};
                    std::transform(collision_vector.begin(), collision_vector.end(), collision_vector.begin(),
                        std::bind(std::multiplies<double>(), std::placeholders::_1, -contact_info.depth)); // If robot link is link 2, negate depth
                    sorted_distances[3 * index + 0] = collision_vector[0];
                    sorted_distances[3 * index + 1] = collision_vector[1];
                    sorted_distances[3 * index + 2] = collision_vector[2];
                }
            }
            for (double distance : sorted_distances)
            {
                g_next_line << distance << ", ";
            }

        }

        void publish()
        {
            ROS_INFO_STREAM("Adding line: " << g_next_line.str());
            g_file << g_next_line.str() << std::endl;
        }

        std::map<std::string, double> getCurrentStateValues()
        {
            return g_planning_scene_monitor->getStateMonitor()->getCurrentStateValues();
        }

        static void emptyCallback(std_msgs::String msg)
        {
            
        }

        static void contactMapCallback(morpheus_msgs::ContactMap msg)
        {
            g_latest_contactmap = msg;
            received_contactmap = true;
            ROS_INFO_STREAM("Received contactmap");
        }

        static void nearestCollisionCallback(moveit_msgs::ContactInformation nearest)
        {
            g_nearest_collision = nearest;
            received_nearest_collision = true;
            ROS_INFO_STREAM("Received nearest collision");
        }


};

int main(int argc, char** argv)
{
    DataNode data_node(argc, argv);
    data_node.spin();
    return 0;
}
