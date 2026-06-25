#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>

namespace visualizer
{

};

class VisualizerNode
{
    public:
        rclcpp::Subscriber model_states_msg_subscriber_;
        inline static gazebo_msgs::ModelStates model_states_msg_;
        inline static bool received_model_states_msg;
        rclcpp::Publisher marker_array_publisher_;
        visualization_msgs::MarkerArray model_markers_;

        VisualizerNode(int argc, char** argv)
        {
            // Initialize ROS node
            rclcpp::init(argc, argv, "visualizer");
            rclcpp::NodeHandle nh;
            rclcpp::AsyncSpinner spinner(0);
            spinner.start();
                        
            // Create a subscriber to receive model state updates
            model_states_msg_subscriber_ = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, gazeboModelStateCallback);

            // Create a marker array publisher for publishing shapes to Rviz
            marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

            // Create an array of markers
            visualization_msgs::MarkerArray model_markers_();

            // Instantiate visual tools for visualizing markers in Rviz
            // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, "world", "/moveit_visual_tools");

            // Add callback which dictates behavior after each scene update
            // planning_scene_monitor->addUpdateCallback
            
            // Get a list of all links in the robot so we can check them for collisions

            // rclcpp::shutdown();
        }

        void spin()
        {
            // Loop at specified rate
            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok())
            {
                // Spin once to invoke subscriber callback(s)
                rclcpp::spinOnce();

                // If message received, visualize
                if (received_model_states_msg)
                {
                    visualize(model_states_msg_);
                }
            }

            // Spin the ROS node
            rclcpp::spin();
        }

        void visualize(gazebo_msgs::ModelStates model_states_msg)
        {
            // Set a color for the visualization markers
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
            color.a = 0.5;

            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::MarkerArray markers;

            // Keep count so that separate namespaces can be kept
            std::map<std::string, unsigned> ns_counts;

            // Visualize models
            for (std::size_t i = 0; i < model_states_msg.name.size(); i++)
            {
                std::string name = model_states_msg.name[i];
                
                if ((name == "world") or
                    (name == "robot") or
                    (name == "ground_plane"))
                {
                    continue;
                }

                geometry_msgs::Pose pose = model_states_msg.pose[i];
                geometry_msgs::Twist twist = model_states_msg.twist[i];
                double scale = 0.0254;
                double x_len = 1;
                double y_len = 12;
                double z_len = 1;
                
                std::string ns_name = name; // String name
                if (ns_counts.find(ns_name) == ns_counts.end())
                    ns_counts[ns_name] = 0;
                else
                    ns_counts[ns_name]++;
                visualization_msgs::Marker mk; // Instantiate marker
                mk.header.stamp = rclcpp::Time::now(); // Timestamp
                mk.header.frame_id = "world"; // Reference frame id
                mk.ns = ns_name; // String name
                mk.id = ns_counts[ns_name]; // Unique number id
                mk.pose = pose; // Pose of the shape
                mk.color = color; // Color specified above
                mk.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
                mk.lifetime = rclcpp::Duration(0.5); // Remain for 0.5 sec or until replaced
                mk.type = visualization_msgs::Marker::CUBE;
                mk.mesh_resource = ""; // Use custom mesh (if type = visualization_msgs::Marker::MESH_RESOURCE)
                mk.scale.x = scale * x_len; // X scale
                mk.scale.y = scale * y_len; // Y scale
                mk.scale.z = scale * z_len; // Z scale
                markers.markers.push_back(mk); // Add to MarkerArray markers
            }

            publishMarkers(markers);
        }

        void publishMarkers(visualization_msgs::MarkerArray& markers)
        {
            // delete old markers
            if (!model_markers_.markers.empty())
            {
                for (auto& marker : model_markers_.markers)
                marker.action = visualization_msgs::Marker::DELETE;

                // marker_array_publisher_->publish(model_markers_);
            }

            // move new markers into model_markers_
            std::swap(model_markers_.markers, markers.markers);

            // draw new markers (if there are any)
            if (!model_markers_.markers.empty())
                marker_array_publisher_.publish(model_markers_);
        }
        
    private:
        // Define a callback to update to be called when the PlanningSceneMonitor receives an update
        static void gazeboModelStateCallback(const gazebo_msgs::ModelStates::SharedPtr msg)
        {
            RCLCPP_INFO(morpheus_spawner->get_logger(),"Updating...");
            model_states_msg_ = *msg;
            received_model_states_msg = true;
        }

};