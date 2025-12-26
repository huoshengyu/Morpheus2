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

namespace analysis
{

};

// Get directory of data folder. Note: cwd is /root/.ros/ by default
static const std::string data_dir = "/root/catkin_ws/src/morpheus_data/data/";

class AnalysisNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        std::chrono::high_resolution_clock::time_point t1;
        std::ifstream g_ifile;
        std::ofstream g_ofile;
        std::string g_filename;
        std::stringstream g_next_line;
        std::vector<std::vector<std::string>> g_data;

        std::vector<std::string> g_robot_links;
        std::vector<std::string> g_obstacle_links;
        std::vector<std::string> g_column_label_vector;

        // Need some variables to be global

        AnalysisNode(int argc, char** argv)
        {
            // Select links to track
            g_robot_links = A_BOT_LINK_VECTOR;
            g_obstacle_links = OBSTACLE_VECTOR;
            
        }

        void read(std::string filename)
        {
            // Read from file
            g_filename = filename;
            ROS_INFO_STREAM("Reading file with name: " << filename);
            std::stringstream filepath;
            filepath << data_dir << filename << ".csv";
            g_ifile = std::ifstream(filepath.str());
            if ( (g_ifile.rdstate() & std::ifstream::failbit ) != 0 )
                std::cerr << "Error opening file\n";

            // Read title and header lines
            std::string title_str;
            std::getline(g_ifile, title_str); // Result should be same as filename

            std::string header_str;
            std::getline(g_ifile, header_str); // Contains names for each column of data
            std::vector<std::string> header_cells = lineToCells(header_str);

            // Loop over remaining lines
            std::string next_line_str;
            while(std::getline(g_ifile, next_line_str))
            {
                std::getline(g_ifile, next_line_str);
                g_next_line.str(next_line_str);
                std::vector<std::string> next_line_cells = lineToCells(next_line_str);
                g_data.push_back(next_line_cells);
            }
        }

        void analyze()
        {
            // Analyze data that has been previously read and save the results
            // Base write filename on read filename
            std::string write_filename = "analyze" + g_filename;
            ROS_INFO_STREAM("Writing file with name: " << write_filename);
            std::stringstream filepath;
            filepath << data_dir << write_filename << ".csv";
            g_ofile = std::ofstream(filepath.str());
            if ( (g_ofile.rdstate() & std::ifstream::failbit ) != 0 )
                std::cerr << "Error opening file\n";

        }

        std::vector<std::string> lineToCells(std::string line)
        {
            std::vector<std::string>   result;
            std::stringstream          lineStream(line);
            std::string                cell_str;

            while(std::getline(lineStream,cell_str, ','))
            {
                result.push_back(cell_str);
            }
            // // This checks for a trailing comma with no data after it.
            // if (!lineStream && cell.empty())
            // {
            //    // If there was a trailing comma then add an empty element.
            //     result.push_back("");
            // }
            return result;
        }

        static void emptyCallback(std_msgs::String msg)
        {
            
        }

};

int main(int argc, char** argv)
{
    AnalysisNode analysis_node(argc, argv);
    if (argc == 1)
    {
        ROS_INFO("No command line arguments given. Please provide data filenames");
    }
    else
    {
        for (std::size_t i = 1; i < argc; ++i)
        {
            std::string filename = argv[i];
            analysis_node.read(filename);
            analysis_node.analyze();

        }
    }
    return 0;
};
