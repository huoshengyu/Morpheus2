// General Imports
#include <ros/ros.h>
// ROS Messages
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
// Moveit
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// Geometry
#include <eigen_conversions/eigen_msg.h>
// Local Imports
#include "morpheus_teleop/button_mappings.h"

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

// Names of groups in srdf which encompass the robot itself (and not the environment)
static const std::string ARM_GROUP_DEFAULT = "arm";
static const std::string GRIPPER_GROUP_DEFAULT = "gripper";

static const std::vector<std::string> GOAL_NAME_VECTOR_DEFAULT
{
    "trossen_home",
    "hera_ceiling",
};

class TrajectoryNode
{
    public:
        // Planning Scene Monitor
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;

        // Subscribers
        ros::Subscriber g_joy_subscriber;
        ros::Subscriber g_gello_subscriber;

        // Publishers
        ros::Publisher g_marker_array_publisher;
        ros::Publisher g_trajectory_publisher;
        ros::Publisher g_nearest_distance_publisher;
        ros::Publisher g_nearest_direction_publisher;
        ros::Publisher g_forward_distance_publisher;
        ros::Publisher g_forward_direction_publisher;
        ros::Publisher g_goal_distance_publisher;
        ros::Publisher g_goal_direction_publisher;
        ros::Publisher g_goal_vector_publisher; //For the directional distance
        visualization_msgs::MarkerArray g_trajectory_marker_array;
        moveit_visual_tools::MoveItVisualToolsPtr g_visual_tools;

        // Move group names
        std::string g_arm_group;
        std::string g_gripper_group;

        // Planning interfaces
        // std::shared_ptr<moveit_cpp::PlanningComponent> g_planning_components;
        // std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters> g_plan_request_parameters;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_move_group_interface;
        // moveit::planning_interface::MoveGroupInterface::Plan g_plan;

        // Planning parameters
        std::vector<robot_trajectory::RobotTrajectory> g_trajectory_vector;
        std::vector<geometry_msgs::PoseStamped> g_target_vector;
        std::vector<geometry_msgs::PoseStamped> g_weight_vector;
        std::vector<moveit_msgs::OrientationConstraint> g_constraint_vector;
        std::vector<double> g_velocity_vector;
        std::vector<std::string> g_goal_name_vector;
        std::string g_mode; // Cartesian or Joint trajectory

        // Guidance parameters
        Eigen::Affine3d g_nearest;
        Eigen::Affine3d g_forward;
        Eigen::Affine3d g_goal;
        int g_segment_index;
        std::map<std::string, int> g_cntlr;                           // Holds the controller button mappings
        bool g_segment_swapped;

        TrajectoryNode(int argc, char** argv)
        {
            // Initialize ROS node
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();

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

            // Retrieve preexisting PlanningSceneMonitor, if possible
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
            
            // Instantiate a move group interface so a trajectory can be generated
            g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(g_arm_group);

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("get_planning_scene"))
            {
                ROS_INFO("Planning Scene Monitor is active and ready.");
            }
            else
            {
                ROS_ERROR("Failed to set up Planning Scene Monitor.");
            }

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
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("move_group/monitored_planning_scene"); // Get scene updates from topic
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("joint_states");

            // Create trajectory msg publisher
            g_trajectory_publisher = nh.advertise<std_msgs::String>("trajectory/msg", 0);
            
            // Create guidance vector publishers
            g_nearest_distance_publisher = nh.advertise<std_msgs::Float64>("trajectory/nearest/distance", 0);
            g_nearest_direction_publisher = nh.advertise<geometry_msgs::Vector3>("trajectory/nearest/direction", 0);
            g_forward_distance_publisher = nh.advertise<std_msgs::Float64>("trajectory/forward/distance", 0);
            g_forward_direction_publisher = nh.advertise<geometry_msgs::Vector3>("trajectory/forward/direction", 0);
            g_goal_distance_publisher = nh.advertise<std_msgs::Float64>("trajectory/goal/distance", 0);
            g_goal_direction_publisher = nh.advertise<geometry_msgs::Vector3>("trajectory/goal/direction", 0);
            g_goal_vector_publisher = nh.advertise<geometry_msgs::Vector3>("trajectory/goal/vector", 0); //For the directional distance
            // Create a marker array publisher for publishing shapes to Rviz
            g_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
            
            // Get robot model from the current planning scene
            const robot_model::RobotModelConstPtr robot_model = g_planning_scene_monitor->getRobotModel();

            // Get the current robot state once so that it does not vary over time
            // A LockedPlanningSceneRO is used to avoid modifying the planning scene.
            // Alternatively, the robot state can be set from a given joint state or from a given pose (using IK)
            robot_state::RobotStatePtr robot_state(
                new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState()));
            robot_state::RobotStatePtr robot_state_home(new moveit::core::RobotState(robot_model));
            robot_state_home->setToDefaultValues();

            // Create a joint model group for tracking the current robot pose and planning group
            const moveit::core::JointModelGroup* joint_model_group =
                robot_state->getJointModelGroup(g_arm_group);
            std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

            // Get target vector from ros server, if possible
            if (ros::param::get("~mode", g_mode))
            {
                ROS_INFO("Using planning mode from parameter server");
            }
            else
            {
                g_mode = "cartesian";
                ROS_INFO("Using planning mode default (Cartesian)");
            }

            // Get target preset from ros server, if possible
            std::string goal_preset_param;
            std::string goal_name_vector_param;
            // If preset trajectory name is retrieved, then try getting the actual preset trajectory
            if (ros::param::get("goal/trajectory_name", goal_preset_param) && 
                ros::param::get("trajectory_presets/" + goal_preset_param, goal_name_vector_param))
            {
                g_goal_name_vector = splitString(goal_name_vector_param, ' ');
                ROS_INFO("Using goal/trajectory_name from parameter server");
            }
            // Else try getting the explicitly defined target vector
            else if (ros::param::get("goal/name_vector", goal_name_vector_param))
            {
                g_goal_name_vector = splitString(goal_name_vector_param, ' ');
                ROS_INFO("Using goal/name_vector from parameter server");
            }
            // Else fall back to the hardcoded default
            else
            {
                g_goal_name_vector = GOAL_NAME_VECTOR_DEFAULT;
                ROS_INFO("Using GOAL_NAME_VECTOR_DEFAULT");
            }

            // Get goal transforms and associated parameters from parameter server
            trajectory_msgs::JointTrajectory preset_trajectory;
            preset_trajectory.header.stamp = ros::Time::now();
            preset_trajectory.header.frame_id = "world";
            preset_trajectory.joint_names = joint_names;
            for (std::string goal_name : g_goal_name_vector)
            {
                if (ros::param::has("goal/" + goal_name))
                {
                    // Get goal transform from parameter server
                    geometry_msgs::PoseStamped target;
                    target.header.stamp = ros::Time::now();
                    target.header.frame_id = "world";
                    ros::param::get("goal/" + goal_name + "/goal_pose/position/x", target.pose.position.x);
                    ros::param::get("goal/" + goal_name + "/goal_pose/position/y", target.pose.position.y);
                    ros::param::get("goal/" + goal_name + "/goal_pose/position/z", target.pose.position.z);
                    ros::param::get("goal/" + goal_name + "/goal_pose/orientation/x", target.pose.orientation.x);
                    ros::param::get("goal/" + goal_name + "/goal_pose/orientation/y", target.pose.orientation.y);
                    ros::param::get("goal/" + goal_name + "/goal_pose/orientation/z", target.pose.orientation.z);
                    ros::param::get("goal/" + goal_name + "/goal_pose/orientation/w", target.pose.orientation.w);
                    g_target_vector.push_back(target);

                    // Get goal transform from parameter server
                    trajectory_msgs::JointTrajectoryPoint joint_target;
                    std::vector<double> joint_positions(6);
                    for (int i = 0; i < joint_names.size(); i++)
                    {
                        ros::param::get("goal/" + goal_name + "/goal_state/" + joint_names[i], joint_positions[i]);
                    }
                    joint_target.positions = joint_positions;
                    preset_trajectory.points.push_back(joint_target);

                    // Get goal weights from parameter server
                    geometry_msgs::PoseStamped weights;
                    target.header.stamp = ros::Time::now();
                    target.header.frame_id = "world";
                    ros::param::get("goal/" + goal_name + "/position_weights/x", weights.pose.position.x);
                    ros::param::get("goal/" + goal_name + "/position_weights/y", weights.pose.position.y);
                    ros::param::get("goal/" + goal_name + "/position_weights/z", weights.pose.position.z);
                    ros::param::get("goal/" + goal_name + "/orientation_weights/x", weights.pose.orientation.x);
                    ros::param::get("goal/" + goal_name + "/orientation_weights/y", weights.pose.orientation.y);
                    ros::param::get("goal/" + goal_name + "/orientation_weights/z", weights.pose.orientation.z);
                    ros::param::get("goal/" + goal_name + "/orientation_weights/w", weights.pose.orientation.w);
                    g_weight_vector.push_back(weights);

                    // Get max velocity scaling factor from parameter server
                    double velocity;
                    ros::param::get("goal/" + goal_name + "/max_velocity_scaling_factor", velocity);
                    g_velocity_vector.push_back(velocity);

                    ROS_INFO_STREAM("Retrieved goal " << goal_name << " from parameter server");
                }
                else
                {
                    ROS_INFO_STREAM("Failed to retrieve goal " << goal_name << "from parameter server!");
                }
            }
            if (g_target_vector.size() == 0)
            {
                // Generate default target pose
                geometry_msgs::PoseStamped target;
                target.header.stamp = ros::Time::now();
                target.header.frame_id = "world";
                target.pose.position.x = 0.4;
                target.pose.position.y = -0.2;
                target.pose.position.z = 1.0;
                target.pose.orientation.x = 0.5;
                target.pose.orientation.y = -0.5;
                target.pose.orientation.z = -0.5;
                target.pose.orientation.w = -0.5;
                g_target_vector.push_back(target);

                ROS_INFO_STREAM("Target vector empty, using default goal pose");
            }
            if (g_weight_vector.size() == 0)
            {
                // Generate default weights
                geometry_msgs::PoseStamped weights;
                weights.header.stamp = ros::Time::now();
                weights.header.frame_id = "world";
                weights.pose.position.x = 0.0;
                weights.pose.position.y = 1.0;
                weights.pose.position.z = 0.0;
                weights.pose.orientation.x = 0.0;
                weights.pose.orientation.y = 0.0;
                weights.pose.orientation.z = 0.0;
                weights.pose.orientation.w = 0.0;
                g_weight_vector.push_back(weights);

                ROS_INFO_STREAM("Weight vector empty, using default goal weights");
            }

            if (g_constraint_vector.size() == 0)
            {
                moveit_msgs::OrientationConstraint constraint;
                constraint.link_name = g_move_group_interface->getEndEffectorLink();
                constraint.header.frame_id = "world";
                constraint.orientation.w = -0.5;
                constraint.absolute_x_axis_tolerance = 0.1;
                constraint.absolute_y_axis_tolerance = 0.1;
                constraint.absolute_z_axis_tolerance = 0.1;
                constraint.weight = 1.0;
                g_constraint_vector.push_back(constraint);

                ROS_INFO_STREAM("Constraint vector empty, using default goal constraints");
            }
            if (g_velocity_vector.size() == 0)
            {
                g_velocity_vector.push_back(0.1);
                ROS_INFO_STREAM("Velocity vector empty, using default velocity scaling factor");
            }

            // Just use waypoints directly without planning
            if (g_mode == "preset" or g_mode == "")
            {
                ROS_INFO_STREAM("Using preset path");
                for (int i = 0; i < preset_trajectory.points.size()-1; i++)
                {
                    // For each point in the preset trajectory, make a trajectory segment to be toggled between
                    trajectory_msgs::JointTrajectory preset_segment;
                    preset_segment.header.stamp = ros::Time::now();
                    preset_segment.header.frame_id = "world";
                    preset_segment.joint_names = joint_names;
                    preset_segment.points = {preset_trajectory.points[i], preset_trajectory.points[i+1]};
                    moveit_msgs::RobotTrajectory msg;
                    msg.joint_trajectory = preset_segment;
                    robot_trajectory::RobotTrajectory trajectory(robot_model, joint_model_group);
                    trajectory.setRobotTrajectoryMsg(*robot_state, msg);
                    g_trajectory_vector.push_back(trajectory);
                }
            }
            // Plan a cartesian trajectory. More prone to failed planning.
            else if (g_mode == "cartesian")
            {
                // Treat target vector's poses as waypoint vector
                ROS_INFO_STREAM("Starting path computation");
                std::vector<geometry_msgs::Pose> waypoints;
                for (int i = 0; i < g_target_vector.size(); i++)
                {
                    ROS_INFO_STREAM(std::to_string(g_target_vector[i].pose.position.x) + " " + std::to_string(g_target_vector[i].pose.position.y) + " " + std::to_string(g_target_vector[i].pose.position.z));
                    waypoints.push_back(g_target_vector[i].pose);
                }
                double step = 0.05;
                for (int i = 0; i < waypoints.size()-1; i++)
                {
                    moveit_msgs::RobotTrajectory msg;
                    std::vector<geometry_msgs::Pose> segment = {waypoints[i], waypoints[i+1]};
                    // const moveit_msgs::Constraints path_constraints;
                    // bool avoid_collisions = true;
                    // moveit_msgs::MoveItErrorCodes error_code;
                    g_move_group_interface->computeCartesianPath(segment, step, msg);

                    // Set trajectory msg
                    ROS_INFO_STREAM("Setting msg");
                    robot_trajectory::RobotTrajectory trajectory(robot_model, joint_model_group);
                    trajectory.setRobotTrajectoryMsg(*robot_state, msg);
                    g_trajectory_vector.push_back(trajectory); // Add current trajectory to vector of all trajectories
                }
            }
            // Plan a joint space trajectory. Less intuitive, more curved paths.
            else if (g_mode == "joint")
            {
                // Iterate over all goal poses
                ROS_INFO_STREAM("Starting path computation");
                robot_state::RobotState next_start_state = *robot_state;
                for (int i = 0; i < g_target_vector.size(); i++)
                {
                    ROS_INFO_STREAM("Starting loop");
                    // Set planning parameters
                    g_move_group_interface->setPlanningTime(60); // time in seconds before timeout
                    g_move_group_interface->setPoseTarget(g_target_vector[i], g_move_group_interface->getEndEffectorLink()); // end effector pose
                    g_move_group_interface->setStartState(next_start_state); // joint state
                    g_move_group_interface->setMaxVelocityScalingFactor(g_velocity_vector[i]); // max joint velocity, from range (0,1]

                    // Create plan
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    ROS_INFO_STREAM("Starting plan()");
                    bool success = (g_move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    robot_trajectory::RobotTrajectory trajectory(robot_model, joint_model_group);
                    ROS_INFO_STREAM("Setting msg");
                    trajectory.setRobotTrajectoryMsg(*robot_state, plan.start_state_, plan.trajectory_);
                    // double delay = 1.0; // Delay between trajectory segments in seconds
                    ROS_INFO_STREAM("Starting append()");
                    g_trajectory_vector.push_back(trajectory); // Add current trajectory to vector of all trajectories
                    ROS_INFO_STREAM("Starting getLastWayPoint()");
                    // Use endpoint as next start state
                    next_start_state = trajectory.getLastWayPoint();
                }
            }
            else
            {
                ROS_ERROR("Morpheus Trajectory node requires one of the following modes: (preset, cartesian, joint)");
                return;
            }

            // Instantiate visual tools for visualizing markers in Rviz
            g_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("/world", "visualization_marker_array", g_planning_scene_monitor);

            // Set current tracked trajectory segment to index 0
            g_segment_index = 0;
            g_segment_swapped = false;

            // Set controller type based on params
            std::string controller_type;
            bool use_joy = false;
            bool use_gello = false;
            if (ros::param::get("~controller", controller_type))
            {
                if (button_mappings.find(controller_type) != button_mappings.end())
                {
                    ROS_INFO_STREAM("Controller type is " + controller_type + ". Joystick commands will be accepted by Morpheus Trajectory.");
                    g_cntlr = button_mappings.find(controller_type)->second;
                    use_joy = true;
                }
                else if (controller_type == "gello")
                {
                    ROS_INFO_STREAM("Controller type is gello. Joystick commands will be ignored by Morpheus Trajectory.");
                    use_gello = true;
                }
                else
                {
                    std::stringstream controller_ss;
                    for (std::map<std::string, std::map<std::string, int>>::const_iterator it = button_mappings.begin(); it != button_mappings.end(); ++it)
                    {
                        controller_ss << it->first;
                    }
                    ROS_INFO_STREAM("Controller type " + controller_type + " not recognized. Accepted controllers are: " + controller_ss.str() + ". Joystick commands will be ignored by Morpheus Trajectory.");
                }
            }
            else
            {
                ROS_INFO_STREAM("Controller type not received from parameter server. Defaulting to ps4.");
                controller_type = "ps4";
            }

            // Create controller msg subscribers
            if (use_joy)
            {
                g_joy_subscriber = nh.subscribe("joy", 10, &TrajectoryNode::joyCallback, this);
            }
            if (use_gello)
            {
                g_gello_subscriber = nh.subscribe("gello/joint_state", 10, &TrajectoryNode::gelloCallback, this);
            }

            spin();
            // ros::shutdown();
        }

        void spin()
        {
            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {   
                // Retrieve and update state once. Avoid updating in functions below to maintain sync.
                auto current_state = planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState();
                current_state.updateLinkTransforms();

                // Get waypoints of all trajectory segments
                std::vector<moveit::core::RobotState> waypoints;
                for (robot_trajectory::RobotTrajectory trajectory : g_trajectory_vector)
                {
                    std::vector<moveit::core::RobotState> traj_waypoints = getWaypoints(trajectory); // Get next set of waypoints
                    waypoints.insert(waypoints.end(), traj_waypoints.begin(), traj_waypoints.end()); // Concatenate
                }
                auto transform_deque = getTransforms(waypoints, g_move_group_interface->getEndEffectorLink());

                // Get waypoints of selected trajectory segment
                robot_trajectory::RobotTrajectory trajectory_segment = g_trajectory_vector[g_segment_index];
                std::vector<moveit::core::RobotState> waypoints_segment = getWaypoints(trajectory_segment);
                auto transform_deque_segment = getTransforms(waypoints_segment, g_move_group_interface->getEndEffectorLink());

                // Find relative transforms to nearest and forward sections of trajectory segment
                Eigen::Affine3d tcp_transform = current_state.getGlobalLinkTransform(g_move_group_interface->getEndEffectorLink());
                std::vector<Eigen::Affine3d> transform_vector = getNearestTransform(transform_deque_segment, tcp_transform);
                g_nearest = transform_vector[0];
                g_forward = transform_vector[1];

                // Find relative transform to goal
                Eigen::Affine3d target_transform = waypoints_segment.back().getGlobalLinkTransform(g_move_group_interface->getEndEffectorLink());
                g_goal = getRelativeTransform(tcp_transform, target_transform);

                // Print tcp transform matrix and quaternion for debugging
                /* std::ostringstream oss;
                oss << tcp_transform.matrix();
                ROS_INFO_STREAM("\n" + oss.str());
                Eigen::Matrix3d tcp_rotation_matrix = tcp_transform.rotation();
                Eigen::Quaterniond quaternion(tcp_rotation_matrix);
                std::ostringstream ss;
                ss << quaternion.coeffs().transpose();
                ROS_INFO_STREAM("\n" + ss.str()); */

                // Publish and visualize
                publishVectors(g_nearest.translation(), g_forward.translation(), g_goal.translation());
                publishTrajectory(transform_deque);
                visualizeTrajectory(transform_deque, current_state);

                loop_rate.sleep();
            }
        }

        // Generate target poses for planning
        geometry_msgs::PoseStamped getPose(double x=0, double y=0, double z=0, 
                                double w=0, double rx=0, double ry=0, double rz=0)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = w;
            pose.pose.orientation.x = rx;
            pose.pose.orientation.y = ry;
            pose.pose.orientation.z = rz;

            return pose;
        }

        // Get waypoints from plan
        std::vector<moveit::core::RobotState> getWaypoints(planning_interface::MotionPlanResponse& plan) // returns a std::deque< robot_state::RobotStatePtr >
        {
            std::vector<moveit::core::RobotState> waypoints;
            for (int i = 0; i < plan.trajectory_->getWayPointCount(); i++)
            {
                waypoints.push_back(plan.trajectory_->getWayPoint(i));
            }
            return waypoints;
        }

        // Get waypoints from trajectory
        std::vector<moveit::core::RobotState> getWaypoints(robot_trajectory::RobotTrajectory trajectory) // returns a std::deque< robot_state::RobotStatePtr >
        {   
            std::vector<moveit::core::RobotState> waypoints;
            for (int i = 0; i < trajectory.getWayPointCount(); i++)
            {
                waypoints.push_back(trajectory.getWayPoint(i));
            }
            return waypoints;
        }

        // Get global transforms of target link from plan
        std::deque<Eigen::Affine3d> getTransforms(std::vector<moveit::core::RobotState> waypoints, std::string link = "tcp_link")
        {
            std::deque<Eigen::Affine3d> transform_deque;
            for (auto robot_state : waypoints) // robot_state::RobotStatePtr
            {
                Eigen::Affine3d transform;
                bool success = false;
                while (!success)
                {
                    try
                    {
                        transform = robot_state.getGlobalLinkTransform(link); // moveit::core::RobotState
                        success = true;
                    }
                    catch (...)
                    {
                        continue;
                    }
                }
                transform_deque.push_back(transform);
            }
            return transform_deque;
        }

        // Get global transforms of target link from plan
        std::deque<Eigen::Affine3d> getTransforms(planning_interface::MotionPlanResponse plan, std::string link = "tcp_link")
        {
            auto waypoints = getWaypoints(plan); // std::deque< robot_state::RobotStatePtr >

            std::deque<Eigen::Affine3d> transform_deque;
            for (auto robot_state : waypoints) // robot_state::RobotStatePtr
            {
                Eigen::Affine3d transform = robot_state.getGlobalLinkTransform(link); // moveit::core::RobotState
                transform_deque.push_back(transform);
            }
            return transform_deque;
        }

        // Calculate transform on trajectory transform_deque nearest to a transform P
        // Return a pair of the transform and its translational distance from P
        std::vector<Eigen::Affine3d> getNearestTransform(std::deque<Eigen::Affine3d> transform_deque, Eigen::Affine3d P)
        {
            // Instantiate loop variables
            int best_index;
            double min_distance = std::numeric_limits<double>::infinity();
            double interpolation_param_nearest;
            Eigen::Affine3d A_nearest;
            Eigen::Affine3d B_nearest;
            // Loop over transform_deque to find where the trajectory is nearest to P
            for (int i = 0; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, trying every consecutive segment AB
                Eigen::Affine3d A_loop = transform_deque[i];
                // Account for size == 0
                if (i > 0)
                {
                    A_loop = transform_deque[i-1];
                }
                Eigen::Affine3d B_loop = transform_deque[i];

                // Find where this segment AB is nearest to P
                std::pair<Eigen::Vector3d, double> pair = getNearestTranslation(A_loop, B_loop, P);
                double this_distance = pair.first.norm();
                // If closest segment yet, replace best
                if (this_distance < min_distance)
                {
                    best_index = i;
                    min_distance = this_distance;
                    interpolation_param_nearest = pair.second;
                    A_nearest = A_loop;
                    B_nearest = B_loop;
                }
            }

            // Interpolate nearest_global, i.e. the global transform between A and B which comes closest to P (including rotations)
            Eigen::Affine3d nearest_global = getInterpolatedTransform(A_nearest, B_nearest, interpolation_param_nearest);
            // Find the relative translation from P to nearest_global
            Eigen::Affine3d nearest_relative = getRelativeTransform(P, nearest_global);

            // Find the vector from P to a point forward of nearest_global by 1 second at the target velocity
            Eigen::Affine3d forward;
            // Loop over the trajectory waypoints until a point far enough forward is found
            double dt = 1; // 1 second
            double velocity = g_velocity_vector[0]; // Just use the first velocity given. TODO: account for trajectory segments with different target velocities
            double target_displacement = (A_nearest.inverse() * B_nearest).translation().norm() * interpolation_param_nearest; // Include an offset to match the nearest point's interpolated displacement along the trajectory
            target_displacement += dt * velocity; // Add on the desired forward motion
            double displacement = 0; // Count cumulative displacement
            double interpolation_param_forward;
            Eigen::Affine3d A_forward;
            Eigen::Affine3d B_forward;
            for (int i = best_index; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, trying every consecutive segment AB starting from A_nearest
                Eigen::Affine3d A_loop = transform_deque[i];
                // Account for size == 0
                if (i > 0)
                {
                    A_loop = transform_deque[i-1];
                }
                Eigen::Affine3d B_loop = transform_deque[i];

                // Check if this segment adds enough displacement or not
                double AB_displacement = (A_loop.inverse() * B_loop).translation().norm(); // Get norm between pos A and pos B
                AB_displacement = std::max(AB_displacement, 0.000001); // Prevent divide by 0
                interpolation_param_forward = (target_displacement - displacement) / AB_displacement;
                // Set (A_forward, B_forward) since they will always be the last (A_loop, B_loop) touched. They are only separate variables for consistency with (A_nearest, B_nearest) and to persist outside the loop.
                A_forward = A_loop;
                B_forward = B_loop;
                if (interpolation_param_forward <= 1) // Exit condition: interpolation param <= 1, target displacement reached
                {
                    break;
                }
                else // Continue condition: interpolation param > 1, set it to 1 in case loop ends here
                {
                    interpolation_param_forward = 1;
                }
                displacement += AB_displacement; // Add this segment to total displacement for next loop
            }

            // Interpolate forward_global, i.e. the global transform further along the trajectory from nearest_global by a distance (dt * v) 
            Eigen::Affine3d forward_global = getInterpolatedTransform(A_forward, B_forward, interpolation_param_forward);
            // Find the relative translation from P to forward_global
            Eigen::Affine3d forward_relative = getRelativeTransform(P, forward_global);

            // Package result into vector
            std::vector<Eigen::Affine3d> out;
            out.push_back(nearest_relative);
            out.push_back(forward_relative);
            return out;
        }

        std::pair<Eigen::Vector3d, double> getNearestTranslation(Eigen::Affine3d A, Eigen::Affine3d B, Eigen::Affine3d P)
        {
            // Use translations for calculating nearest point
            Eigen::Vector3d At = A.translation();
            Eigen::Vector3d Bt = B.translation();
            Eigen::Vector3d Pt = P.translation();
            // Calculate relative translations
            Eigen::Vector3d ABt = Bt - At;
            Eigen::Vector3d APt = Pt - At;
            double interpolation_param = APt.dot(ABt) / ABt.squaredNorm();
            // Clamp interpolation to the bounds of the line segment
            if (interpolation_param < 0)
            {
                interpolation_param = 0;
            }
            else if (interpolation_param > 1)
            {
                interpolation_param = 1;
            }

            Eigen::Vector3d translation = (interpolation_param * ABt + At) - Pt;
            std::pair<Eigen::Vector3d, double> out;
            out.first = translation;
            out.second = interpolation_param;
            return out;
        }

        Eigen::Affine3d getInterpolatedTransform(Eigen::Affine3d A, Eigen::Affine3d B, double interpolation_param)
        {
            Eigen::Affine3d interp;
            // Use translations for calculating the interpolated translation
            Eigen::Vector3d At(A.translation());
            Eigen::Vector3d Bt(B.translation());
            interp.translation() = interpolation_param * Bt + (1 - interpolation_param) * At;
            // Use Eigen's slerp for calculating the interpolated rotation
            Eigen::Quaterniond Ar(A.linear());
            Eigen::Quaterniond Br(B.linear());
            interp.linear() = Ar.slerp(interpolation_param, Br).toRotationMatrix();
            return interp;
        }

        Eigen::Affine3d getRelativeTransform(Eigen::Affine3d A, Eigen::Affine3d B)
        {
            Eigen::Affine3d R;
            // Use translations for calculating difference of translations
            Eigen::Vector3d At(A.translation());
            Eigen::Vector3d Bt(B.translation());
            R.translation() = Bt - At;
            // Invert to calculate relative rotation, since RA = B --> R = B(A^-1)
            Eigen::Quaterniond Ar(A.linear());
            Eigen::Quaterniond Br(B.linear());
            R.linear() = (Br * (Ar.inverse())).toRotationMatrix();
            return R;
        }

        void publishVectors(Eigen::Vector3d nearest, Eigen::Vector3d forward, Eigen::Vector3d goal)
        {
            // Calculate euclidean norm of nearest
            double nearest_dot_product = 0;
            for (int i = 0; i < nearest.size(); i++)
            {
                nearest_dot_product += nearest[i] * nearest[i];
            }
            double nearest_distance = std::sqrt(nearest_dot_product);
            std_msgs::Float64 nearest_distance_msg;
            nearest_distance_msg.data = nearest_distance;
            // Calculate normalized vector of nearest
            geometry_msgs::Vector3 nearest_direction_msg;
            nearest_direction_msg.x = nearest[0] / nearest_distance;
            nearest_direction_msg.y = nearest[1] / nearest_distance;
            nearest_direction_msg.z = nearest[2] / nearest_distance;

            // Calculate euclidean norm of forward
            double forward_dot_product = 0;
            for (int i = 0; i < forward.size(); i++)
            {
                forward_dot_product += forward[i] * forward[i];
            }
            double forward_distance = std::sqrt(forward_dot_product);
            std_msgs::Float64 forward_distance_msg;
            forward_distance_msg.data = forward_distance;
            // Calculate normalized vector of forward
            geometry_msgs::Vector3 forward_direction_msg;
            forward_direction_msg.x = forward[0] / forward_distance;
            forward_direction_msg.y = forward[1] / forward_distance;
            forward_direction_msg.z = forward[2] / forward_distance;

            // Calculate euclidean norm of goal (relative to end effector)
            double goal_dot_product = 0;
            for (int i = 0; i < goal.size(); i++)
            {
                goal_dot_product += goal[i] * goal[i];
            }
            double goal_distance = std::sqrt(goal_dot_product);
            std_msgs::Float64 goal_distance_msg;
            goal_distance_msg.data = goal_distance;
            // Calculate normalized vector of goal (relative to end effector)
            geometry_msgs::Vector3 goal_direction_msg;
            goal_direction_msg.x = goal[0] / goal_distance;
            goal_direction_msg.y = goal[1] / goal_distance;
            goal_direction_msg.z = goal[2] / goal_distance;
            geometry_msgs::Vector3 goal_vector_msg; //For the directional distance
            goal_vector_msg.x = goal[0]; //For the directional distance
            goal_vector_msg.y = goal[1]; //For the directional distance
            goal_vector_msg.z = goal[2]; //For the directional distance   
            // Publish
            g_nearest_distance_publisher.publish(nearest_distance_msg);
            g_nearest_direction_publisher.publish(nearest_direction_msg);
            g_forward_distance_publisher.publish(forward_distance_msg);
            g_forward_direction_publisher.publish(forward_direction_msg);
            g_goal_distance_publisher.publish(goal_distance_msg);
            g_goal_direction_publisher.publish(goal_direction_msg);
            g_goal_vector_publisher.publish(goal_vector_msg); 
        }

        void publishTrajectory(std::deque<Eigen::Affine3d> transform_deque)
        {
            // Convert transform deque to string
            std::stringstream ss;
            for (Eigen::Affine3d transform : transform_deque)
            {
                Eigen::Vector3d translation = transform.translation();
                ss << '[';
                ss << translation[0] << ", ";
                ss << translation[1] << ", ";
                ss << translation[2];
                ss << ']';
            }
            std_msgs::String trajectory_msg;
            trajectory_msg.data = ss.str();

            // Publish
            g_trajectory_publisher.publish(trajectory_msg);
        }

        void publishMarkers(visualization_msgs::MarkerArray& markers)
        {
            // delete old markers
            if (!g_trajectory_marker_array.markers.empty())
            {
                for (auto& marker : g_trajectory_marker_array.markers)
                marker.action = visualization_msgs::Marker::DELETE;

                g_marker_array_publisher.publish(g_trajectory_marker_array);
            }

            // move new markers into g_trajectory_marker_array
            std::swap(g_trajectory_marker_array.markers, markers.markers);

            // draw new markers (if there are any)
            if (!g_trajectory_marker_array.markers.empty())
                g_marker_array_publisher.publish(g_trajectory_marker_array);
        }

        void visualizeTrajectory(std::deque<Eigen::Affine3d> transform_deque, moveit::core::RobotState robot_state)
        {
            //// Visualize the Trajectory itself /////

            // Set a color for the visualization markers
            std_msgs::ColorRGBA traj_color;
            traj_color.r = 0.0;
            traj_color.g = 1.0;
            traj_color.b = 0.0;
            traj_color.a = 0.5;

            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::MarkerArray markers;
            std::map<std::string, unsigned> ns_counts;

            // Loop over transform_deque to visualize each segment on the trajectory
            for (int i = 0; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, visualizing every consecutive segment AB
                Eigen::Affine3d transform_A = transform_deque[i];
                if (i > 0)
                {
                    transform_A = transform_deque[i-1];
                }
                Eigen::Vector3d translation_A = transform_A.translation();
                geometry_msgs::Point point_A;
                point_A.x = translation_A[0];
                point_A.y = translation_A[1];
                point_A.z = translation_A[2];
                Eigen::Affine3d transform_B = transform_deque[i];
                Eigen::Vector3d translation_B = transform_B.translation();
                geometry_msgs::Point point_B;
                point_B.x = translation_B[0];
                point_B.y = translation_B[1];
                point_B.z = translation_B[2];

                // If this is the target segment, increase the alpha
                if (i == g_segment_index-1)
                {
                    traj_color.a = 0.5;
                }
                else
                {
                    traj_color.a = 0.25;
                }

                // Create a marker for this segment AB
                std::vector<geometry_msgs::Point> points; // Put points in the array type accepted by Marker
                points.push_back(point_A);
                points.push_back(point_B);
                
                std::stringstream ss;
                ss << (i - 1) << "=" << i;
                std::string ns_name = ss.str(); // String name
                if (ns_counts.find(ns_name) == ns_counts.end())
                    ns_counts[ns_name] = 0;
                else
                    ns_counts[ns_name]++;
                visualization_msgs::Marker mk_traj; // Instantiate marker
                mk_traj.header.stamp = ros::Time::now(); // Timestamp
                mk_traj.header.frame_id = "world"; // Reference frame id
                mk_traj.ns = ns_name; // String name
                mk_traj.id = ns_counts[ns_name]; // Unique number id
                mk_traj.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
                mk_traj.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
                mk_traj.points = points; // Start and end points of arrow
                mk_traj.scale.x = 0.005; // Arrow shaft diameter
                mk_traj.scale.y = 0.015; // Arrow head diameter
                mk_traj.scale.z = 0.015; // Arrow head length
                mk_traj.color = traj_color; // Color specified above
                mk_traj.lifetime = ros::Duration(1); // Remain for 1 second or until updated
                markers.markers.push_back(mk_traj); // Add to MarkerArray markers
            }

            //// Visualize the tcp -> Trajectory transform
            
            // Set a different color for the tcp -> Trajectory transform
            std_msgs::ColorRGBA tcp_color;
            tcp_color.r = 0.0;
            tcp_color.g = 0.0;
            tcp_color.b = 1.0;
            tcp_color.a = 0.5;

            // Add a marker for the tcp -> nearest transform
            Eigen::Vector3d tcp_translation = robot_state.getGlobalLinkTransform(g_move_group_interface->getEndEffectorLink()).translation();
            Eigen::Vector3d nearest_translation = g_nearest.translation();
            geometry_msgs::Point tcp_point;
            tcp_point.x = tcp_translation[0];
            tcp_point.y = tcp_translation[1];
            tcp_point.z = tcp_translation[2];
            geometry_msgs::Point nearest_point;
            nearest_point.x = tcp_translation[0] + nearest_translation[0];
            nearest_point.y = tcp_translation[1] + nearest_translation[1];
            nearest_point.z = tcp_translation[2] + nearest_translation[2];

            std::vector<geometry_msgs::Point> points;
            points.push_back(tcp_point);
            points.push_back(nearest_point);

            std::string ns_name = "nearest_marker"; // String name
            if (ns_counts.find(ns_name) == ns_counts.end())
                ns_counts[ns_name] = 0;
            else
                ns_counts[ns_name]++;
            visualization_msgs::Marker mk_nearest; // Instantiate marker
            mk_nearest.header.stamp = ros::Time::now(); // Timestamp
            mk_nearest.header.frame_id = "world"; // Reference frame id
            mk_nearest.ns = ns_name; // String name
            mk_nearest.id = ns_counts[ns_name]; // Unique number id
            mk_nearest.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
            mk_nearest.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
            mk_nearest.points = points; // Start and end points of arrow
            mk_nearest.scale.x = 0.01; // Arrow shaft diameter
            mk_nearest.scale.y = 0.015; // Arrow head diameter
            mk_nearest.scale.z = 0.015; // Arrow head length
            mk_nearest.color = tcp_color; // Color specified above
            mk_nearest.lifetime = ros::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk_nearest); // Add to MarkerArray markers

            //// Visualize the tcp --> forward transform
            
            // Set a different color for the tcp --> forward transform
            std_msgs::ColorRGBA forward_color;
            forward_color.r = 1.0;
            forward_color.g = 0.0;
            forward_color.b = 0.0;
            forward_color.a = 0.5;

            // Add a marker for the tcp --> forward transform
            Eigen::Vector3d forward_translation = g_forward.translation();
            geometry_msgs::Point forward_point;
            forward_point.x = tcp_translation[0] + forward_translation[0];
            forward_point.y = tcp_translation[1] + forward_translation[1];
            forward_point.z = tcp_translation[2] + forward_translation[2];

            std::vector<geometry_msgs::Point> points_forward;
            points_forward.push_back(tcp_point);
            points_forward.push_back(forward_point);

            std::string ns_name_forward = "forward_marker"; // String name
            if (ns_counts.find(ns_name_forward) == ns_counts.end())
                ns_counts[ns_name_forward] = 0;
            else
                ns_counts[ns_name_forward]++;
            visualization_msgs::Marker mk_forward; // Instantiate marker
            mk_forward.header.stamp = ros::Time::now(); // Timestamp
            mk_forward.header.frame_id = "world"; // Reference frame id
            mk_forward.ns = ns_name_forward; // String name
            mk_forward.id = ns_counts[ns_name_forward]; // Unique number id
            mk_forward.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
            mk_forward.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
            mk_forward.points = points_forward; // Start and end points of arrow
            mk_forward.scale.x = 0.01; // Arrow shaft diameter
            mk_forward.scale.y = 0.015; // Arrow head diameter
            mk_forward.scale.z = 0.015; // Arrow head length
            mk_forward.color = forward_color; // Color specified above
            mk_forward.lifetime = ros::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk_forward); // Add to MarkerArray markers

            //// Visualize the tcp --> goal transform
            
            // Set a different color for the tcp --> goal transform
            std_msgs::ColorRGBA goal_color;
            goal_color.r = 1.0;
            goal_color.g = 1.0;
            goal_color.b = 1.0;
            goal_color.a = 0.5;

            // Add a marker for the tcp --> goal transform
            Eigen::Vector3d goal_translation = g_goal.translation();
            geometry_msgs::Point goal_point;
            goal_point.x = tcp_translation[0] + goal_translation[0];
            goal_point.y = tcp_translation[1] + goal_translation[1];
            goal_point.z = tcp_translation[2] + goal_translation[2];

            std::vector<geometry_msgs::Point> points_goal;
            points_goal.push_back(tcp_point);
            points_goal.push_back(goal_point);

            std::string ns_name_goal = "goal_marker"; // String name
            if (ns_counts.find(ns_name_goal) == ns_counts.end())
                ns_counts[ns_name_goal] = 0;
            else
                ns_counts[ns_name_goal]++;
            visualization_msgs::Marker mk_goal; // Instantiate marker
            mk_goal.header.stamp = ros::Time::now(); // Timestamp
            mk_goal.header.frame_id = "world"; // Reference frame id
            mk_goal.ns = ns_name_goal; // String name
            mk_goal.id = ns_counts[ns_name_goal]; // Unique number id
            mk_goal.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
            mk_goal.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
            mk_goal.points = points_goal; // Start and end points of arrow
            mk_goal.scale.x = 0.01; // Arrow shaft diameter
            mk_goal.scale.y = 0.015; // Arrow head diameter
            mk_goal.scale.z = 0.015; // Arrow head length
            mk_goal.color = goal_color; // Color specified above
            mk_goal.lifetime = ros::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk_goal); // Add to MarkerArray markers

            //// Update markers to be published ////
            publishMarkers(markers);
        }

        std::vector<std::string> splitString(std::string string, char delimiter=' ')
        {
            // Split string into string vector by delimiter
            std::vector<std::string> tokens;
            std::stringstream ss;
            ss << string;
            std::string token;
            while (std::getline(ss, token, delimiter)) {
                tokens.push_back(token);
            }
            return tokens;
        }
        
    private:
        // Define a callback to be called when the PlanningSceneMonitor receives an update
        void planningSceneMonitorCallback(const moveit_msgs::PlanningScene::ConstPtr& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
        {
            ROS_INFO("Updating...");
        }

        // Define a callback to use joystick inputs to control trajectory segment selection
        void joyCallback(const sensor_msgs::Joy& msg)
        {
            // If left stick is pressed, decrement trajectory segment and lock
            if (msg.buttons.at(g_cntlr["FLIP_EE_X"]) == 1 && g_segment_swapped == false)
            {
                int new_index = (g_segment_index - 1);
                while (new_index < 0)
                {
                    new_index += static_cast<int>(g_goal_name_vector.size() - 1);
                }
                g_segment_index = new_index;
                g_segment_swapped = true;
            }
            // Else if right stick is pressed, increment trajectory segment and lock
            else if (msg.buttons.at(g_cntlr["FLIP_EE_ROLL"]) == 1 && g_segment_swapped == false)
            {
                int new_index = (g_segment_index + 1) % (static_cast<int>(g_goal_name_vector.size() - 1));
                g_segment_index = new_index;
                g_segment_swapped = true;
            }
            // Else if neither stick is pressed, unlock
            else if (msg.buttons.at(g_cntlr["FLIP_EE_X"]) == 0 && 
                    msg.buttons.at(g_cntlr["FLIP_EE_ROLL"]) == 0 && 
                    g_segment_swapped == true)
            {
                g_segment_swapped = false;
            }
        }

        // Define a callback to use GELLO controller inputs to control trajectory segment selection
        void gelloCallback(const sensor_msgs::JointState& msg)
        {
            // If GELLO gripper is closed, increment trajectory segment and lock
            if (msg.position.back() > 0.95 && g_segment_swapped == false)
            {
                g_segment_index = (g_segment_index + 1) % static_cast<int>(g_goal_name_vector.size());
                g_segment_swapped = true;
            }
            // Else if right stick is pressed, increment trajectory segment and lock
            else if (msg.position.back() > 0.95 && g_segment_swapped == false)
            {
                g_segment_index = (g_segment_index + 1) % static_cast<int>(g_goal_name_vector.size());
                g_segment_swapped = true;
            }
            // Else if neither stick is pressed, unlock
            else if (msg.position.back() <= 0.95 &&
                    g_segment_swapped == true)
            {
                g_segment_swapped = false;
            }
        }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory");
    TrajectoryNode trajectory_node(argc, argv);
    // trajectory_node.spin();
    return 0;
}