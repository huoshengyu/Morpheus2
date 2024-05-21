#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

static const std::vector<std::string> GOAL_NAME_VECTOR_DEFAULT
{
    "keyhole_pickup", // name of the scene associated with the goal pose
    "keyhole_start",
    "keyhole_end"
};

namespace trajectory
{

};

class TrajectoryNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;

        ros::Publisher g_marker_array_publisher;
        ros::Publisher g_trajectory_publisher;
        ros::Publisher g_nearest_distance_publisher;
        ros::Publisher g_nearest_direction_publisher;
        ros::Publisher g_forward_distance_publisher;
        ros::Publisher g_forward_direction_publisher;
        visualization_msgs::msg::MarkerArray g_trajectory_marker_array;
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

        // std::shared_ptr<moveit_cpp::PlanningComponent> g_planning_components;
        // std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters> g_plan_request_parameters;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_move_group_interface;
        moveit::planning_interface::PlanningSceneInterface g_planning_scene_interface;
        // moveit::planning_interface::MoveGroupInterface::Plan g_plan;
        std::vector<robot_trajectory::RobotTrajectory> g_trajectory_vector;
        Eigen::Affine3d g_nearest;
        Eigen::Affine3d g_forward;
        std::vector<geometry_msgs::msg::PoseStamped> g_target_vector;
        std::vector<geometry_msgs::msg::PoseStamped> g_weight_vector;
        std::vector<moveit_msgs::msg::OrientationConstraint> g_constraint_vector;
        std::vector<double> g_velocity_vector;

        std::vector<std::string> g_goal_name_vector;

        TrajectoryNode(int argc, char** argv)
        {
            // Initialize ROS node
            rclcpp::Node nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();

            // Joints to plan for, from srdf file
            static const std::string PLANNING_GROUP = "arm";

            // Retrieve preexisting PlanningSceneMonitor, if possible
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);
            
            // Instantiate a move group interface so a trajectory can be generated
            g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene"))
            {
                RCLCPP_INFO(rclcpp::get_logger("MorpheusTrajectory"), "Planning Scene Monitor is active and ready.");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("MorpheusTrajectory"), "Failed to set up Planning Scene Monitor.");
            }

            try
            {   
                // Change the PlanningScene's collision detector to Bullet
                // Bullet supports distance vectors, as well as distances to multiple obstacles
                g_planning_scene_monitor->getPlanningScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), 
                                                    true /* exclusive */);
                
                if (strcmp((g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName()).c_str(), "Bullet") == 0)
                {
                    RCLCPP_INFO(rclcpp::get_logger("MorpheusTrajectory"), "Planning Scene is active and ready.");
                }    
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("MorpheusTrajectory"), "Collision detector incorrect");
                    std::string collision_detector_name = g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName();
                    throw collision_detector_name;
                }
            }
            catch (std::string collision_detector_name)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MorpheusTrajectory"), "Failed to retrieve PlanningScene.");
            }
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene"); // Get scene updates from topic
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("/joint_states");

            // Create trajectory msg publisher
            g_trajectory_publisher = nh.advertise<std_msgs::msg::String>("trajectory/msg", 0);
            
            // Create guidance vector publishers
            g_nearest_distance_publisher = nh.advertise<std_msgs::msg::Float64>("trajectory/nearest/distance", 0);
            g_nearest_direction_publisher = nh.advertise<geometry_msgs::msg::Vector3>("trajectory/nearest/direction", 0);
            g_forward_distance_publisher = nh.advertise<std_msgs::msg::Float64>("trajectory/forward/distance", 0);
            g_forward_direction_publisher = nh.advertise<geometry_msgs::msg::Vector3>("trajectory/forward/direction", 0);

            // Create a marker array publisher for publishing shapes to Rviz
            g_marker_array_publisher = nh.advertise<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 100);

            // Get target vector from ros server, if possible
            if (ros::param::get("/goal/name_vector", g_goal_name_vector))
            {
                RCLCPP_INFO(rclcpp::get_logger("MorpheusTrajectory"), "Using GOAL_NAME_VECTOR from parameter server");
            }
            else
            {
                g_goal_name_vector = GOAL_NAME_VECTOR_DEFAULT;
                RCLCPP_INFO(rclcpp::get_logger("MorpheusTrajectory"), "Using GOAL_NAME_VECTOR_DEFAULT");
            }

            // Get goal transforms and associated parameters from parameter server
            for (std::string goal_name : g_goal_name_vector)
            {
                if (ros::param::has("/goal/" + goal_name))
                {
                    // Get goal transform from parameter server
                    geometry_msgs::msg::PoseStamped target;
                    target.header.stamp = rclcpp::Time::now();
                    target.header.frame_id = "world";
                    ros::param::get("/goal/" + goal_name + "/goal_pose/position/x", target.pose.position.x);
                    ros::param::get("/goal/" + goal_name + "/goal_pose/position/y", target.pose.position.y);
                    ros::param::get("/goal/" + goal_name + "/goal_pose/position/z", target.pose.position.z);
                    ros::param::get("/goal/" + goal_name + "/goal_pose/orientation/x", target.pose.orientation.x);
                    ros::param::get("/goal/" + goal_name + "/goal_pose/orientation/y", target.pose.orientation.y);
                    ros::param::get("/goal/" + goal_name + "/goal_pose/orientation/z", target.pose.orientation.z);
                    ros::param::get("/goal/" + goal_name + "/goal_pose/orientation/w", target.pose.orientation.w);
                    g_target_vector.push_back(target);

                    // Get goal weights from parameter server
                    geometry_msgs::msg::PoseStamped weights;
                    target.header.stamp = rclcpp::Time::now();
                    target.header.frame_id = "world";
                    ros::param::get("/goal/" + goal_name + "/position_weights/x", weights.pose.position.x);
                    ros::param::get("/goal/" + goal_name + "/position_weights/y", weights.pose.position.y);
                    ros::param::get("/goal/" + goal_name + "/position_weights/z", weights.pose.position.z);
                    ros::param::get("/goal/" + goal_name + "/orientation_weights/x", weights.pose.orientation.x);
                    ros::param::get("/goal/" + goal_name + "/orientation_weights/y", weights.pose.orientation.y);
                    ros::param::get("/goal/" + goal_name + "/orientation_weights/z", weights.pose.orientation.z);
                    ros::param::get("/goal/" + goal_name + "/orientation_weights/w", weights.pose.orientation.w);
                    g_weight_vector.push_back(weights);

                    // Get max velocity scaling factor from parameter server
                    double velocity;
                    ros::param::get("/goal/" + goal_name + "/max_velocity_scaling_factor", velocity);
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
                geometry_msgs::msg::PoseStamped target;
                target.header.stamp = rclcpp::Time::now();
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
                geometry_msgs::msg::PoseStamped weights;
                weights.header.stamp = rclcpp::Time::now();
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
                moveit_msgs::msg::OrientationConstraint constraint;
                constraint.link_name = "tcp_link";
                constraint.header.frame_id = "world";
                constraint.orientation.w = -0.5;
                constraint.absolute_x_axis_tolerance = 0.1;
                constraint.absolute_y_axis_tolerance = 0.1;
                constraint.absolute_z_axis_tolerance = 0.1;
                constraint.weight = 1.0;
                g_constraint_vector.push_back(constraint);

                ROS_INFO_STREAM("Constraint vector empty, using default goal constraints");
            }

            // Get robot model from the current planning scene
            const robot_model::RobotModelConstPtr robot_model = g_planning_scene_monitor->getRobotModel();

            // Get the current robot state once so that it does not vary over time
            // A LockedPlanningSceneRO is used to avoid modifying the planning scene.
            // Alternatively, the robot state can be set from a given joint state or from a given pose (using IK)
            robot_state::RobotStatePtr robot_state(
                new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState()));

            // Create a joint model group for tracking the current robot pose and planning group
            const moveit::core::JointModelGroup* joint_model_group =
                robot_state->getJointModelGroup(PLANNING_GROUP);
            
            ROS_INFO_STREAM(g_move_group_interface->getDefaultPlannerId());
            ROS_INFO_STREAM(g_move_group_interface->getPlannerId());

            // Iterate over all goal poses
            robot_state::RobotState next_start_state = *robot_state;
            for (int i = 0; i < g_target_vector.size(); i++)
            {
                ROS_INFO_STREAM("Starting loop");
                // Set planning parameters
                g_move_group_interface->setPlanningTime(60); // time in seconds before timeout
                g_move_group_interface->setPoseTarget(g_target_vector[i], "tcp_link"); // end effector pose
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

            // Instantiate visual tools for visualizing markers in Rviz
            visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("/world", "visualization_marker_array", g_planning_scene_monitor);

            spin();
            // ros::shutdown();
        }

        void spin()
        {
            ROS_INFO_STREAM("Starting spin()");
            // Loop collision requests and publish at specified rate
            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok())
            {   
                // Retrieve and update state once. Avoid updating in functions below to maintain sync.
                auto current_state = g_planning_scene_monitor->getPlanningScene()->getCurrentState();
                current_state.updateLinkTransforms();

                // Plan trajectory
                // auto plan = g_planning_components->getLastMotionPlanResponse();
                std::vector<moveit::core::RobotState> waypoints;
                for (robot_trajectory::RobotTrajectory trajectory : g_trajectory_vector)
                {
                    std::vector<moveit::core::RobotState> traj_waypoints = getWaypoints(trajectory); // Get next set of waypoints
                    waypoints.insert(waypoints.end(), traj_waypoints.begin(), traj_waypoints.end()); // Concatenate
                }
                auto transform_deque = getTransforms(waypoints);

                // Find relative transforms to nearest and forward sections of trajectory
                Eigen::Affine3d tcp_transform = current_state.getGlobalLinkTransform("tcp_link");
                std::vector<Eigen::Affine3d> transform_vector = getNearestTransform(transform_deque, tcp_transform);
                g_nearest = transform_vector[0];
                g_forward = transform_vector[1];

                // Publish and visualize
                publishVectors(g_nearest.translation(), g_forward.translation());
                publishTrajectory(transform_deque);
                visualizeTrajectory(transform_deque);

                loop_rate.sleep();
            }

            // Spin the ROS node
            rclcpp::spin(node);
        }

        // Generate target poses for planning
        geometry_msgs::msg::PoseStamped getPose(double x=0, double y=0, double z=0, 
                                double w=0, double rx=0, double ry=0, double rz=0)
        {
            geometry_msgs::msg::PoseStamped pose;
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
            Eigen::Vector3d translation;
            double interpolation_param;
            Eigen::Affine3d A;
            Eigen::Affine3d B;
            // Loop over transform_deque to find where the trajectory is nearest to P
            for (int i = 1; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, trying every consecutive segment AB
                Eigen::Affine3d A_loop = transform_deque[i-1];
                Eigen::Affine3d B_loop = transform_deque[i];

                // Find where this segment AB is nearest to P
                std::pair<Eigen::Vector3d, double> pair = getNearestTranslation(A_loop, B_loop, P);
                double this_distance = pair.first.norm();
                // If closest segment yet, replace best
                if (this_distance < min_distance)
                {
                    best_index = i;
                    min_distance = this_distance;
                    translation = pair.first;
                    interpolation_param = pair.second;
                    A = A_loop;
                    B = B_loop;
                }
            }

            // Include rotations when calcuating the full interpolated transform
            Eigen::Affine3d ABinterp = getInterpolatedTransform(A, B, interpolation_param);
            // Find the relative translation from P to ABinterp
            Eigen::Affine3d nearest = getRelativeTransform(P, ABinterp);

            // Find the vector from P to a point forward of the nearest point by 1 waypoint's distance
            Eigen::Affine3d forward;
            // Get the next waypoint along the trajectory
            int C_index = best_index+1;
            if ((C_index + 1) > transform_deque.size())
            {
                C_index = best_index;
            }
            Eigen::Affine3d C = transform_deque[C_index];
            // Interpolate the forward point along the trajectory
            Eigen::Affine3d BCinterp = getInterpolatedTransform(B, C, interpolation_param);
            // Find the relative translation from P to BCinterp
            forward = getRelativeTransform(P, BCinterp);

            // Package result into vector
            std::vector<Eigen::Affine3d> out;
            out.push_back(nearest);
            out.push_back(forward);
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

        void publishVectors(Eigen::Vector3d nearest, Eigen::Vector3d forward)
        {
            // Calculate euclidean norm of nearest
            double nearest_dot_product = 0;
            for (int i = 0; i < nearest.size(); i++)
            {
                nearest_dot_product += nearest[i] * nearest[i];
            }
            double nearest_distance = std::sqrt(nearest_dot_product);
            std_msgs::msg::Float64 nearest_distance_msg;
            nearest_distance_msg.data = nearest_distance;
            // Calculate normalized vector of nearest
            geometry_msgs::msg::Vector3 nearest_direction_msg;
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
            std_msgs::msg::Float64 forward_distance_msg;
            forward_distance_msg.data = forward_distance;
            // Calculate normalized vector of forward
            geometry_msgs::msg::Vector3 forward_direction_msg;
            forward_direction_msg.x = forward[0] / forward_distance;
            forward_direction_msg.y = forward[1] / forward_distance;
            forward_direction_msg.z = forward[2] / forward_distance;

            // Publish
            g_nearest_distance_publisher.publish(nearest_distance_msg);
            g_nearest_direction_publisher.publish(nearest_direction_msg);
            g_forward_distance_publisher.publish(forward_distance_msg);
            g_forward_direction_publisher.publish(forward_direction_msg);
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
            std_msgs::msg::String trajectory_msg;
            trajectory_msg.data = ss.str();

            // Publish
            g_trajectory_publisher.publish(trajectory_msg);
        }

        void publishMarkers(visualization_msgs::msg::MarkerArray& markers)
        {
            // delete old markers
            if (!g_trajectory_marker_array.markers.empty())
            {
                for (auto& marker : g_trajectory_marker_array.markers)
                marker.action = visualization_msgs::msg::Marker::DELETE;

                g_marker_array_publisher.publish(g_trajectory_marker_array);
            }

            // move new markers into g_trajectory_marker_array
            std::swap(g_trajectory_marker_array.markers, markers.markers);

            // draw new markers (if there are any)
            if (!g_trajectory_marker_array.markers.empty())
                g_marker_array_publisher.publish(g_trajectory_marker_array);
        }

        void visualizeTrajectory(std::deque<Eigen::Affine3d> transform_deque)
        {
            //// Visualize the Trajectory itself /////

            // Set a color for the visualization markers
            std_msgs::msg::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 0.5;

            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::msg::MarkerArray markers;
            std::map<std::string, unsigned> ns_counts;

            // Loop over transform_deque to visualize each segment on the trajectory
            for (int i = 1; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, visualizing every consecutive segment AB
                Eigen::Affine3d transform_A = transform_deque[i-1];
                Eigen::Vector3d translation_A = transform_A.translation();
                geometry_msgs::msg::Point point_A;
                point_A.x = translation_A[0];
                point_A.y = translation_A[1];
                point_A.z = translation_A[2];
                Eigen::Affine3d transform_B = transform_deque[i];
                Eigen::Vector3d translation_B = transform_B.translation();
                geometry_msgs::msg::Point point_B;
                point_B.x = translation_B[0];
                point_B.y = translation_B[1];
                point_B.z = translation_B[2];

                // Create a marker for this segment AB
                std::vector<geometry_msgs::msg::Point> points; // Put points in the array type accepted by Marker
                points.push_back(point_A);
                points.push_back(point_B);
                
                std::stringstream ss;
                ss << (i - 1) << "=" << i;
                std::string ns_name = ss.str(); // String name
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
                mk.scale.x = 0.005; // Arrow shaft diameter
                mk.scale.y = 0.015; // Arrow head diameter
                mk.scale.z = 0.015; // Arrow head length
                mk.color = color; // Color specified above
                mk.lifetime = rclcpp::Duration(1); // Remain for 1 second or until updated
                markers.markers.push_back(mk); // Add to MarkerArray markers
            }

            //// Visualize the tcp -> Trajectory transform
            
            // Set a different color for the tcp -> Trajectory transform
            std_msgs::msg::ColorRGBA tcp_color;
            tcp_color.r = 0.0;
            tcp_color.g = 0.0;
            tcp_color.b = 1.0;
            tcp_color.a = 0.5;

            // Add a marker for the tcp -> Trajectory transform
            auto current_state = g_planning_scene_monitor->getPlanningScene()->getCurrentState();
            Eigen::Vector3d tcp_translation = current_state.getGlobalLinkTransform("tcp_link").translation();
            Eigen::Vector3d nearest_translation = g_nearest.translation();
            geometry_msgs::msg::Point tcp_point;
            tcp_point.x = tcp_translation[0];
            tcp_point.y = tcp_translation[1];
            tcp_point.z = tcp_translation[2];
            geometry_msgs::msg::Point nearest_point;
            nearest_point.x = tcp_translation[0] + nearest_translation[0];
            nearest_point.y = tcp_translation[1] + nearest_translation[1];
            nearest_point.z = tcp_translation[2] + nearest_translation[2];

            std::vector<geometry_msgs::msg::Point> points;
            points.push_back(tcp_point);
            points.push_back(nearest_point);

            std::string ns_name = "tcp_marker"; // String name
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
            mk.scale.y = 0.015; // Arrow head diameter
            mk.scale.z = 0.015; // Arrow head length
            mk.color = tcp_color; // Color specified above
            mk.lifetime = rclcpp::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk); // Add to MarkerArray markers

            //// Visualize the tcp --> forward transform
            
            // Set a different color for the tcp --> forward transform
            std_msgs::msg::ColorRGBA forward_color;
            forward_color.r = 1.0;
            forward_color.g = 0.0;
            forward_color.b = 0.0;
            forward_color.a = 0.5;

            // Add a marker for the tcp --> forward transform
            Eigen::Vector3d forward_translation = g_forward.translation();
            geometry_msgs::msg::Point forward_point;
            forward_point.x = tcp_translation[0] + forward_translation[0];
            forward_point.y = tcp_translation[1] + forward_translation[1];
            forward_point.z = tcp_translation[2] + forward_translation[2];

            std::vector<geometry_msgs::msg::Point> points_forward;
            points_forward.push_back(tcp_point);
            points_forward.push_back(forward_point);

            std::string ns_name_forward = "forward_marker"; // String name
            if (ns_counts.find(ns_name_forward) == ns_counts.end())
                ns_counts[ns_name_forward] = 0;
            else
                ns_counts[ns_name_forward]++;
            visualization_msgs::msg::Marker mk_forward; // Instantiate marker
            mk_forward.header.stamp = rclcpp::Time::now(); // Timestamp
            mk_forward.header.frame_id = "world"; // Reference frame id
            mk_forward.ns = ns_name_forward; // String name
            mk_forward.id = ns_counts[ns_name_forward]; // Unique number id
            mk_forward.type = visualization_msgs::msg::Marker::ARROW; // Arrow marker shape
            mk_forward.action = visualization_msgs::msg::Marker::ADD; // Add shape to Rviz
            mk_forward.points = points_forward; // Start and end points of arrow
            mk_forward.scale.x = 0.01; // Arrow shaft diameter
            mk_forward.scale.y = 0.015; // Arrow head diameter
            mk_forward.scale.z = 0.015; // Arrow head length
            mk_forward.color = forward_color; // Color specified above
            mk_forward.lifetime = rclcpp::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk_forward); // Add to MarkerArray markers


            //// Update markers to be published ////
            publishMarkers(markers);
        }
        
    private:
        // Define a callback to update to be called when the PlanningSceneMonitor receives an update
        static void planningSceneMonitorCallback(const moveit_msgs::msg::PlanningScene::ConstSharedPtr& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
        {
            RCLCPP_INFO(rclcpp::get_logger("MorpheusTrajectory"), "Updating...");
        }


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("trajectory");
    TrajectoryNode trajectory_node(argc, argv);
    trajectory_node.spin();
    return 0;
}
