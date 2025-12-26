import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import WrenchStamped, PoseStamped
from compliant_trajectory_control.cfg import Tolerance6DOFConfig, GoalTimeToleranceConfig
from compliant_trajectory_control.msg import FollowCompliantTrajectoryAction, FollowCompliantTrajectoryFeedback, FollowCompliantTrajectoryResult
from cartesian_control_msgs.msg import FollowCartesianTrajectoryGoal, CartesianTolerance, FollowCartesianTrajectoryAction
from compliant_trajectory_control.joint_to_cartesian_converter import JointToCartesianConverter
from actionlib_msgs.msg import GoalStatus
import numpy as np
from compliant_trajectory_control.utils import pose_error   

class GoalTimeToleranceServer:
    def __init__(self, ns):
        self.goal_time_tolerance = 0

        self.srv = Server(GoalTimeToleranceConfig, self.reconfigure_callback, ns)

    def reconfigure_callback(self, config, level):
        self.goal_time_tolerance = config.goal_time_tolerance
        return config

    def getGoalTimeTolerance(self):
        return self.goal_time_tolerance

class Tolerance6DOFServer:
    def __init__(self, ns):
        self.tolerance = [0] * 6

        self.srv = Server(Tolerance6DOFConfig, self.reconfigure_callback, ns)

    def reconfigure_callback(self, config, level):
        self.tolerance = [config.trans_x, config.trans_y, config.trans_z, config.rot_x, config.rot_y, config.rot_z]
        return config

    def getTolerance(self):
        return self.tolerance
    
class CartesianToleranceServer:
    def __init__(self, ns):
        self.pose_tolerance = Tolerance6DOFServer(ns + "/pose_error")
        self.twist_tolerance = Tolerance6DOFServer(ns + "/twist_error")
        self.accel_tolerance = Tolerance6DOFServer(ns + "/accel_error")

    def getTolerance(self):
        pose_tolerance = self.pose_tolerance.getTolerance()
        twist_tolerance = self.twist_tolerance.getTolerance()
        accel_tolerance = self.accel_tolerance.getTolerance()

        cartesian_tolerance = CartesianTolerance()
        cartesian_tolerance.position_error.x = pose_tolerance[0]
        cartesian_tolerance.position_error.y = pose_tolerance[1]
        cartesian_tolerance.position_error.z = pose_tolerance[2]
        cartesian_tolerance.orientation_error.x = pose_tolerance[3]
        cartesian_tolerance.orientation_error.y = pose_tolerance[4]
        cartesian_tolerance.orientation_error.z = pose_tolerance[5]
        cartesian_tolerance.twist_error.linear.x = twist_tolerance[0]
        cartesian_tolerance.twist_error.linear.y = twist_tolerance[1]
        cartesian_tolerance.twist_error.linear.z = twist_tolerance[2]
        cartesian_tolerance.twist_error.angular.x = twist_tolerance[3]
        cartesian_tolerance.twist_error.angular.y = twist_tolerance[4]
        cartesian_tolerance.twist_error.angular.z = twist_tolerance[5]
        cartesian_tolerance.acceleration_error.linear.x = accel_tolerance[0]
        cartesian_tolerance.acceleration_error.linear.y = accel_tolerance[1]
        cartesian_tolerance.acceleration_error.linear.z = accel_tolerance[2]
        cartesian_tolerance.acceleration_error.angular.x = accel_tolerance[3]
        cartesian_tolerance.acceleration_error.angular.y = accel_tolerance[4]
        cartesian_tolerance.acceleration_error.angular.z = accel_tolerance[5]

        return cartesian_tolerance
    
class FollowCompliantTrajectoryActionServer:
    def __init__(self):  

        name = rospy.get_name()    

        # Load parameters from the parameter server
        robot_description_param = rospy.search_param("robot_description")
        robot_description = rospy.get_param(robot_description_param)

        self.base_link = rospy.get_param(name + "/robot_base_link")
        self.tip_link = rospy.get_param(name + "/end_effector_link")
        self.joint_names = rospy.get_param(name + "/joints")

        self.converter = JointToCartesianConverter(robot_description, self.joint_names, self.base_link, self.tip_link)

        # Create a TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Initialize the wrench publisher
        self.wrench_publisher = rospy.Publisher(name + "/target_wrench", WrenchStamped, queue_size=1)
        self.frame_publisher = rospy.Publisher(name + "/target_frame", PoseStamped, queue_size=1)

        self.path_tolerance = CartesianToleranceServer("path_tolerance")
        self.goal_tolerance = CartesianToleranceServer("goal_tolerance")
        self.goal_time_tolerance = GoalTimeToleranceServer("")

        # Initialize the action server
        self.compliant_trajectory_action_server = actionlib.SimpleActionServer(
            name + '/follow_compliant_trajectory',
            FollowCompliantTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.compliant_trajectory_action_server.start()

    def execute_cb(self, goal):
        # retrieve values
        cs_x, cs_y, cs_z, s_rot, duration = self.converter.joint_traj_to_cart_spline(goal.joint_trajectory)
        wrench_stamped = goal.wrench

        if goal.override_default_tolerances:
            path_tolerance = goal.path_tolerance
            goal_tolerance = goal.goal_tolerance
            goal_time_tolerance = goal.goal_time_tolerance
        else:
            path_tolerance = self.path_tolerance.getTolerance()
            goal_tolerance = self.goal_tolerance.getTolerance()
            goal_time_tolerance = self.goal_time_tolerance.getGoalTimeTolerance()


        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.compliant_trajectory_action_server.is_preempt_requested():
                self.compliant_trajectory_action_server.set_preempted()
                return

            current_time = (rospy.Time.now() - start_time).to_sec()

            # Interpolate position and rotation using your splines
            target_pos = np.array([cs_x(current_time), cs_y(current_time), cs_z(current_time)])
            target_quat = s_rot(current_time).as_quat()

            # Publish the target pose
            self.publish_pose(target_pos, target_quat)

            # publish wrench
            self.publish_wrench_in_tip_frame(wrench_stamped)

            # get current pose
            current_pos, current_quat, ok = self.get_current_pose()
            if not ok:
                continue

            # Calculate the pose error
            error = pose_error(target_pos, target_quat, current_pos, current_quat)

            # check path tolerance
            if not self.check_tolerance(path_tolerance, error):
                rospy.logerr("Path tolerance violated")
                cartesian_trajectory_result = FollowCompliantTrajectoryResult()
                cartesian_trajectory_result.error_code = FollowCompliantTrajectoryResult.PATH_TOLERANCE_VIOLATED
                self.compliant_trajectory_action_server.set_aborted(cartesian_trajectory_result)
                return
        
        finish_time = rospy.Time.now()
        while (rospy.Time.now() - finish_time).to_sec() < goal_time_tolerance:
            if self.compliant_trajectory_action_server.is_preempt_requested():
                self.compliant_trajectory_action_server.set_preempted()
                return

            # Continually publish a wrench in the frame of the tip link
            self.publish_wrench_in_tip_frame(wrench_stamped)

            # Get the current pose of the end effector
            current_pos, current_quat, ok = self.get_current_pose()
            if not ok:
                continue

            # Calculate the pose error
            error = pose_error(target_pos, target_quat, current_pos, current_quat)

            # Check if the goal constraint is satisfied
            if self.check_tolerance(goal_tolerance, error):
                rospy.loginfo("Goal constraint satisfied")
                cartesian_trajectory_result = FollowCompliantTrajectoryResult()
                cartesian_trajectory_result.error_code = FollowCompliantTrajectoryResult.SUCCESSFUL
                self.compliant_trajectory_action_server.set_succeeded(cartesian_trajectory_result)
                return

        rospy.logerr("Goal time tolerance exceeded")
        cartesian_trajectory_result = FollowCompliantTrajectoryResult()
        cartesian_trajectory_result.error_code = FollowCompliantTrajectoryResult.GOAL_TOLERANCE_VIOLATED
        self.compliant_trajectory_action_server.set_aborted(cartesian_trajectory_result)

    def publish_wrench_in_tip_frame(self, wrench_stamped):
        try:
            if wrench_stamped.header.frame_id == "":
                wrench_stamped.header.frame_id = self.tip_link

            # Transform the WrenchStamped message to the tip frame
            transformed_wrench_stamped = self.tf_buffer.transform(wrench_stamped, self.tip_link, timeout=rospy.Duration(1.0))

            # Publish the transformed WrenchStamped message
            self.wrench_publisher.publish(transformed_wrench_stamped)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF2 Exception: {}".format(e))

    def publish_pose(self, position, rotation):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_link
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = rotation[0]
        pose_stamped.pose.orientation.y = rotation[1]
        pose_stamped.pose.orientation.z = rotation[2]
        pose_stamped.pose.orientation.w = rotation[3]
        self.frame_publisher.publish(pose_stamped)

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_link, self.tip_link, rospy.Time(0), timeout=rospy.Duration(1.0))
            # Extract position and quaternion from the transform
            position = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            return position, quaternion, True
        except Exception as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return None, None, False  # Handle lookup failure gracefully
        
    def check_tolerance(self, cartesian_tolerance, error):
        # Check if the pose error is within the tolerance
        if (np.abs(error[0]) > cartesian_tolerance.position_error.x or
            np.abs(error[1]) > cartesian_tolerance.position_error.y or
            np.abs(error[2]) > cartesian_tolerance.position_error.z or
            np.abs(error[3]) > cartesian_tolerance.orientation_error.x or
            np.abs(error[4]) > cartesian_tolerance.orientation_error.y or
            np.abs(error[5]) > cartesian_tolerance.orientation_error.z):
            return False

        return True

