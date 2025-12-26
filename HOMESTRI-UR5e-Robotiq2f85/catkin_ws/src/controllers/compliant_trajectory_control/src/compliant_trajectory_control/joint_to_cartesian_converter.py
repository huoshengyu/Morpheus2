import rospy
import kdl_parser_py.urdf as urdf_parser
import PyKDL as kdl
from trajectory_msgs.msg import JointTrajectory
from cartesian_control_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
import numpy as np
from scipy.spatial.transform import Rotation, RotationSpline
from scipy.interpolate import CubicSpline

class JointToCartesianConverter:
    def __init__(self, urdf, joint_names, base_link, tip_link):
        self.joint_names = joint_names
        self.base_link = base_link
        self.tip_link = tip_link

        # Create a KDL tree and chain
        success, robot_tree = urdf_parser.treeFromString(urdf)
        if not success:
            rospy.logerr("Failed to parse the URDF")
            raise RuntimeError("Failed to parse the URDF")
        
        self.robot_chain = robot_tree.getChain(self.base_link, self.tip_link)
        # Add the segments to the chain (use your own code to define the robot's kinematic chain)
        self.fk_vel_solver = kdl.ChainFkSolverVel_recursive(self.robot_chain)
        self.fk_pos_solver = kdl.ChainFkSolverPos_recursive(self.robot_chain)

    def joint_traj_to_cart_spline(self, joint_trajectory):
        joint_map = self.createJointMap(joint_trajectory.joint_names)

        n = len(joint_trajectory.points)
        time = np.zeros(n)
        x = np.zeros(n)
        y = np.zeros(n)
        z = np.zeros(n)
        quats = np.zeros((n, 4))

        for i in range(n):
            # Convert joint positions to KDL JntArray
            joint_positions = kdl.JntArray(len(self.joint_names))
            for j in range(len(self.joint_names)):
                k = joint_map[j]
                joint_positions[j] = joint_trajectory.points[i].positions[k]

            frame = kdl.Frame()
            self.fk_pos_solver.JntToCart(joint_positions, frame)
            
            time[i] = joint_trajectory.points[i].time_from_start.to_sec()
            x[i] = frame.p.x()
            y[i] = frame.p.y()
            z[i] = frame.p.z()
            quat = frame.M.GetQuaternion()
            quats[i, :] = quat

        cs_x = CubicSpline(time, x)
        cs_y = CubicSpline(time, y)
        cs_z = CubicSpline(time, z)
        s_rot = RotationSpline(time, Rotation.from_quat(quats))
        duration = time[-1]

        return cs_x, cs_y, cs_z, s_rot, duration

    def createJointMap(self, req_joint_names):
        if len(req_joint_names) != len(self.joint_names):
            rospy.logerr("Mismatch in the number of joints. Expected {} joints, but got {} joints."
                         .format(len(self.joint_names), len(req_joint_names)))
            raise RuntimeError("Mismatch in the number of joints")

        joint_map = []
        joint_name_to_index = {}

        for i in range(len(req_joint_names)):
            joint_name_to_index[req_joint_names[i]] = i

        for i in range(len(self.joint_names)):
            joint_name = self.joint_names[i]
            if joint_name in joint_name_to_index:
                joint_map.append(joint_name_to_index[joint_name])

        return joint_map
