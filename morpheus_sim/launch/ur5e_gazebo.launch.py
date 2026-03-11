from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_sim_gz_pkg_path = FindPackageShare('ur_simulation_gz')
    ur_sim_control_launch_file = PathJoinSubstitution([ur_sim_gz_pkg_path, 'launch', 'ur_sim_control.launch.py'])
    
    ur_moveit_config_pkg_path = FindPackageShare('ur_moveit_config')
    ur_moveit_launch_file = PathJoinSubstitution([ur_moveit_config_pkg_path, 'launch', 'ur_moveit.launch.py'])
    
    morpheus_sim_pkg_path = FindPackageShare('morpheus_sim')
    controllers_file = PathJoinSubstitution([morpheus_sim_pkg_path, 'config', 'ur_controllers.yaml'])
    
    morpheus_description_pkg_path = FindPackageShare('morpheus_description')
    description_file = PathJoinSubstitution([morpheus_description_pkg_path, 'urdf', 'scenes', 'b_bot_scene.xacro'])
    rviz_config_file = PathJoinSubstitution([morpheus_description_pkg_path, 'config', 'morpheus_ur5e.rviz'])
    
    initial_joint_controller = 'cartesian_compliance_controller'
    
    ur_type = 'ur5e'

    ur_sim_control_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_sim_control_launch_file]),
        launch_arguments=[('ur_type', ur_type),
                          ('controllers_file', controllers_file),
                          ('description_file', description_file),
                          ('rviz_config_file', rviz_config_file),
                          ('initial_joint_controller', initial_joint_controller),
                          ('use_sim_time', 'true'),
                          ('launch_rviz', 'false')],
    )
    
    ur_moveit_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_moveit_launch_file]),
        launch_arguments=[('ur_type', ur_type),
                          ('launch_servo', 'true'),
                          ('use_sim_time', 'true'),
                          ('launch_rviz', 'true')],
    )

    # Create the launch description and populate
    ld  = LaunchDescription()
    
    # Add the actions to launch all of the bridge + spawn_model nodes
    ld.add_action(ur_sim_control_description)
    ld.add_action(ur_moveit_description)

    return ld


if __name__ == '__main__':
    generate_launch_description()
