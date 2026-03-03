from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_sim_gz_pkg_path = FindPackageShare('ur_simulation_gz')
    ur_sim_moveit_launch_file = PathJoinSubstitution([ur_sim_gz_pkg_path, 'launch', 'ur_sim_moveit.launch.py'])
    
    morpheus_sim_pkg_path = FindPackageShare('morpheus_sim')
    controllers_file = PathJoinSubstitution([morpheus_sim_pkg_path, 'config', 'ur', 'ur5e', 'gazebo_controllers.yaml'])
    
    morpheus_description_pkg_path = FindPackageShare('morpheus_description')
    controllers_file = PathJoinSubstitution([morpheus_description_pkg_path, 'urdf', 'scenes', 'ur5e_scene.xacro'])
    
    ur_type = 'ur5e'

    ur_sim_moveit_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_sim_moveit_launch_file]),
        launch_arguments=[('ur_type', ur_type),
                          ('controllers_file', controllers_file),
                          ('description_file', controllers_file)]
    )

    # Create the launch description and populate
    ld  = LaunchDescription()
    
    # Add the actions to launch all of the bridge + spawn_model nodes
    ld.add_action(ur_sim_moveit_description)

    return ld


if __name__ == '__main__':
    generate_launch_description()
