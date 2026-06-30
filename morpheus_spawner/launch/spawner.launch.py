
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='mesh_path',
            default_value=get_package_share_directory(
                'morpheus_description') + '/meshes/components/collision/dragon_simple.obj'
        ),
        DeclareLaunchArgument(
            name='world_pose',
            default_value='-x -0.1 -y 0.8528 -z 0.91915 -R 0.0 -P 0.0 -Y 0.0',
            description='Pose to spawn the object at'
        ),
        DeclareLaunchArgument(
            name='mode',
            default_value='none'
        ),
        DeclareLaunchArgument(
            name='arm_group',
            default_value='arm'
        ),
        Node(
            package='morpheus_spawner',
            executable='morpheus_spawner',
            name='morpheus_spawner',
            output='screen',
            parameters=[
                {
                    'arm_group': LaunchConfiguration('arm_group')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
