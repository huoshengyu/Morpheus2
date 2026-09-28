import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='twist_topic',
            default_value='/target_twist'
        ),
        launch_ros.actions.Node(
            package='teleop_twist_keyboard',
            executable='keyboard_twist.py',
            name='keyboard_twist',
            output='screen',
            parameters=[
                {
                    'twist_topic': launch.substitutions.LaunchConfiguration('twist_topic')
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_teleop'), 'launch/twist_to_pose.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
