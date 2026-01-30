import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='morpheus_trajectory',
            executable='goal_haptics.py',
            name='goal_haptics',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'morpheus_trajectory'), 'launch/goal_transform.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
