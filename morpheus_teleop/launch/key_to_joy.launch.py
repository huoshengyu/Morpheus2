import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='morpheus_teleop',
            executable='key_to_joy.py',
            name='key_to_joy',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
