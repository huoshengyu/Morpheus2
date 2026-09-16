from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # UR specific arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    # End effector arguments
    ee_type = LaunchConfiguration("ee_type")
    # Control arguments
    controller = LaunchConfiguration("controller")
    use_keyboard = LaunchConfiguration("use_keyboard")
    # Moveit arguments
    collision = LaunchConfiguration("collision")

    ur_control_node = IncludeLaunchDescription(
        launch_description_source=AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("morpheus_driver"), "launch", "ur_control.launch.py"]
            )
        )
    )
    
    control_nodes = []
    # If a gamepad is selected as controller, launch it
    if controller.perform(context) == "ps4":
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "ps4_twist.launch.py"]
                    )
                ),
            )
        )
    elif controller.perform(context) == "xbox":
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "xbox360_twist.launch.py"]
                    )
                ),
            )
        )
    # If keyboard is selected as controller, spoof ps4 input via keyboard
    if controller.perform(context) == "keyboard":
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "key_ps4_twist.launch.py"]
                    )
                ),
            )
        )
    # Else, if use_keyboard is set to true, spoof any gamepad input via keyboard
    elif use_keyboard.perform(context) == "true":
        control_nodes.append(
            IncludeLaunchDescription(
                launch_description_source=AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("morpheus_teleop"), "launch", "key_to_joy.launch.py"]
                    )
                ),
            )
        )
    
    if collision.perform(context) == "true":
        moveit_node = IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("morpheus_moveit"), "launch", "moveit.launch.py"]
                )
            )
        )
        collision_node = IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("morpheus_collision"), "launch", "collision.launch.py"]
                )
            )
        )
        control_nodes.append(moveit_node)
        control_nodes.append(collision_node)
    

    nodes_to_start = [
        ur_control_node,
    ] + control_nodes

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", 
            default_value="0.0.0.0",
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_type",
            description="Type/series of end effector.",
            choices=[
                "",
                "robotiq",
                "onrobot",
            ],
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
             description="Type of control device.",
            choices=[
                "",
                "ps4",
                "xbox",
                "keyboard",
            ],
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_keyboard",
            description="Type of control device.",
            default_value="false",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])