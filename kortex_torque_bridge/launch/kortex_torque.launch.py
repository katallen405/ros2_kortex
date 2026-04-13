from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip",         default_value="192.168.1.10"),
        DeclareLaunchArgument("username",          default_value="admin"),
        DeclareLaunchArgument("password",          default_value="admin"),
        DeclareLaunchArgument("dof",               default_value="7"),
        DeclareLaunchArgument("cyclic_period_ms",  default_value="1"),
        DeclareLaunchArgument("torque_topic",      default_value="~/joint_torque_command"),

        Node(
            package="kortex_torque_bridge",
            executable="kortex_torque_node",
            name="kortex_torque_bridge",
            output="screen",
            parameters=[{
                "robot_ip":         LaunchConfiguration("robot_ip"),
                "username":         LaunchConfiguration("username"),
                "password":         LaunchConfiguration("password"),
                "dof":              LaunchConfiguration("dof"),
                "cyclic_period_ms": LaunchConfiguration("cyclic_period_ms"),
                "torque_topic":     LaunchConfiguration("torque_topic"),
            }],
        ),
    ])
