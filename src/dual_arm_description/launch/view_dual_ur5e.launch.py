from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = FindPackageShare("dual_arm_description")
    rviz_config = PathJoinSubstitution([pkg, "rviz", "view_dual_ur5e.rviz"])
    urdf_xacro = PathJoinSubstitution([pkg, "urdf", "dual_ur5e.urdf.xacro"])

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_xacro]),
            value_type=str,
        )
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])