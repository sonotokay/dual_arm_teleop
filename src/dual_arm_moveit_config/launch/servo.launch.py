import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_pkg = FindPackageShare("dual_arm_description")
    moveit_pkg = "dual_arm_moveit_config"

    # robot_description (URDF) + semantic (SRDF)
    urdf = PathJoinSubstitution([desc_pkg, "urdf", "dual_ur5e.urdf.xacro"])
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf]), value_type=str
        )
    }

    srdf_path = os.path.join(
        get_package_share_directory(moveit_pkg), "config", "dual_ur5e.srdf"
    )
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml(moveit_pkg, "config/kinematics.yaml")

    left_servo = load_yaml(moveit_pkg, "config/left_servo.yaml")
    right_servo = load_yaml(moveit_pkg, "config/right_servo.yaml")

    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_servo_node",
        parameters=[
            {"moveit_servo": left_servo},
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output="screen",
    )

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_servo_node",
        parameters=[
            {"moveit_servo": right_servo},
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output="screen",
    )

    return LaunchDescription([left_servo_node, right_servo_node])