#!/usr/bin/env python3
# import os # 移除 os
# from ament_index_python.packages import get_package_share_directory # 移除

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable # 恢复 Command, FindExecutable
# from launch.substitutions import PathJoinSubstitution # 恢复
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # 恢复 ParameterValue

def generate_launch_description():
    pkg = FindPackageShare("tracked_bot_description") # 恢复 pkg 变量
    # urdf_path = os.path.join(pkg_path, "urdf", "tracked_bot.urdf") # 移除直接路径
    xacro_path = PathJoinSubstitution([pkg, "urdf", "tracked_bot.urdf.xacro"]) # 使用 xacro 文件路径

    # with open(urdf_path, 'r') as infp:
    #     robot_description = infp.read()
    # 移除直接读取

    # 恢复 Xacro 处理
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="my_joint_state_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
        ),
    ])
