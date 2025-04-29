#!/usr/bin/env python3
# 履带式机器人显示启动文件
# 用于启动机器人状态发布器、关节状态发布器和RViz可视化工具

# ROS2功能包路径管理
from ament_index_python.packages import get_package_share_path
# Launch系统相关导入
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
# ROS2节点相关导入
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 获取功能包路径
    tracked_bot_path = get_package_share_path('tracked_bot_description')
    # 设置默认的URDF模型文件路径
    default_model_path = tracked_bot_path / 'urdf/tracked_bot.urdf'
    # 设置默认的RViz配置文件路径
    default_rviz_config_path = tracked_bot_path / 'rviz/urdf.rviz'

    # 声明URDF模型文件路径参数
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    
    # 声明RViz配置文件路径参数
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 使用xacro处理URDF文件
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 配置机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',    # 节点所属功能包
        executable='robot_state_publisher', # 可执行文件名
        output='screen',                   # 输出到屏幕
        parameters=[robot_description]      # 传递机器人描述参数
    )

    # 配置关节状态发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',    # 节点所属功能包
        executable='joint_state_publisher'  # 可执行文件名
    )

    # 配置RViz可视化节点
    rviz_node = Node(
        package='rviz2',                   # 节点所属功能包
        executable='rviz2',                # 可执行文件名
        name='rviz2',                      # 节点名称
        output='screen',                   # 输出到屏幕
        arguments=['-d', LaunchConfiguration('rvizconfig')], # 加载RViz配置文件
    )

    # 返回启动描述
    return LaunchDescription([
        model_arg,                    # URDF模型文件路径参数
        rviz_arg,                    # RViz配置文件路径参数
        joint_state_publisher_node,  # 关节状态发布节点
        robot_state_publisher_node,  # 机器人状态发布节点
        rviz_node                    # RViz可视化节点
    ]) 