from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path  = get_package_share_directory('tracked_bot_description')
    world     = os.path.join(pkg_path, 'worlds', 'empty.world')
    robot_desc= os.path.join(pkg_path, 'urdf',   'tracked_bot.urdf.xacro')

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # Gazebo server + client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'))
        ),
        # 发布机器人到 Gazebo
        Node(package='gazebo_ros',
             executable='spawn_entity.py',
             arguments=['-entity', 'tracked_bot',
                        '-topic',  'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.1'],
             output='screen'),
        # 同步机器人状态 → TF
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'use_sim_time': True}],
             arguments=[robot_desc]),
    ])

