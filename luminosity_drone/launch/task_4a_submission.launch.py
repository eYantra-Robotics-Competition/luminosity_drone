import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix


def generate_launch_description():
    pid_dir = get_package_share_directory('pid_tune')
    # lum_dir = get_package_share_directory('luminosity_drone')
   
     
    pid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_dir, 'launch', 'pid_tune_drone.launch.py'),
        )
    )
   
    rosbag =   launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/whycon/poses'],
            output='screen'
        )

    controller = LaunchDescription([
        Node(
            package='luminosity_drone',
            executable='controller',
            name='controller',
        ),
    ])

    plotjuggler = LaunchDescription([
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
        ),
    ])


    return LaunchDescription([
        pid,
        plotjuggler,
        controller,
        rosbag
        ])