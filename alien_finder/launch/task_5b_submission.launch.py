import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    rosbag =   launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/whycon/poses', '/rosin', '/astrobiolocation'],
            output='screen'
        )
   

    mini_core = LaunchDescription([
        Node(
            package='alien_finder',
            executable='mini_core',
            name='mini_core'
        ),
    ])

    return LaunchDescription([
        rosbag,
        mini_core
        ])
