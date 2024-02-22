import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    rosbag =   launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/whycon/poses', '/rosin', '/astrobiolocation'],
            output='screen'
        )
   

    spawner = LaunchDescription([
        Node(
            package='alien_finder',
            executable='spawner',
            name='spawner'
        ),
    ])

    return LaunchDescription([
        rosbag,
        spawner
        ])
