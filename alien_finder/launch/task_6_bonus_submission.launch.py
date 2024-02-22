import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    rosbag =   launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/whycon/poses', '/rosin', '/astrobiolocation'],
            output='screen'
        )
   

    spawner_bonus = LaunchDescription([
        Node(
            package='alien_finder',
            executable='spawner_bonus',
            name='spawner_bonus'
        ),
    ])

    return LaunchDescription([
        rosbag,
        spawner_bonus
        ])
