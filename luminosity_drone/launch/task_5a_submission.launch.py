import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    rosbag =   launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/whycon/poses'],
            output='screen'
        )
   

    waypoint_controller = LaunchDescription([
        Node(
            package='luminosity_drone',
            executable='waypoint_controller',
            name='waypoint_controller',
        ),
    ])

    return LaunchDescription([
        waypoint_controller,
        rosbag
        ])