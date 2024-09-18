# Import necessary modules from the launch and launch_ros packages
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# This function generates the launch description
def generate_launch_description():
    return LaunchDescription([

        Node(
            package='custom_mode',
            executable='custom_mode',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('custom_mode'),
                    'cfg/params.yaml'
                ])
            ]
        ),
    ])

