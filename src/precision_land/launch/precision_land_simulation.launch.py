from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Start the Micro XRCE-DDS Agent using ExecuteProcess
        # ExecuteProcess(
        #     cmd=['screen', '-dmS', 'dds_agent', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888'],
        #     name='dds_agent_process'
        # ),
        # Bridge for /camera topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image']
        ),
        # Bridge for /camera_info topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            output='screen',
            arguments=['/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo']
        ),
        # Bridge for /clock topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock']
        ),
        # Launch the precision_land node
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml']),
                {'use_sim_time': True}
            ],
        ),
    ])
