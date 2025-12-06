from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='my_publisher',
            name='publisher_node',
            output='screen'
        ),
        Node(
            package='my_first_package',
            executable='my_subscriber',
            name='subscriber_node',
            output='screen'
        ),
        Node(
            package='my_service_package',
            executable='add_two_ints_server',
            name='add_server_node',
            output='screen'
        ),
        Node(
            package='my_action_package',
            executable='fibonacci_action_server',
            name='fibonacci_server_node',
            output='screen'
        )
    ])
