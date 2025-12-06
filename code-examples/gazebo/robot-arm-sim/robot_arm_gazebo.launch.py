import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the common code examples
    common_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], # Assuming ament_prefix_path is set in the environment
        'share',
        'my_gazebo_package', # This package needs to exist for get_package_share_directory
        '../../../../common' # Relative path from share/my_gazebo_package to common/
    )
    urdf_file_path = os.path.join(common_path, 'robot-arm.urdf')

    # Path to Gazebo launch files
    gazebo_ros_launch_dir = get_package_share_directory('gazebo_ros')

    # Start Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_launch_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(common_path, 'empty_world.sdf')}.items()
    )

    # Robot State Publisher node
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Spawn the URDF model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot_arm'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity
    ])
