import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package name and path
    pkg_name = 'my_robot_description'
    pkg_path = get_package_share_directory(pkg_name)

    # Process the Xacro file to get URDF XML
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Path to RViz config file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')

    return LaunchDescription([
        # Node 1: Robot State Publisher
        # Publishes the robot model and transforms (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config.toxml()}]
        ),
        
        # Node 2: Joint State Publisher GUI
        # Provides sliders to control joint angles
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        
        # Node 3: RViz2
        # 3D visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
            output='screen'
        )
    ])