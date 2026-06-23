import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('amr_description')
    urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_robot', '-topic', 'robot_description'],
            output='screen'
        ),
        # Bridge ROS2 /cmd_vel to Gazebo /cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),
    ])