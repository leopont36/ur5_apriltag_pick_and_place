from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ir_launch_dir = get_package_share_directory('ir_launch')
    ir_launch_file = os.path.join(ir_launch_dir, 'launch', 'assignment_2.launch.py')

    apriltag_pkg = get_package_share_directory('ur5_moveit')
    apriltag_file = os.path.join(apriltag_pkg, 'launch', 'camera_36h11.launch.yml')

    assignment_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(ir_launch_file))
    
    apriltag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(apriltag_file)
    )

    gripper_node = Node(
        package='ur5_moveit',
        executable='gripper_node',
        output='screen',
        emulate_tty=True
    )

    delayed_servers = TimerAction(
        period=5.0,
        actions=[gripper_node]
    )

    return LaunchDescription([
        assignment_launch,
        #apriltag_launch,
        gripper_node,
    ])  
    
