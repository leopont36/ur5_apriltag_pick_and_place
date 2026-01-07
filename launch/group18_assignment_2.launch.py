from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    ir_launch_dir = get_package_share_directory('ir_launch')
    ir_launch_file = os.path.join(ir_launch_dir, 'launch', 'assignment_2.launch.py')
    apriltag_pkg = get_package_share_directory('group18_assignment_2')
    apriltag_file = os.path.join(apriltag_pkg, 'launch', 'camera_36h11.launch.yml')
    
    # Launch files
    assignment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ir_launch_file)
    )
    
    apriltag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(apriltag_file)
    )
    
    # Nodes
    gripper_node = Node(
        package='group18_assignment_2',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    
    motion_planner = Node(
        package='group18_assignment_2',
        executable='motion_planner',
        name='motion_planner',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    
    swap_coordinator = Node(
        package='group18_assignment_2',
        executable='swap_coordinator',
        name='swap_coordinator',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        # Launch assignment e apriltag subito
        assignment_launch,
        apriltag_launch,
        
        # Gripper dopo 5 secondi
        TimerAction(
            period=5.0,
            actions=[gripper_node]
        ),
        
        # Motion planner dopo 20 secondi
        TimerAction(
            period=10.0,
            actions=[motion_planner]    
        ),
        
        # Swap coordinator dopo 30 secondi
        TimerAction(
            period=20.0,
            actions=[swap_coordinator]    
        ),
    ])