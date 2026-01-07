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
    apriltag_pkg = get_package_share_directory('ur5_moveit')
    apriltag_file = os.path.join(apriltag_pkg, 'launch', 'camera_36h11.launch.yml')
    
    # Launch files
    assignment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ir_launch_file)
    )
    
    apriltag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(apriltag_file)
    )
    
    # Nodes - AGGIUNGI use_sim_time: True
    gripper_node = Node(
        package='ur5_moveit',
        executable='gripper_node',
        name='gripper_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]  # ← FONDAMENTALE
    )
    
    motion_planner = Node(
        package='ur5_moveit',
        executable='motion_planner',
        name='motion_planner',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]  # ← FONDAMENTALE
    )
    
    swap_coordinator = Node(
        package='ur5_moveit',
        executable='swap_coordinator',
        name='swap_coordinator',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}]  # ← FONDAMENTALE
    )
    
    return LaunchDescription([
        # 1. Launch assignment e apriltag subito
        assignment_launch,
        apriltag_launch,
        
        # 2. Gripper dopo 5 secondi
        TimerAction(
            period=5.0,
            actions=[gripper_node]
        ),
        
        # 3. Motion planner dopo 20 secondi
        TimerAction(
            period=10.0,
            actions=[motion_planner]    
        ),
        
        # 4. Swap coordinator dopo 30 secondi
        TimerAction(
            period=20.0,
            actions=[swap_coordinator]    
        ),
    ])