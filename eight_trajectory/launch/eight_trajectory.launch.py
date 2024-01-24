# humble
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True',
        description='Use simulation (Gazebo) clock if true')
    # Use the launch argument in the node configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # MoveItCpp demo executable
    eight_trajectory_node = Node(
        name="eight_trajectory_node",
        package="eight_trajectory",
        executable="eight_trajectory",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    
    # MoveItCpp demo executable
    kinematic_model_node = Node(
        name="kinematic_model_node",
        package="kinematic_model",
        executable="kinematic_model",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        kinematic_model_node,
        LogInfo(msg=["Starting Eight Trajectory Node..."]),
        TimerAction(period=5.0, 
                    actions=[eight_trajectory_node]),
    ])