import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # #Launch joint_state_broadcaster
        # Node(
        # package='controller_manager',
        # executable='spawner',
        # arguments=['joint_state_broadcaster', '--controller-manager',
        #            ['controller_manager']],
        # ),
        # Launch joint_traj_actions node
        Node(
            package='controllers',
            executable='joint_traj_pub',
            name='joint_traj_pub_node',
            output='screen'
        ),

        #Launch iiwa_arm_controller
        Node(
            package='controllers',
            executable='iiwa_ik',
            name='iiwa_ik_node',
            output='screen'
        ),

        

    ])
