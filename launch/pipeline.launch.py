import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    
    acquire_query_node = Node(
        package='relocalization_pkg',
        executable='acquire_query_node',
        name='acquire_query_node',
        output='screen')
    
    pose_estimation_node = Node(
        package='relocalization_pkg',
        executable='pose_estimation_node',
        name='pose_estimation_node',
        output='screen')
    
    reloc_eval_node = Node(
        package='relocalization_pkg',
        executable='reloc_eval_node',
        name='reloc_eval_node',
        output='screen')
    
    converge_to_pose_node = Node(
        package='relocalization_pkg',
        executable='converge_to_pose_node',
        name='converge_to_pose_node',
        output='screen')
    
    ld = LaunchDescription()
    
    ld.add_action(acquire_query_node)
    ld.add_action(reloc_eval_node)
    ld.add_action(converge_to_pose_node)
    ld.add_action(pose_estimation_node)
    
    
    return ld