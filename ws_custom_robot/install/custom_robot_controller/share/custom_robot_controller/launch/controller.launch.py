import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.parameter_descriptions import ParameterValue # type: ignore
from launch.substitutions import Command, LaunchConfiguration # type: ignore
from launch.conditions import UnlessCondition # type: ignore
from ament_index_python.packages import get_package_share_directory # type: ignore



def generate_launch_description():
    return LaunchDescription(
        [
        ]
    )