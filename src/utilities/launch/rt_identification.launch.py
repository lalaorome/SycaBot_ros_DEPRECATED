from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

SYCABOT_ID = 1

def generate_launch_description():

    description = []

    identifier = Node(
            package= 'utilities',
            namespace= 'SycaBot_W' + str(SYCABOT_ID),
            executable= 'rt_identification',
            output = 'screen',
            emulate_tty=True,
            parameters=['src/utilities/config/params_identification.yaml']
        )
    
    description.append(identifier)
    return LaunchDescription(description)