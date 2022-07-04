from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node

SYCABOT_ID = 1

def generate_launch_description():

    description = []

    motors = Node(
            package= 'jetbot',
            namespace= 'SycaBot_W' + str(SYCABOT_ID),
            executable= 'motors',
            output = 'screen',
            emulate_tty=True,
        )
    
    description.append(motors)
    return LaunchDescription(description)