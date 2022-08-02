from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


SYCABOT_ID = 1

def generate_launch_description():

    description = []

    DeadzoneActionServer = Node(
        package= 'jetbot',
        namespace= 'SycaBot_W' + str(SYCABOT_ID),
        executable= 'DeadzoneActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml']
    )
    

    IdentificationActionServer = Node(
        package= 'jetbot',
        namespace= 'SycaBot_W' + str(SYCABOT_ID),
        executable= 'IdentificationActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml']
    )
    description.append(IdentificationActionServer)
    description.append(DeadzoneActionServer)
    return LaunchDescription(description)