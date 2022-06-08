from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


n_sycabots = int(input('number of Sycabots ?'))

def generate_launch_description():

    description = []

    for i in range(n_sycabots):
        description.append( Node(
            package= 'opti_track',
            namespace= 'SycaBot_W' + str(i+1),
            executable= 'opti_link',
            remappings = [
                ('/vrpn_client_node/SycaBot_W1/pose', '/vrpn_client_node/SycaBot_W'+ str(i+1)+'/pose')
            ]
        ))

    return LaunchDescription(description)