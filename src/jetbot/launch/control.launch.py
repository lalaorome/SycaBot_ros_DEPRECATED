from tkinter import N
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node

ids = []
n = int(input("Enter number of Sycabots : "))
  
for i in range(0, n):
    ids.append(int(input("id n°%d : "%(i))))


def generate_launch_description():

    description = []

    for i in ids :
        SYCABOT_ID = i
        jb_main = Node(
                package= 'jetbot',
                namespace= 'SycaBot_W' + str(SYCABOT_ID),
                executable= 'start',
                output = 'screen',
                emulate_tty=True,
                parameters=[
                    {'id': SYCABOT_ID}
                ],
            )

    description.append(jb_main)
    return LaunchDescription(description)