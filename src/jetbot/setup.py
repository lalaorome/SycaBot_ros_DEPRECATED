from setuptools import setup
import os
from glob import glob

package_name = 'jetbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sycamore',
    maintainer_email='sycamore@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = jetbot.jb_main:main',
            'motors = jetbot.motors_waveshare:main',
            'test = jetbot.jb_testcom:main',
        ],
    },
)
