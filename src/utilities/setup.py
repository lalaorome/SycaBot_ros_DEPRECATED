from setuptools import setup
import os
from glob import glob

package_name = 'utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sycamore',
    maintainer_email='pierre.chassagne@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'identification = utilities.identification:main',
            'rt_identification = utilities.rt_identification:main',
            'deadzone = utilities.deadzone:main'
        ],
    },
)
