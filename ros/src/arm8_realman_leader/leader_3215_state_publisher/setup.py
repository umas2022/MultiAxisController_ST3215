from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leader_3215_state_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umas',
    maintainer_email='umas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publish_joint7_pose = leader_3215_state_publisher.publish_joint7_pose:main',
            'simple_joint_publisher = leader_3215_state_publisher.simple_joint_publisher:main'
        ],
    },
)