from setuptools import setup

package_name = 'alphabot2_door_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/door_finder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Door detection and search nodes for Alphabot2 in ROS2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'door_detector = alphabot2_door_detection.door_detector_node:main',
            'door_search = alphabot2_door_detection.door_search_node:main',
        ],
    },
)
