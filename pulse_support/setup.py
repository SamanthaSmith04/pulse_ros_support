from setuptools import find_packages, setup

package_name = 'pulse_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pulse', 'warp', 'rclpy', 'pulse_msgs'],
    zip_safe=True,
    maintainer='Samantha Smith',
    maintainer_email='smith.15485@osu.edu',
    description='ROS2 wrapper for pulse',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pulse_server = pulse_support.pulse_server:main',
        ],
    },
)
