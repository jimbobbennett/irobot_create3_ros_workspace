from setuptools import setup
from setuptools import find_packages

package_name = 'irobot_create3_example_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jimbobbennett',
    maintainer_email='jim.bennett@microsoft.com',
    description='Example ROS 2 Python code to connect an iRobot® Create® 3 to Azure IoT Central',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_robot = irobot_create3_example_py.control_robot.control_robot:main'
        ],
    },
)
