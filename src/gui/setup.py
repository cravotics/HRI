from setuptools import find_packages, setup

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],  # Added PyQt5 as a dependency
    zip_safe=True,
    maintainer='swaraj',
    maintainer_email='swarajmr@umd.edu',
    description='A ROS2 GUI package for robot monitoring and control using PyQt5',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_monitor = gui.gui:main',  # Entry point for the GUI
        ],
    },
)
