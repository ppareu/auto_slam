from setuptools import find_packages, setup

package_name = 'auto_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyqt5'],
    zip_safe=True,
    maintainer='gibeom',
    maintainer_email='gibeom@todo.todo',
    description='A ROS 2 package for autonomous SLAM and user interface management.',
    license='Apache-2.0',
    tests_require=['pytest', 'flake8'],
    entry_points={
        'console_scripts': [
            'auto_slam = auto_slam.auto_slam:main',
            'user_interface = auto_slam.UserInterface:main'
        ],
    },
)
