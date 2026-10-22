from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv1_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'sim'), glob('config/sim/*')),
        (os.path.join('share', package_name, 'sim', 'worlds'), glob('sim/worlds/*')),
    ] + [
        # Simulation models, one install directory per model (and its textures)
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in glob('sim/models/**/*', recursive=True) if os.path.isfile(f)
    ],
    package_data={
        'ugv1_controller.web': ['static/landing/*', 'static/drive/*'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_control_node = ugv1_controller.motor_control_node:main",
            "capture_node = ugv1_controller.capture_node:main",
            "process_node = ugv1_controller.process_node:main",
            "web_node = ugv1_controller.web_node:main",
            "imu_node = ugv1_controller.imu_node:main",
            "odom_tf_node = ugv1_controller.odom_tf_node:main",
            "sim_camera_relay = ugv1_controller.sim_camera_relay:main",
            "sim_route_node = ugv1_controller.sim_route_node:main",
            "vio_eval = ugv1_controller.vio_eval:main"
        ],
    },
)
