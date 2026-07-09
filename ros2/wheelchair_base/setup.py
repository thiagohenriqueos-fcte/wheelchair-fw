from setuptools import find_packages, setup

package_name = 'wheelchair_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/wheelchair.yaml',
            'config/ekf.yaml',
            'config/slam.yaml',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/wheelchair.urdf.xacro',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheelchair',
    maintainer_email='wheelchair@local',
    description='Ponte serial ESP32 + integracao com IMU para a cadeira.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = wheelchair_base.bridge_node:main',
        ],
    },
)
