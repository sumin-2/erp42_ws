from setuptools import setup,find_packages
import os
from glob import glob

package_name = 'erp42_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # 여기에 주의
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # 없으면 ros2에서 인식 안됨
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob(os.path.join('worlds', '*.world'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/urdf',
            glob(os.path.join('urdf', '*.xacro')) +
            glob(os.path.join('urdf', '*.gazebo'))),
        ('share/' + package_name + '/meshes', glob(os.path.join('meshes', '*'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sumin',
    maintainer_email='sumin@example.com',
    description='ERP42 robot description package using xacro and Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "front_camera_viewer = erp42_description.front_camera_viewer:main",
            'steering_drive_bridge = erp42_description.steering_drive_bridge:main',
            'rgbd_fusion = erp42_description.rgbd_fusion:main',
        ],
    },
)
