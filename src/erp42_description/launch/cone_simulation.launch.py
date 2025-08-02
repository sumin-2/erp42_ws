#!/usr/bin/env python3
import os
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('erp42_description')

    # 1) xacro → URDF (controllers_file 인자로 .gazebo 플러그인 파일을 넘김)
    xacro_file = os.path.join(pkg, 'urdf', 'erp42.urdf.xacro')
    gazebo_ctl = os.path.join(pkg, 'urdf', 'erp42_control.gazebo')
    xacro_exe = shutil.which('xacro')
    urdf = subprocess.check_output([
        xacro_exe, xacro_file,
        f'pkg_share:={pkg}',
        f'controllers_file:={gazebo_ctl}'
    ]).decode()
    robot_description = {'robot_description': urdf}

    # 2) robot_state_publisher
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )
    world_path = os.path.join(
        get_package_share_directory('erp42_description'),
        'worlds', 'cone1.world' 
    )
    # 3) Gazebo server & client
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    # 4) Spawn entity (여기서 URDF + 플러그인 모두 같이 들어갑니다)
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'erp42',
            '-x', '0', '-y', '0', '-z', '0.4'
        ],
        output='screen'
    )

    # 5) spawner 들
    #    — ros2_control_node 는 플러그인 안에서 이미 올라갑니다.
    #    — spawner 만 실행하면, controller_manager 서비스에 붙어서 자동으로 로드/시작합니다.
    js_spawner = Node(
        package='controller_manager', executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    fp_spawner = Node(
        package='controller_manager', executable='spawner.py',
        arguments=['forward_position_controller'],
        output='screen'
    )
    vel_spawner = Node(
        package='controller_manager', executable='spawner.py',
        arguments=['velocity_controller'],
        output='screen'
    )

    # 6) steering_drive_bridge
    bridge = Node(
        package='erp42_description', executable='steering_drive_bridge',
        name='steering_drive_bridge',
        parameters=[
            {'wheel_radius': 0.28},
            {'steering_gain': 0.3},
            {'max_steer': 0.523599},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # — core
        rsp,
        gzserver,
        gzclient,
        spawn,

        # — spawn_entity 후에 플러그인이 올라갈 시간을 줍니다
        TimerAction(period=1.0, actions=[js_spawner]),
        TimerAction(period=1.5, actions=[fp_spawner]),
        TimerAction(period=2.0, actions=[vel_spawner]),

        # — 마지막에 브리지
        TimerAction(period=3.0, actions=[bridge]),
    ])
