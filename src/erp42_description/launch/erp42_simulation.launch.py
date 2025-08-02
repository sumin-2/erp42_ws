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

    # xacro -> URDF
    xacro_file = os.path.join(pkg, 'urdf', 'erp42.urdf.xacro')
    gazebo_ctl = os.path.join(pkg, 'urdf', 'erp42_control.gazebo')
    xacro_exe = shutil.which('xacro')
    urdf = subprocess.check_output([
        xacro_exe, xacro_file,
        f'pkg_share:={pkg}',
        f'controllers_file:={gazebo_ctl}'
    ]).decode()
    robot_description = {'robot_description': urdf}

    # 1) robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # 2) Gazebo server & client (ExecuteProcess)
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 3) spawn_entity after short delay
    spawn = TimerAction(
        period=1.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'erp42',
                '-x', '0', '-y', '0', '-z', '0.4'
            ],
            output='screen'
        )]
    )

    # 4) controller spawners in sequence
    js_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager', executable='spawner.py',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )]
    )
    fp_spawner = TimerAction(
        period=4.5,
        actions=[Node(
            package='controller_manager', executable='spawner.py',
            arguments=['forward_position_controller'],
            output='screen'
        )]
    )
    vel_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager', executable='spawner.py',
            arguments=['velocity_controller'],
            output='screen'
        )]
    )

    # 5) steering_drive_bridge after controllers
    
    bridge = TimerAction(
        period=7.5,
        actions=[Node(
            package='erp42_description', executable='steering_drive_bridge',
            name='steering_drive_bridge',
            parameters=[
                {'wheel_radius': 0.28},
                {'steering_gain': 0.3},
                {'max_steer': 0.523599},
                {'use_sim_time': True},
            ],
            output='screen'
        )]
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rsp,
        gzserver,
        gzclient,
        spawn,
        js_spawner,
        fp_spawner,
        vel_spawner,
        bridge,
    ])