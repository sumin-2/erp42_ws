#erp42_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import shutil
import subprocess

def generate_launch_description():
    pkg_erp42 = get_package_share_directory('erp42_description')

    # xacro -> urdf 변환
    xacro_file = os.path.join(pkg_erp42, 'urdf', 'erp42.urdf.xacro')
    xacro_executable = shutil.which('xacro')
    xacro_cmd = [
        xacro_executable,
        xacro_file,
        'pkg_share:=' + pkg_erp42,
        'controllers_file:=none'  # 필요 없는 인자지만 placeholder로 둠
    ]
    robot_description_raw = subprocess.check_output(xacro_cmd).decode('utf-8')
    robot_description_param = {'robot_description': robot_description_raw}

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': True}],
        output='screen'
    )

    # Gazebo 실행
    gazebo = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # robot spawn
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'erp42', '-x', '0', '-y', '0', '-z', '0.4'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        gzclient,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity])
    ])