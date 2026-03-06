# mecanum_control.launch.py
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'rosmaster'  # <-- change to your package name
    share_dir = get_package_share_directory(pkg_name)

    # Path to your xacro
    xacro_file = PathJoinSubstitution([share_dir, 'description', 'rosmaster.urdf.xacro'])
    controllers_file = PathJoinSubstitution([share_dir, 'config', 'controllers.yaml'])

    # Convert xacro -> robot_description (as a STRING)
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description_param = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Start the ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_param, str(controllers_file)],
        output='screen'
    )

    # Spawners for controllers (delayed slightly so controller_manager is up)
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_wheels_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheels_velocity_controller'],
        output='screen'
    )

    # Delay the spawners so ros2_control_node is ready
    delayed_spawner = TimerAction(
        period=1.0,
        actions=[spawn_joint_state_broadcaster, TimerAction(period=0.5, actions=[spawn_wheels_controller])]
    )

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(delayed_spawner)
    return ld