import os
 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
	package_name = 'rosmaster_driver'

 
	params_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')
 
	rosmaster_driver = Node(
		package=package_name,
		executable='driver',
		parameters=[params_file]
 
	)
 
	return LaunchDescription([
		rosmaster_driver
	])
