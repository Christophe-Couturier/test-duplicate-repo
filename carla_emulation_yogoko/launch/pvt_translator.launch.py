from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	vehicle_name = LaunchConfiguration("vehicle_name")
	sensor_name = LaunchConfiguration("sensor_name")
	topic = LaunchConfiguration("topic")
 
	lon_offset = LaunchConfiguration("lon_offset")
	lat_offset = LaunchConfiguration("lat_offset")
	alt_offset = LaunchConfiguration("alt_offset")

	vehicle_name_arg = DeclareLaunchArgument("vehicle_name", default_value="ego_vehicle")
	sensor_name_arg = DeclareLaunchArgument("sensor_name", default_value="default")
	topic_arg = DeclareLaunchArgument("topic", default_value="pvt")
 
	lon_offset_arg = DeclareLaunchArgument("lon_offset", default_value="-1.625653")
	lat_offset_arg = DeclareLaunchArgument("lat_offset", default_value="48.132341")
	alt_offset_arg = DeclareLaunchArgument("alt_offset", default_value="0.0")
	
	launch_desc = [
		vehicle_name_arg,
		sensor_name_arg,
		topic_arg,
		lon_offset_arg,
		lat_offset_arg,
		alt_offset_arg,
	]

	launch_desc.append(
		Node(
			package="carla_emulation_yogoko",
			executable="pvt_translator",
			parameters=[
				{"vehicle_name": vehicle_name},
				{"sensor_name": sensor_name},
				{"topic": topic},
				{"lon_offset": lon_offset},
				{"lat_offset": lat_offset},
				{"alt_offset": alt_offset},
			],
			name="pvt_translator",
		),
	)
	return LaunchDescription(launch_desc)
