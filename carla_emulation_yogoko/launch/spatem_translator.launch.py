from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	spawn_sensor = LaunchConfiguration("spawn_sensor")
	sensor_name = LaunchConfiguration("sensor_name")
	topic = LaunchConfiguration("topic")
 
	period = LaunchConfiguration("period")
	station_id = LaunchConfiguration("station_id")
	protocol_version = LaunchConfiguration("protocol_version")
	config = LaunchConfiguration("config")

	spawn_sensor_arg = DeclareLaunchArgument("spawn_sensor", default_value="True")
	sensor_name_arg = DeclareLaunchArgument("sensor_name", default_value="spatem_sensor")
	topic_arg = DeclareLaunchArgument("topic", default_value="spatem")
 
	period_arg = DeclareLaunchArgument("period", default_value="1000")
	station_id_arg = DeclareLaunchArgument("station_id", default_value="10000")
	protocol_version_arg = DeclareLaunchArgument("protocol_version", default_value="1")
	config_arg = DeclareLaunchArgument("config", default_value="")

	
	launch_desc = [
		spawn_sensor_arg,
		sensor_name_arg,
		topic_arg,
		period_arg,
		station_id_arg,
		protocol_version_arg,
 		config_arg
	]

	launch_desc.append(
		Node(
			package="carla_emulation_yogoko",
			executable="spatem_translator",
			parameters=[
				{"spawn_sensor": spawn_sensor},
				{"sensor_name": sensor_name},
				{"topic": topic},
				{"period": period},
				{"station_id": station_id},
				{"protocol_version": protocol_version},
				{"config": config}
			],
			name="spatem_translator",
		),
	)
	return LaunchDescription(launch_desc)
