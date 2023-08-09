from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Used se we have a context, and thus can compute arguments at launchtime
    # Declaring parameter
    name_ns = LaunchConfiguration("name_ns")
    
    vehicle_ns = LaunchConfiguration("vehicle_ns")
    sensor_name = LaunchConfiguration("sensor_name")
    out_topic_ns = LaunchConfiguration("out_topic_ns")
    
    lat_offset = LaunchConfiguration("lat_offset")
    lon_offset = LaunchConfiguration("lon_offset")
    alt_offset = LaunchConfiguration("alt_offset")
    
    n_instances = LaunchConfiguration("n_instances")
    
    launch_desc = []
    for k in range(0, int(n_instances.perform(context))):
        launch_desc.append(
            Node(
                package="carla_emulation_yogoko",
                executable="pvt_translator",
                parameters=[
                    {"topic": out_topic_ns.perform(context) + str(k)},
                    {"vehicle_name": vehicle_ns.perform(context) + str(k)},
                    {"sensor_name": sensor_name},
                    {"lat_offset": lat_offset},
                    {"lon_offset": lon_offset},
                    {"alt_offset": alt_offset},
                ],
                name=name_ns.perform(context) + str(k),
            )
        )

    return launch_desc


def generate_launch_description():
    # Declare launch file arguments
    out_topic_ns_arg = DeclareLaunchArgument("out_topic_ns", default_value="pvt_")
    
    vehicle_ns_arg = DeclareLaunchArgument("vehicle_ns", default_value="cam_vehicle_")
    name_ns_arg = DeclareLaunchArgument("name_ns", default_value="pvt_translator_")
    sensor_name_arg = DeclareLaunchArgument("sensor_name", default_value="default")
    
    lat_offset_arg = DeclareLaunchArgument("lat_offset", default_value="48.129791363")
    lon_offset_arg = DeclareLaunchArgument("lon_offset", default_value="-1.628170677676")
    alt_offset_arg = DeclareLaunchArgument("alt_offset", default_value="0")
    
    
    n_instances_arg = DeclareLaunchArgument("n_instances", default_value="1")

    # Adding arguments to the launch description, as well as the launch_setup function 
    launch_desc = [
        out_topic_ns_arg,
        vehicle_ns_arg,
        name_ns_arg,
        sensor_name_arg,
        lat_offset_arg,
        lon_offset_arg,
        alt_offset_arg,
        n_instances_arg,
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(launch_desc)
