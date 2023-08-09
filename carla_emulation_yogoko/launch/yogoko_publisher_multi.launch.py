from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Used se we have a context, and thus can compute arguments at launchtime
    # Declaring parameter
    in_topic_ns = LaunchConfiguration("in_topic_ns")
    name_ns = LaunchConfiguration("name_ns")
    app_id_base = LaunchConfiguration("app_id_base")
    port_base = LaunchConfiguration("port_base")
    url = LaunchConfiguration("url")
    message_id = LaunchConfiguration("message_id")
    
    n_instances = LaunchConfiguration("n_instances")
    
    launch_desc = []
    for k in range(0, int(n_instances.perform(context))):
        launch_desc.append(
            Node(
                package="yogoko_ros_api",
                executable="yogoko_publisher",
                parameters=[
                    {"topic": in_topic_ns.perform(context) + str(k)},
                    {"message_id": message_id},
                    {"app_id": int(app_id_base.perform(context)) + k},
                    {"url": "{}:{}".format(url.perform(context), int(port_base.perform(context)) + k)},
                ],
                name=name_ns.perform(context) + str(k),
            )
        )

    return launch_desc


def generate_launch_description():
    # Declare launch file arguments
    in_topic_ns_arg = DeclareLaunchArgument("in_topic_ns", default_value="pvt_")
    name_ns_arg = DeclareLaunchArgument("name_ns", default_value="pvt_publisher_")
    app_id_base_arg = DeclareLaunchArgument("app_id_base", default_value="1000")
    port_base_arg = DeclareLaunchArgument("port_base", default_value="49200")
    url_arg = DeclareLaunchArgument("url", default_value="tcp://127.0.0.1")
    message_id_arg = DeclareLaunchArgument("message_id", default_value="203")
    
    
    
    n_instances_arg = DeclareLaunchArgument("n_instances", default_value="1")

    # Adding arguments to the launch description, as well as the launch_setup function 
    launch_desc = [
        in_topic_ns_arg,
        app_id_base_arg,
        port_base_arg,
        url_arg,
        message_id_arg,
        n_instances_arg,
        name_ns_arg,
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(launch_desc)
