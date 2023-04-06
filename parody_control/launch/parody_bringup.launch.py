import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    hostname = "192.168.10.1:801"
    buffer_size = 200
    topic_namespace = "vicon"

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "node_prefix",
                default_value=[launch.substitutions.EnvironmentVariable("USER"), "_"],
                description="Prefix for node names",
            ),
            launch_ros.actions.Node(
                package="parody_control",
                executable="parody_controller",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                arguments=["udp4", "--port", "8888"],
            ),
            launch_ros.actions.Node(
                package="vicon_receiver",
                executable="vicon_client",
                parameters=[
                    {
                        "hostname": hostname,
                        "buffer_size": buffer_size,
                        "namespace": topic_namespace,
                    }
                ],
                output="screen"
            ),
        ]
    )
