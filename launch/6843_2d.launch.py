from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

devices = [
    "00ED2284",
    # "00F48CC4",
    # "00F48C12",
]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mmwave",
            executable="mmwave_node",
            namespace=f"mmwave_{i}",
            name="mmwave",
            parameters=[{
                "device_id": device,
                "config_file": PathJoinSubstitution([
                    FindPackageShare("mmwave"),
                    "cfg",
                    "6843_2d.cfg"
                ]),
                "frame_id": f"mmwave_{i}",
            }],
            output="screen",
            emulate_tty=True
        ) for i, device in enumerate(devices)
    ])
