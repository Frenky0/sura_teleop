import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    environment = LaunchConfiguration("environment").perform(context)
    pwm_channels = LaunchConfiguration("pwm_channels").perform(context)
    pressure_offset_pa = LaunchConfiguration("pressure_offset_pa").perform(context)

    if environment not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported environment '{environment}'. Use 'sim' or 'real'."
        )

    description_pkg = get_package_share_directory("bluerov_description")
    hardware_pkg = get_package_share_directory("sura_hardware_interface")

    xacro_file = os.path.join(
        description_pkg,
        "urdf",
        "bluerov.urdf.xacro",
    )

    csv_file = os.path.join(
        hardware_pkg,
        "config",
        "t200_lookup.csv",
    )

    xacro_command = [
        "xacro ",
        xacro_file,
        " robot_namespace:=",
        robot_namespace,
        " environment:=",
        environment,
        " lookup_csv:=",
        csv_file,
        " stonefish_topic:=/",
        robot_namespace,
        "/controller/thruster_setpoints_sim",
        " pwm_channels:=",
        pwm_channels,
        " pressure_offset_pa:=",
        pressure_offset_pa,
    ]

    robot_description = Command(xacro_command)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=robot_namespace,
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                }
            ],
            remappings=[
                ("/robot_description", f"/{robot_namespace}/robot_description"),
                ("/joint_states", f"/{robot_namespace}/joint_states"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="bluerov2"),
            DeclareLaunchArgument("environment", default_value="real"),
            DeclareLaunchArgument("pwm_channels", default_value="0,1,2,3,4,5,6,7"),
            DeclareLaunchArgument("pressure_offset_pa", default_value="101325.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )