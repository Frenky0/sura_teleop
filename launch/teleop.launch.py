import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def namespaced_config(config_file, robot_namespace, node_name):
    text = Path(config_file).read_text(encoding="utf-8")
    if robot_namespace:
        text = text.replace(f"{node_name}:", f"/{robot_namespace}/{node_name}:", 1)
        text = text.replace("/sura/", f"/{robot_namespace}/")

    output_file = f"/tmp/sura_teleop_{robot_namespace or 'root'}_{node_name}_{Path(config_file).name}"
    Path(output_file).write_text(text, encoding="utf-8")
    return output_file


def resolve_robot_model(robot_model, robot_namespace):
    if robot_model and robot_model != "auto":
        return robot_model.lower()
    return robot_namespace.lower()


def teleop_spec(package_share, robot_model):
    if robot_model in ("blueboat", "surface", "catamaran", "usv"):
        return {
            "executable": "blueboat_teleop",
            "node_name": "blueboat_teleop",
            "config": os.path.join(package_share, "config", "teleop_params_blueboat.yaml"),
        }
    if robot_model == "bluerov":
        return {
            "executable": "cirtesub_teleop",
            "node_name": "sura_teleop",
            "config": os.path.join(package_share, "config", "teleop_params_bluerov.yaml"),
        }
    return {
        "executable": "cirtesub_teleop",
        "node_name": "sura_teleop",
        "config": os.path.join(package_share, "config", "teleop_params_cirtesub.yaml"),
    }


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    robot_model = resolve_robot_model(
        LaunchConfiguration("robot_model").perform(context).strip(),
        robot_namespace,
    )
    package_share = get_package_share_directory("sura_teleop")
    teleop = teleop_spec(package_share, robot_model)

    config_file = LaunchConfiguration("config_file").perform(context).strip()
    if not config_file:
        config_file = teleop["config"]
    params_file = namespaced_config(config_file, robot_namespace, teleop["node_name"])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": int(LaunchConfiguration("joy_device_id").perform(context)),
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
    )

    teleop_node = Node(
        package="sura_teleop",
        executable=teleop["executable"],
        name=teleop["node_name"],
        namespace=robot_namespace,
        output="screen",
        parameters=[params_file],
    )

    return [
        joy_node,
        teleop_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="sura"),
        DeclareLaunchArgument("robot_model", default_value="auto"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        OpaqueFunction(function=launch_setup),
    ])
