from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("bluerov_description"),
        "config", "bluerov.rviz"
    ])

    urdf_file_fish = PathJoinSubstitution([
        FindPackageShare("bluerov_description"),
        "urdf", "bluerov", "bluerov.urdf.xacro"
    ])

    description_file_cirtesu = PathJoinSubstitution([
        FindPackageShare("bluerov_description"),
        "urdf","cirtesu","cirtesu.urdf.xacro"
    ])

    robot_description_cirtesu = Command(["xacro ", description_file_cirtesu])
    
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world2ned",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "3.1415",
            "--frame-id",
            "world",
            "--child-frame-id",
            "world_ned",
        ],
        output="screen",
    )

    cirtesu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "3.1416",
            "--yaw", "0",
            "--frame-id", "world_ned",
            "--child-frame-id", "cirtesu_tank"
        ]
    )

    rsp_node_bluerov = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_bluerov",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_file_fish])
        }],
        remappings=[
            ("/joint_states", "/joint_states"),
            ("/robot_description", "/bluerov/robot_description")
        ],
    )

    rsp_node_cirtesu = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher_cirtesu',
        remappings=[('/robot_description', '/cirtesu/robot_description')],
        parameters=[{
            'robot_description': robot_description_cirtesu
        }]
    )

    return LaunchDescription([
        rsp_node_bluerov,
        rsp_node_cirtesu,
        static_transform_publisher_node,
        cirtesu_static_tf
    ])
