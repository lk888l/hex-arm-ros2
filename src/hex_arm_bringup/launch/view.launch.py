from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uuid import uuid4


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    # Each preview owns its state and TF tree, including when several windows
    # or a real/simulated robot are running in the same ROS domain.
    namespace = "hex_arm_view_" + uuid4().hex[:12]
    remappings = [
        ("/joint_states", f"/{namespace}/joint_states"),
        ("/robot_description", f"/{namespace}/robot_description"),
        ("/tf", f"/{namespace}/tf"),
        ("/tf_static", f"/{namespace}/tf_static"),
    ]
    model = PathJoinSubstitution([FindPackageShare("hex_arm_description"), "urdf", "firefly_y6.urdf.xacro"])
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", model, " backend:=view"])
    }
    rviz_config = PathJoinSubstitution([FindPackageShare("hex_arm_bringup"), "rviz", "firefly_y6.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=namespace, remappings=remappings, parameters=[robot_description]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             namespace=namespace, remappings=remappings),
        Node(package="rviz2", executable="rviz2", namespace=namespace,
             remappings=remappings, arguments=["-d", rviz_config], condition=IfCondition(use_rviz)),
    ])
