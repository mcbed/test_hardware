import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    urdf_file_name = 'config/dint.urdf'
    urdf = os.path.join(
        get_package_share_directory('double_integrator_hardware'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {"robot_description": robot_description_content}

    dint_controllers = PathJoinSubstitution(
        [
            FindPackageShare("double_integrator_hardware"),
            "config",
            "dint_controllers.yaml",
        ]
    )

    # node_robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[robot_description],
    # )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, dint_controllers],
        output="screen",
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_jt_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mpi_controller"],
        output="screen",
    )

    nodes = [
        controller_manager_node,
        # node_robot_state_publisher,
        spawn_jsb_controller,
        spawn_jt_controller,
    ]
    return LaunchDescription(nodes)
