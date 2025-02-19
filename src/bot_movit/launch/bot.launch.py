
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess



def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory("bot_movit")

    # Load joint limits yaml
    joint_limits_yaml = os.path.join(pkg_share, "config", "joint_limits.yaml")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
        description="Set to True for simulation, False for real hardware"
    )
    
    is_sim = LaunchConfiguration("is_sim")
    
    moveit_config = (
        MoveItConfigsBuilder("simple_manipulator", package_name="bot_movit")
        .robot_description(file_path=os.path.join(
            pkg_share,
            "config",
            "bot_discribe.urdf.xacro")
        )
        .robot_description_semantic(file_path=os.path.join(
            pkg_share,
            "config",
            "simple_manipulator.srdf")
        )
        .trajectory_execution(file_path=os.path.join(
            pkg_share,
            "config",
            "moveit_controllers.yaml")
        )
        .robot_description_kinematics(file_path=os.path.join(
            pkg_share,
            "config",
            "kinematics.yaml")
        )
        .joint_limits(file_path=joint_limits_yaml)
        .to_moveit_configs()
    )

    # Additional parameters dictionary
    additional_params = {
        "use_sim_time": is_sim,
        "publish_robot_description_semantic": True,
        "robot_description_planning.joint_limits": moveit_config.joint_limits["robot_description_planning"]["joint_limits"]
    }

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": is_sim}
        ]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": is_sim}]
    )
    
    # MoveIt! Move Group
    moveit_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            additional_params
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )
    
    # RViz Configuration
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ]
    )

    # Move Bot Service Node
    move_bot_service_node = Node(
        package="bot_movit",
        executable="move_bot_service",
        output="screen"
    )

    # Run the Python Qt GUI
    # qt_gui_process = ExecuteProcess(
        # cmd=["python3", os.path.join(
            # "/home/naor/Desktop/naor/study/naor_task2/install/bot_movit/lib/bot_movit", "qt_gui.py")],
        # output="screen"
    # )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        moveit_group_node,
        rviz_node,
        move_bot_service_node,
        # qt_gui_process  # Ensure this runs within the launch file
    ])
