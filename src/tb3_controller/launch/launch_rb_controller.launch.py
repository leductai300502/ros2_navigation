from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("tb3_description"), "urdf", "turtlebot3_waffle.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    twist_mux_params = os.path.join(get_package_share_directory("tb3_controller"),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')]
        )

    controller_params_file = PathJoinSubstitution(
        [
            FindPackageShare("tb3_controller"),
            "config",
            "diff_drive_controllers.yaml",
        ]
    )  

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )


    # Maybe need this because robot state publisher must finish starting up before control manager
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )


    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # static_transform_publisher_node = Node(
    # package='tf2_ros',
    # executable='static_transform_publisher',
    # name='link1_broadcaster',
    # arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    # output='screen',
    # )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        twist_mux,
        controller_manager,
        robot_state_pub_node,
        #static_transform_publisher_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ])