from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("spot_description"), "urdf", "spot_with_arm.urdf"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("spot_description"), "config", "display.rviz"]
    )

    return LaunchDescription([
        # Publica o robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(Command(["xacro ", urdf_file]), value_type=str)
            }],
        ),
        # Inicia o Joint State Publisher GUI
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        # Inicia o RViz com configuração predefinida
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
