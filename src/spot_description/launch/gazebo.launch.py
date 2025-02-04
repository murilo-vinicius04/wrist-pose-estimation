from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Diretório do pacote e arquivo URDF
    pkg_share = FindPackageShare('spot_description').find('spot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'spot_with_arm.urdf')

    # Carrega o conteúdo do arquivo URDF
    with open(urdf_file, 'r') as file:
        robot_urdf = file.read()

    # Publica o robot_description como um tópico
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
        output='screen'
    )

    # Publica os estados das juntas (se necessário)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Inclui o servidor do Gazebo
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'pause': 'false'}.items()
    )

    # Inclui o cliente do Gazebo
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Configuração para spawnar o robô no Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'spot', '-topic', 'robot_description'],
        output='screen'
    )

    # Retorna a descrição completa do lançamento
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_entity_node,
    ])
