from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory  # Importação necessária
import os

def generate_launch_description():
    # Path do pacote realsense2_camera
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        # 🖼️ Lançamento da câmera RealSense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'depth_module.profile': '640x480x30',
                'color_module.profile': '640x480x30',
                'enable_color': 'true',
                'enable_depth': 'true',
                'align_depth.enable': 'true'
            }.items()
        ),

        # 🦾 Visualizador do braço direito
        Node(
            package='blazepose_right_arm_3d',
            executable='right_arm_viewer',
            name='right_arm_viewer_node',
            output='screen'
        ),


        # Novo nó pointcloud_test
        Node(
            package='blazepose_right_arm_3d',
            executable='pointcloud_test',
            name='pointcloud_test_node',
            output='screen'
        ),

        # # Novo nó aruco_tf_node
        # Node(
        #     package='blazepose_right_arm_3d',
        #     executable='aruco_tf_node',
        #     name='aruco_tf_node',
        #     output='screen'
        # ),

        # # Novo nó wrist_transform_node
        # Node(
        #     package='blazepose_right_arm_3d',
        #     executable='wrist_transform_node',
        #     name='wrist_transform_node',
        #     output='screen'
        # ),

        # # Novo nó blazepose_map3d
        # Node(
        #     package='blazepose_right_arm_3d',
        #     executable='blazepose_3d',
        #     name='blazepose_map3d_node',
        #     output='screen'
        # ),
    ])
