from setuptools import setup
from glob import glob

package_name = 'blazepose_right_arm_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluindo todos os arquivos .py da pasta launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu.email@exemplo.com',
    description='Pacote para detectar e publicar landmarks 3D do braço direito usando BlazePose',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blazepose_node = blazepose_right_arm_3d.blazepose_right_arm_3d_node:main',
            'pointcloud_test = blazepose_right_arm_3d.pointcloud_test:main',
            'right_arm_viewer = blazepose_right_arm_3d.right_arm_viewer:main',
            'wrist_transform_node = blazepose_right_arm_3d.tf2:main',
            'blazepose_3d = blazepose_right_arm_3d.blazepose_map3d:main',
            'calibration_node = blazepose_right_arm_3d.calibration:main',
        ],
    },
)
