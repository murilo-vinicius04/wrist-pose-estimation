cmake_minimum_required(VERSION 3.5)
project(spot_description)

# Encontrar pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)

# Instalar arquivos URDF
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Instalar arquivos de mesh
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Instalar arquivos de lançamento
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Finalizar
ament_package()
