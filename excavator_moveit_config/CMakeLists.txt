cmake_minimum_required(VERSION 3.8)
project(excavator_moveit_config)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
