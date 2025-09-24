cmake_minimum_required(VERSION 3.8)
project(piscanner_control)

# Ausführliche Kommentare erklären jeden Schritt des Build-Prozesses.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generiert Python- und C++-Code für die Service-Definition.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfigureScan.srv"
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
