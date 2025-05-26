cmake_minimum_required(VERSION 3.8.0)
project(gz_misil_plugin)

# Establecer C++14 como estándar por defecto
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-physics7   REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(gz_uwb_beacon_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)

# Crear la biblioteca del plugin
add_library(${PROJECT_NAME} SHARED src/gz_misil_plugin.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencias
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  gz-sim7
  gz-plugin2
  gz-common5
  gz-math7
  gz-transport12
  gz-physics7
  ros_gz_bridge
  std_msgs
  geometry_msgs
  visualization_msgs
  # gz_uwb_beacon_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  px4_msgs
)

# Instalar el plugin
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Exportar dependencias
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  gz-sim7
  gz-plugin2
  gz-common5
  gz-math7
  gz-transport12
  gz-physics7
  ros_gz_bridge
  std_msgs
  geometry_msgs
  visualization_msgs
  # gz_uwb_beacon_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  px4_msgs
  rosidl_default_runtime
)

ament_package()
