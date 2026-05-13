cmake_minimum_required(VERSION 3.8)
project(gpd_ros)

# -----------------------------
# Compiler settings
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

# ------------------------------------------------------------------------------
# Find dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------------------------------------------
# External GPD library
# ------------------------------------------------------------------------------

find_library(GPD_LIB
  NAMES gpd
  PATHS /home/pg/rrs_ss25/manjeet/libs/gpd_install/
  #PATHS /usr/local/lib
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH
)

if(NOT GPD_LIB)
  message(FATAL_ERROR "Library GPD not found")
endif()
set (GPD_INCLUDE_DIRS /home/pg/rrs_ss25/manjeet/libs/gpd_install/include)


# ------------------------------------------------------------------------------
# Custom messages/services
# ------------------------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudIndexed.msg"
  "msg/CloudSamples.msg"
  "msg/CloudSources.msg"
  "msg/GraspConfig.msg"
  "msg/GraspConfigList.msg"
  "msg/SamplesMsg.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)
# Link generated messages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ------------------------------------------------------------------------------
# grasp_messages
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_grasp_messages
  src/grasp_messages.cpp
)

# Include dirs for libraries
target_include_directories(${PROJECT_NAME}_grasp_messages PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GPD_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_grasp_messages
  ${cpp_typesupport_target}
  ${GPD_LIB}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_grasp_messages
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_eigen
)


# ------------------------------------------------------------------------------
# grasp_plotter
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_grasp_plotter
  src/grasp_plotter.cpp
)

target_include_directories(${PROJECT_NAME}_grasp_plotter PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${GPD_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_grasp_plotter
  ${GPD_LIB}
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}

)

ament_target_dependencies(${PROJECT_NAME}_grasp_plotter
  rclcpp
  visualization_msgs
)

# ------------------------------------------------------------------------------
# detect_grasps node
# ------------------------------------------------------------------------------
add_executable(detect_grasps
  src/grasp_detection_node.cpp
  src/main.cpp
)

target_include_directories(detect_grasps PRIVATE
  include
  ${GPD_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(detect_grasps
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  ${GPD_LIB}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(detect_grasps
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  std_srvs
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------

install(TARGETS
  detect_grasps
  ${PROJECT_NAME}_grasp_messages
  ${PROJECT_NAME}_grasp_plotter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()