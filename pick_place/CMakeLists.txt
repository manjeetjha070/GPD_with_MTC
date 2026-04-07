cmake_minimum_required(VERSION 3.8)
project(pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gpd_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(mtc_node 
    src/mtc_task_node.cpp
    src/main.cpp
)


target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(
  mtc_node
  "moveit_task_constructor_core"
  "rclcpp"
  "tf2_ros"
  "visualization_msgs"
  "geometry_msgs"
  "gpd_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )



ament_package()
