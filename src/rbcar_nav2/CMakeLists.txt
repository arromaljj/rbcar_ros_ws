cmake_minimum_required(VERSION 3.8)
project(rbcar_nav2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(rviz2 REQUIRED)

install(
  DIRECTORY launch param rviz config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 