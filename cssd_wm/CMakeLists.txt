cmake_minimum_required(VERSION 3.5)
project(cssd_wm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cssd_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include )


add_executable(cssd_wm${target_suffix} src/cssd_wm.cpp)
ament_target_dependencies(cssd_wm${target_suffix}
  "cssd_msgs"
  "rclcpp${target_suffix}"
  "geometry_msgs")
target_link_libraries(cssd_wm ${catkin_LIBRARIES} mysqlcppconn)

install(TARGETS cssd_wm DESTINATION lib/${PROJECT_NAME})

#custom_executable(. cssd_wm)

ament_package()
