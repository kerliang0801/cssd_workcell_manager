cmake_minimum_required(VERSION 3.5)
project(cssd_workcell_manager)

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
find_package(rmf_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include )


add_executable(cssd_workcell_manager${target_suffix} src/cssd_workcell_manager.cpp)
ament_target_dependencies(cssd_workcell_manager${target_suffix}
  "rclcpp${target_suffix}"
  "rmf_msgs"
  "geometry_msgs")
target_link_libraries(cssd_workcell_manager ${catkin_LIBRARIES} mysqlcppconn)

install(TARGETS cssd_workcell_manager DESTINATION lib/${PROJECT_NAME})

#custom_executable(. cssd_wm)

ament_package()
