cmake_minimum_required(VERSION 3.5)
project(vesc_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

set(deps serial rclcpp std_msgs vesc_msgs)
    
ament_export_include_directories(include)
include_directories(
    include
    ${serial_INCLUDE_DIRS}
)


add_library(vesc_driver_lib SHARED src/vesc_driver.cpp src/vesc_interface.cpp src/vesc_packet.cpp src/vesc_packet_factory.cpp)
#target_link_libraries(vesc_driver_lib serial)
ament_target_dependencies(vesc_driver_lib ${deps})
#ament_export_targets(vesc_driver_lib)

add_executable(vesc_driver_node src/vesc_driver_node.cpp)
target_link_libraries(vesc_driver_node vesc_driver_lib serial)
#ament_target_dependencies(vesc_driver_node serial rclcpp std_msgs)

target_include_directories(vesc_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS vesc_driver_node 
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS vesc_driver_lib
        EXPORT vesc_driver_lib
  DESTINATION lib)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
