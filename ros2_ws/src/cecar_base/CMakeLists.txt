cmake_minimum_required(VERSION 3.5)
project(cecar_base)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cecar_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(nav_msgs)

#Create executables:
add_executable(rcu_comm
  src/rcu_comm.cpp
  src/uart.cpp
)

add_executable(drive
  src/drive.cpp
)

add_executable(keyboard_rc
  src/keyboard_rc.cpp
)

add_executable(gamepad_rc
  src/gamepad_rc.cpp
  include/gamepad/gamepad.c
)

add_executable(self_control
  src/self_control.cpp
)

add_executable(self_protection
  src/self_protection.cpp
)

add_executable(intelligence_collection
  src/intelligence_collection.cpp
)

ament_target_dependencies(rcu_comm
  "rclcpp"
  "cecar_msgs"
  "ackermann_msgs"
)

ament_target_dependencies(drive
  "rclcpp"
  "cecar_msgs"
  "ackermann_msgs"
)

ament_target_dependencies(keyboard_rc
  "rclcpp"
  "cecar_msgs"
)

ament_target_dependencies(gamepad_rc
  "rclcpp"
  "cecar_msgs"
)

ament_target_dependencies(self_control
  "rclcpp"
  "cecar_msgs"
  "ackermann_msgs"
)

ament_target_dependencies(self_protection
  "rclcpp"
  "cecar_msgs"
  "ackermann_msgs"
)

ament_target_dependencies(intelligence_collection
  "rclcpp"
  "cecar_msgs"
  "nav_msgs"
  "ackermann_msgs"
)

install(TARGETS
  rcu_comm
  drive
  keyboard_rc
  gamepad_rc
  self_control
  self_protection
  intelligence_collection
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

target_link_libraries(drive pthread)

#For ncurses:
target_link_libraries(keyboard_rc ncurses)

#For Joystick support:
target_link_libraries(gamepad_rc udev)
target_include_directories(gamepad_rc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/gamepad)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../install/cecar_msgs/include/cecar_msgs/msg/)

ament_package()
