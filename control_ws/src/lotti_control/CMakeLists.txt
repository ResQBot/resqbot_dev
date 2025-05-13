cmake_minimum_required(VERSION 3.16)
project(lotti_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
)

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  sensor_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
#add_executable(arm_teleop lotti_control/arm_teleop.cpp)
#add_executable(chain_controller lotti_control/chain_controller.cpp)
#add_executable(flipper_controller lotti_control/flipper_controller.cpp)


#ament_target_dependencies(
#  send_trajectory PUBLIC
#  ${REF_GEN_INCLUDE_DEPENDS}
#)

add_library(
  lotti_control
  SHARED
  hardware/arm_interface.cpp
  hardware/chain_interface.cpp
  hardware/flipper_interface.cpp
  hardware/diffbot_system.cpp
  controller/arm_teleop.cpp
  controller/chain_controller.cpp
  #controller/flipper_controller.cpp
)

target_compile_features(lotti_control PUBLIC cxx_std_17)
target_include_directories(lotti_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/lotti_control>
)
ament_target_dependencies(
  lotti_control PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)
target_link_libraries(lotti_control PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lotti_control.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface lotti_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/lotti_control
)
install(
  DIRECTORY controller/include/
  DESTINATION include/lotti_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/lotti_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/lotti_control
)

install(TARGETS lotti_control
  EXPORT export_lotti_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(lotti_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_lotti_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_lotti_launch test/test_lotti_controller_launch.py)
endif()

## EXPORTS
ament_export_targets(export_lotti_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
