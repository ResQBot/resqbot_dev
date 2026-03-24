cmake_minimum_required(VERSION 3.16)
project(lotti_control3 LANGUAGES CXX)

# Include and lib directories
include_directories(include)
link_directories(lib)

# Unitree SDK library selection based on architecture
set(UNITREE_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Linux64.so")

# Output path for libraries
set(LIBRARY_OUTPUT_PATH "../lib")

# Compiler options for GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Ensure Windows export all symbols (for DLLs)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Hardware interface dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
)

# Find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Your main shared library
add_library(
  lotti_control3
  SHARED
  hardware/arm3_interface.cpp
  hardware/flipper3_interface.cpp
  hardware/drive3_interface.cpp
)

# Compile features and includes
target_compile_features(lotti_control3 PUBLIC cxx_std_17)
target_include_directories(lotti_control3 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/lotti_control3>
)

ament_target_dependencies(
  lotti_control3 PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

# Link serial and Unitree SDK library
target_link_libraries(lotti_control3 PUBLIC serial ${UNITREE_SDK_LIB})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lotti_control3.xml)

# Install rules
install(
  DIRECTORY hardware/include/
  DESTINATION include/lotti_control3
)

install(
  DIRECTORY description/launch description/urdf description/srdf description/meshes description/worlds
  DESTINATION share/lotti_control3/description
)

install(
  DIRECTORY description/ros2_control 
  DESTINATION share/lotti_control3
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/lotti_control3
)

install(TARGETS lotti_control3
  EXPORT export_lotti_control3
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(lotti_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_lotti_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_lotti_launch test/test_lotti_controller_launch.py)
endif()

# Export targets and dependencies
ament_export_targets(export_lotti_control3 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})

ament_package()
