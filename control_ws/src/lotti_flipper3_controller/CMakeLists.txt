cmake_minimum_required(VERSION 3.8)
project(lotti_flipper3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  std_msgs
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


#compile
add_library(
  lotti_flipper3_controller
  SHARED
  lotti_flipper3_controller/flipper3_controller.cpp
)

target_compile_features(lotti_flipper3_controller PUBLIC cxx_std_17)
target_include_directories(lotti_flipper3_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lotti_flipper3_controller/include/>
$<INSTALL_INTERFACE:include/lotti_flipper3_controller>
)
ament_target_dependencies(
  lotti_flipper3_controller PUBLIC
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface lotti_flipper3_controller.xml)

# INSTALL
install(
  DIRECTORY lotti_flipper3_controller/include/
  DESTINATION include/lotti_flipper3_controller
)
install(TARGETS lotti_flipper3_controller
  EXPORT export_lotti_flipper3_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_lotti_flipper3_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_package()

