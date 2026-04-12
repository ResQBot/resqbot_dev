cmake_minimum_required(VERSION 3.16)
project(lotti_control3 LANGUAGES CXX)

option(
  LOTTI_ENABLE_VENDOR_UNITREE_SDK
  "Link the optional vendor Unitree SDK for the drive interface transport."
  OFF
)

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
  rclcpp_lifecycle
)

# Find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_library(LIBSERIAL_LIBRARY NAMES serial libserial REQUIRED)

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
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  lotti_control3 PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

target_link_libraries(lotti_control3 PUBLIC ${LIBSERIAL_LIBRARY})

if(LOTTI_ENABLE_VENDOR_UNITREE_SDK)
  set(UNITREE_SDK_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Linux64.so")

  if(NOT EXISTS "${UNITREE_SDK_LIB}")
    message(FATAL_ERROR
      "LOTTI_ENABLE_VENDOR_UNITREE_SDK is ON, but ${UNITREE_SDK_LIB} was not found. "
      "Provide the vendor SDK in control_ws/src/lotti_control3/lib/ or disable the option."
    )
  endif()

  target_link_libraries(lotti_control3 PUBLIC ${UNITREE_SDK_LIB})
  message(STATUS "Building lotti_control3 with vendor Unitree SDK support enabled.")
else()
  message(STATUS "Building lotti_control3 without the vendor Unitree SDK. The drive interface stays in stub mode.")
endif()

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lotti_control3.xml)

# Install rules
install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
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

install(
  FILES lotti_control3.xml
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
  message(STATUS "No package-local tests are currently registered for lotti_control3.")
endif()

# Export targets and dependencies
ament_export_targets(export_lotti_control3 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})

ament_package()
