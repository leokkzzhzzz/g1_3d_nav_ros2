cmake_minimum_required(VERSION 3.8)
project(g1_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_sdk2 QUIET)
find_package(bot_custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(nav_msgs REQUIRED)

set(_default_unitree_sdk2_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_sdk2")
set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}" CACHE PATH
  "Root directory of a custom Unitree SDK2 install (expects include/ and lib/)")

if(NOT UNITREE_SDK2_ROOT AND EXISTS "${_default_unitree_sdk2_root}")
  set(UNITREE_SDK2_ROOT "${_default_unitree_sdk2_root}" CACHE PATH
    "Root directory of a custom Unitree SDK2 install (expects include/ and lib/)" FORCE)
endif()

set(_unitree_include_hints)
set(_unitree_lib_hints)

if(UNITREE_SDK2_ROOT)
  list(APPEND _unitree_include_hints
    "${UNITREE_SDK2_ROOT}/include"
    "${UNITREE_SDK2_ROOT}/include/ddscxx")
  list(APPEND _unitree_lib_hints
    "${UNITREE_SDK2_ROOT}/lib"
    "${UNITREE_SDK2_ROOT}/lib/aarch64-linux-gnu")
  message(STATUS "Using custom UNITREE_SDK2_ROOT=${UNITREE_SDK2_ROOT}")
endif()

function(_find_library_prefer_sdk out_var)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs NAMES)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  unset(${out_var} CACHE)
  find_library(${out_var}
    NAMES ${ARG_NAMES}
    HINTS ${_unitree_lib_hints}
    NO_DEFAULT_PATH
  )

  if(NOT ${out_var})
    find_library(${out_var}
      NAMES ${ARG_NAMES}
      HINTS ${_unitree_lib_hints}
      PATHS /usr/local/lib /opt/ros/humble/lib /opt/ros/humble/lib/aarch64-linux-gnu
    )
  endif()

  set(${out_var} "${${out_var}}" PARENT_SCOPE)
endfunction()

find_path(UNITREE_SDK2_INCLUDE_DIR
  NAMES unitree/robot/g1/loco/g1_loco_client.hpp
  HINTS ${_unitree_include_hints}
  PATHS /usr/local/include
)

set(DDSCXX_INCLUDE_DIR "")
foreach(_candidate
    "${UNITREE_SDK2_ROOT}/include/ddscxx"
    "${UNITREE_SDK2_ROOT}/include"
    "/usr/local/include/ddscxx"
    "/usr/local/include")
  if(_candidate AND EXISTS "${_candidate}")
    set(DDSCXX_INCLUDE_DIR "${_candidate}")
    break()
  endif()
endforeach()

_find_library_prefer_sdk(UNITREE_SDK2 NAMES unitree_sdk2 libunitree_sdk2.a libunitree_sdk2.so)
_find_library_prefer_sdk(DDSCXX NAMES ddscxx libddscxx.so.0 libddscxx.so)
_find_library_prefer_sdk(DDSC NAMES ddsc libddsc.so.0 libddsc.so)

if(NOT UNITREE_SDK2_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find Unitree SDK2 headers. Set UNITREE_SDK2_ROOT or install headers under /usr/local/include.")
endif()

if(NOT DDSCXX_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find a usable CycloneDDS include directory. Set UNITREE_SDK2_ROOT or install ddscxx headers under /usr/local/include/ddscxx.")
endif()

if(NOT UNITREE_SDK2)
  message(FATAL_ERROR "Could not find libunitree_sdk2. Set UNITREE_SDK2_ROOT or install it under /usr/local/lib.")
endif()

if(NOT DDSCXX OR NOT DDSC)
  message(FATAL_ERROR "Could not find libddscxx/libddsc. Set UNITREE_SDK2_ROOT or install them under /usr/local/lib.")
endif()

get_filename_component(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2}" DIRECTORY)
get_filename_component(DDSCXX_LIB_DIR "${DDSCXX}" DIRECTORY)
get_filename_component(DDSC_LIB_DIR "${DDSC}" DIRECTORY)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/g1_driver
    ${UNITREE_SDK2_INCLUDE_DIR}
    ${DDSCXX_INCLUDE_DIR}
)

add_executable(g1_write_node 
               src/g1_driver/g1_driver.cpp
               src/g1_write_node.cpp
               src/g1_write.cpp
)

install(TARGETS g1_write_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(g1_write_node 
              rclcpp
              rclcpp_lifecycle
              bot_custom_interfaces
              geometry_msgs
              std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    config
    urdf
    meshes
    xacro
    launch
    maps
  DESTINATION
    share/${PROJECT_NAME}/
)

target_compile_definitions(g1_write_node
  PRIVATE
    G1_PKG_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(g1_write_node
  ${UNITREE_SDK2}
  ${DDSCXX}
  ${DDSC}
  pthread
)

set_target_properties(g1_write_node PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_LIB_DIR};${DDSCXX_LIB_DIR};${DDSC_LIB_DIR};/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu"
  INSTALL_RPATH "${UNITREE_SDK2_LIB_DIR};${DDSCXX_LIB_DIR};${DDSC_LIB_DIR};/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu"
)

# Python nodes and modules
install(PROGRAMS
  scripts/g1_read.py
  scripts/g1_controller_commands.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME}) # Only needed in case rosidl_generate_interfaces is not called

ament_package()
