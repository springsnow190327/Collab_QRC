cmake_minimum_required(VERSION 3.8)
project(go2_tare_planner_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(TARE_ORTOOLS_PROVIDER "auto" CACHE STRING "OR-Tools provider policy: auto|system|vendored")
set_property(CACHE TARE_ORTOOLS_PROVIDER PROPERTY STRINGS auto system vendored)

set(TARE_VENDORED_ORTOOLS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/generated/tare_planner/or-tools" CACHE PATH
  "Path to vendored OR-Tools tree with include/ and lib/ subfolders")
set(TARE_EXPECTED_ARCH "x86_64" CACHE STRING "Expected architecture for vendored OR-Tools")

if(NOT TARE_ORTOOLS_PROVIDER MATCHES "^(auto|system|vendored)$")
  message(FATAL_ERROR "Invalid TARE_ORTOOLS_PROVIDER='${TARE_ORTOOLS_PROVIDER}' (expected auto|system|vendored).")
endif()

set(TARE_SELECTED_ORTOOLS_PROVIDER "none")
set(TARE_USE_SYSTEM_ORTOOLS FALSE)
set(TARE_USE_VENDORED_ORTOOLS FALSE)

if(TARE_ORTOOLS_PROVIDER STREQUAL "auto" OR TARE_ORTOOLS_PROVIDER STREQUAL "system")
  find_package(ortools QUIET)
endif()

set(TARE_VENDORED_TREE_OK FALSE)
if(EXISTS "${TARE_VENDORED_ORTOOLS_ROOT}/include" AND EXISTS "${TARE_VENDORED_ORTOOLS_ROOT}/lib")
  set(TARE_VENDORED_TREE_OK TRUE)
endif()

if(TARE_ORTOOLS_PROVIDER STREQUAL "system")
  if(NOT ortools_FOUND)
    message(FATAL_ERROR "TARE_ORTOOLS_PROVIDER=system requested but ortools was not found.")
  endif()
  set(TARE_SELECTED_ORTOOLS_PROVIDER "system")
  set(TARE_USE_SYSTEM_ORTOOLS TRUE)
elseif(TARE_ORTOOLS_PROVIDER STREQUAL "vendored")
  if(NOT TARE_VENDORED_TREE_OK)
    message(FATAL_ERROR
      "TARE_ORTOOLS_PROVIDER=vendored requested but vendored tree is missing: ${TARE_VENDORED_ORTOOLS_ROOT}")
  endif()
  set(TARE_SELECTED_ORTOOLS_PROVIDER "vendored")
  set(TARE_USE_VENDORED_ORTOOLS TRUE)
else()
  if(ortools_FOUND)
    set(TARE_SELECTED_ORTOOLS_PROVIDER "system")
    set(TARE_USE_SYSTEM_ORTOOLS TRUE)
  elseif(TARE_VENDORED_TREE_OK)
    set(TARE_SELECTED_ORTOOLS_PROVIDER "vendored")
    set(TARE_USE_VENDORED_ORTOOLS TRUE)
  else()
    message(WARNING
      "No OR-Tools provider found in auto mode; proceeding with stub planner node only.")
  endif()
endif()

if(TARE_USE_VENDORED_ORTOOLS)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" TARE_SYSTEM_ARCH)
  string(TOLOWER "${TARE_EXPECTED_ARCH}" TARE_EXPECTED_ARCH_LOWER)
  if(NOT TARE_SYSTEM_ARCH MATCHES "x86_64|amd64")
    message(FATAL_ERROR
      "Vendored OR-Tools policy currently supports x86_64 only. "
      "Use TARE_ORTOOLS_PROVIDER=system for other architectures.")
  endif()
  if(NOT TARE_EXPECTED_ARCH_LOWER MATCHES "x86_64|amd64")
    message(FATAL_ERROR "TARE_EXPECTED_ARCH must currently be x86_64 for vendored policy.")
  endif()
endif()

add_executable(tare_planner_node src/tare_planner_node.cpp)
target_compile_features(tare_planner_node PUBLIC cxx_std_17)
ament_target_dependencies(
  tare_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)
target_compile_definitions(
  tare_planner_node
  PRIVATE
    TARE_ORTOOLS_PROVIDER_STR="${TARE_SELECTED_ORTOOLS_PROVIDER}"
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated")
  install(DIRECTORY generated DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS tare_planner_node DESTINATION lib/${PROJECT_NAME})

message(STATUS "go2_tare_planner_ros2: OR-Tools provider selected: ${TARE_SELECTED_ORTOOLS_PROVIDER}")

ament_package()
