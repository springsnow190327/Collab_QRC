cmake_minimum_required(VERSION 3.22)
# We only have .cpp files here; nvblox's CUDA symbols are linked from
# the prebuilt libnvblox_lib.so so we don't need nvcc to build this pkg.
project(nvblox_frontend LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(nvblox_frontend_msgs REQUIRED)

# ---------------------------------------------------------------------
# Locate nvblox: we build the core library in-tree at
# src/vendor/nvblox/build/. The Python-bindings + ROS wrapper are NOT
# used; we link directly against libnvblox_lib.so + libnvblox_gpu_hash.a.
# ---------------------------------------------------------------------
set(NVBLOX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/nvblox" CACHE PATH
    "Path to the vendored nvblox source tree")
set(NVBLOX_BUILD "${NVBLOX_ROOT}/build" CACHE PATH
    "Path to the nvblox build directory (where libnvblox_lib.so lives)")

if(NOT EXISTS "${NVBLOX_BUILD}/nvblox/libnvblox_lib.so")
  message(WARNING "nvblox_lib.so not found at ${NVBLOX_BUILD}/nvblox/. "
                  "Build nvblox first: see src/vendor/nvblox/README in this repo.")
endif()

add_library(nvblox_lib_imported SHARED IMPORTED)
set_target_properties(nvblox_lib_imported PROPERTIES
  IMPORTED_LOCATION "${NVBLOX_BUILD}/nvblox/libnvblox_lib.so"
  INTERFACE_INCLUDE_DIRECTORIES "${NVBLOX_ROOT}/nvblox/include")

add_library(nvblox_gpu_hash_imported STATIC IMPORTED)
set_target_properties(nvblox_gpu_hash_imported PROPERTIES
  IMPORTED_LOCATION "${NVBLOX_BUILD}/nvblox/libnvblox_gpu_hash.a"
  INTERFACE_INCLUDE_DIRECTORIES "${NVBLOX_ROOT}/nvblox/include")

# nvblox bundles its own glog/gflags/stdgpu as third-party FetchContent
# builds. nvblox headers inline glog macros (LOG_IF, CHECK_*, etc.), so
# any TU that includes <nvblox/nvblox.h> needs glog at link time.
add_library(nvblox_glog_imported STATIC IMPORTED)
set_target_properties(nvblox_glog_imported PROPERTIES
  IMPORTED_LOCATION "${NVBLOX_BUILD}/_deps/ext_glog-build/libglog.a"
  INTERFACE_INCLUDE_DIRECTORIES "${NVBLOX_BUILD}/_deps/ext_glog-build")

add_library(nvblox_gflags_imported STATIC IMPORTED)
set_target_properties(nvblox_gflags_imported PROPERTIES
  IMPORTED_LOCATION "${NVBLOX_BUILD}/_deps/ext_gflags-build/libgflags_nothreads.a")

# ---------------------------------------------------------------------
# Mapper node executable.
# ---------------------------------------------------------------------
add_executable(mapper_node
  src/mapper_node.cpp)

target_include_directories(mapper_node
  PRIVATE
    include
    ${NVBLOX_ROOT}/nvblox/include
    ${EIGEN3_INCLUDE_DIR})

target_link_libraries(mapper_node
  nvblox_lib_imported
  nvblox_gpu_hash_imported
  nvblox_glog_imported
  nvblox_gflags_imported
  CUDA::cudart
  Eigen3::Eigen
  unwind     # transitive dep of bundled glog when GFLAGS_HAVE_LIBUNWIND
  pthread)

ament_target_dependencies(mapper_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  nvblox_frontend_msgs)

install(TARGETS mapper_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Ship a runpath that points at the in-tree nvblox build so we don't have
# to LD_LIBRARY_PATH at every launch. The link-time location is also the
# runtime location on this machine.
set_target_properties(mapper_node PROPERTIES
  INSTALL_RPATH "${NVBLOX_BUILD}/nvblox"
  BUILD_RPATH   "${NVBLOX_BUILD}/nvblox")

ament_package()
