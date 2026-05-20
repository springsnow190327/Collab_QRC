cmake_minimum_required(VERSION 3.18)
project(nav2_mppi_controller_cuda_plugin LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# ── CUDA architectures ──────────────────────────────────────────────────────
# 87 = Jetson Orin NX / Nano (Ampere)
# 89 = RTX 4050 mobile (Ada)
# 120 = RTX 5090 (Blackwell) — only on CUDA 12.8+; auto-omitted on 12.6/12.7.
# 89-virtual retained as a PTX-only forward-compat path so older toolkits
# can still JIT-launch on Blackwell at runtime.
if(${CMAKE_CUDA_COMPILER_VERSION} VERSION_GREATER_EQUAL 12.8)
  set(CMAKE_CUDA_ARCHITECTURES 87-real 89-real 89-virtual 120-real)
else()
  set(CMAKE_CUDA_ARCHITECTURES 87-real 89-real 89-virtual)
endif()
message(STATUS "nav2_mppi_controller_cuda_plugin: CUDA arches = ${CMAKE_CUDA_ARCHITECTURES}")

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_mppi_controller_cuda REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Don't call nav2_package(): it issues global -Werror=deprecated etc. that
# nvcc rejects (`Value '-Wdeprecated' is not defined for option 'Werror'`).
# We set our own Release defaults + CXX-only warnings via generator
# expressions on the target below.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

set(dependencies_pkgs
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_mppi_controller_cuda
  geometry_msgs
  nav_msgs
  tf2_ros
)

# ── Compile the CUDA kernels straight from the catkin-based source tree.
#    We compile each .cu file as a colcon-managed target source rather than
#    depending on the catkin package's own build. The kernels are pure CUDA
#    (no ROS deps) so this is portable; only cuda_backend.cu touches the
#    Optimizer header, and the NAV_ALGO_MPPI_CUDA_USE_NAV2 define below
#    routes that include to the patched nav2 header.
set(NAV_ALGO_MPPI_CUDA_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../vendor/nav_algo_ros1/nav_algo_mppi_cuda)

add_library(cuda_mppi_controller SHARED
  src/cuda_mppi_controller.cpp
  ${NAV_ALGO_MPPI_CUDA_DIR}/src/cuda_backend.cu
  ${NAV_ALGO_MPPI_CUDA_DIR}/src/critics.cu
  ${NAV_ALGO_MPPI_CUDA_DIR}/src/control_update.cu
  ${NAV_ALGO_MPPI_CUDA_DIR}/src/integrate.cu
)

set_target_properties(cuda_mppi_controller PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE  ON
)

target_include_directories(cuda_mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${NAV_ALGO_MPPI_CUDA_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# cuda_runtime.h: needed by both host .cpp and device .cu sources. nvcc
# auto-finds it for .cu files but the host compiler doesn't unless we
# point it at CUDA::cudart's include dir explicitly.
target_include_directories(cuda_mppi_controller PRIVATE
  ${CUDAToolkit_INCLUDE_DIRS}
)

# Host-only warning flags (matches what nav2_package() would have set).
# Wrapped so nvcc-driven CUDA translation units don't see -Wdeprecated etc.
target_compile_options(cuda_mppi_controller PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wall>
  $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
  $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  $<$<COMPILE_LANGUAGE:CXX>:-fPIC>
)

# All sources (incl. cuda_backend.cu + cuda_mppi_controller.cpp) MUST see
# the same nav2-flavoured ICudaBackend / ParametersHandler so there's no
# ODR mismatch between .cu and .cpp translation units linked into this .so.
target_compile_definitions(cuda_mppi_controller PRIVATE NAV_ALGO_MPPI_CUDA_USE_NAV2)

# Optional probe sentinels — touches /tmp/cuda_backend_* files so a smoke
# test can confirm the GPU path is being hit. OFF by default; flip on per
# request: colcon build --cmake-args -DNAV_ALGO_CUDA_PROBE=ON
option(NAV_ALGO_CUDA_PROBE "Emit /tmp/cuda_backend_* probe files" OFF)
if(NAV_ALGO_CUDA_PROBE)
  target_compile_definitions(cuda_mppi_controller PRIVATE NAV_ALGO_CUDA_PROBE)
endif()

ament_target_dependencies(cuda_mppi_controller ${dependencies_pkgs})

# Explicit cudart link — nvcc already implicitly links it for executables
# but a shared lib needs it stated so downstream loaders don't ENOENT.
find_library(CUDART_LIB cudart
  PATHS /usr/local/cuda/lib64 ENV CUDA_HOME
  PATH_SUFFIXES lib64)
if(NOT CUDART_LIB)
  message(FATAL_ERROR
    "libcudart not found. Set CUDA_HOME or install cuda-cudart-dev-12-x.")
endif()
target_link_libraries(cuda_mppi_controller ${CUDART_LIB})

# ── Install ─────────────────────────────────────────────────────────────────
install(TARGETS cuda_mppi_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

pluginlib_export_plugin_description_file(nav2_core mppic_cuda.xml)

ament_export_include_directories(include)
ament_export_libraries(cuda_mppi_controller)
ament_export_dependencies(${dependencies_pkgs})

ament_package()
