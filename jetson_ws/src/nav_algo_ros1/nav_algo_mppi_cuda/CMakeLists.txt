cmake_minimum_required(VERSION 3.16)
project(nav_algo_mppi_cuda LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CMake 3.16 doesn't have built-in CUDA17 dialect knowledge. Add -std=c++17
# flag to nvcc directly. nvcc 12.x defaults to host-compiler's mode anyway,
# so this is mostly belt+braces to keep CUB happy.
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++17")

# Target architectures: 8.7 = Jetson Orin (Ampere), 8.9 = RTX 4050 mobile
# (host dev machine). Pre-3.18 CMake doesn't honor CUDA_ARCHITECTURES;
# pass -gencode flags directly. Add more arches by extending this list.
set(CMAKE_CUDA_FLAGS
  "${CMAKE_CUDA_FLAGS} \
   -gencode=arch=compute_87,code=sm_87 \
   --expt-relaxed-constexpr"
)

# Warnings/optim only for the host C++ compiler; nvcc would error on -Wall
# (it forwards a different syntax via -Xcompiler).
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>
                    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
                    $<$<COMPILE_LANGUAGE:CXX>:-O2>
                    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>)

find_package(catkin REQUIRED COMPONENTS roscpp nav_algo_core)

# CMake 3.16 doesn't ship FindCUDAToolkit; rely on the CUDA language built-in
# variables instead. /usr/local/cuda is the standard install prefix used by
# both this Docker image and the Orin JetPack 6 setup.
find_library(CUDART_LIB cudart
  PATHS /usr/local/cuda/lib64 ENV CUDA_HOME
  PATH_SUFFIXES lib64)
if(NOT CUDART_LIB)
  message(FATAL_ERROR "libcudart not found. Set CUDA_HOME or install cuda-cudart-dev-12-x.")
endif()
message(STATUS "CUDA runtime: ${CUDART_LIB}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_algo_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Kernel library — pure CUDA, no ROS deps.
add_library(${PROJECT_NAME} SHARED
  src/integrate.cu
  src/critics.cu
  src/control_update.cu
  src/cuda_backend.cu
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE  ON
)
# cuda_backend.cu calls Optimizer::generateNoisedTrajectoriesNoIntegrate
# (inline, but it calls protected Optimizer::updateStateVelocities which
# lives in libnav_algo_mppi.a from nav_algo_core). Whole-archive link so
# the optimizer/critic translation units aren't dead-stripped.
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${catkin_LIBRARIES}     # pulls in nav_algo_mppi + nav_algo_smac
  ${CUDART_LIB})

# Standalone benchmark — CPU reference vs GPU output diff + timing.
add_executable(integrate_bench src/integrate_bench.cpp)
target_include_directories(integrate_bench PRIVATE /usr/local/cuda/include)
target_link_libraries(integrate_bench
  ${PROJECT_NAME}
  ${CUDART_LIB}
)

# Critics bench — same structure, exercises each critic kernel against its
# CPU reference. Grows as new critics get ported.
add_executable(critics_bench src/critics_bench.cpp)
target_include_directories(critics_bench PRIVATE /usr/local/cuda/include)
target_link_libraries(critics_bench
  ${PROJECT_NAME}
  ${CUDART_LIB}
)

add_executable(control_update_bench src/control_update_bench.cpp)
target_include_directories(control_update_bench PRIVATE /usr/local/cuda/include)
target_link_libraries(control_update_bench
  ${PROJECT_NAME}
  ${CUDART_LIB}
)

install(TARGETS ${PROJECT_NAME} integrate_bench critics_bench control_update_bench
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
