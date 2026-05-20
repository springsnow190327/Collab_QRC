# This file is adapted from the STDGPU repo: https://github.com/stotko/stdgpu If
# CMAKE_CUDA_ARCHITECTURES is not set externaly, we'll set it based on the
# native GPU based on
# http://stackoverflow.com/questions/2285185/easiest-way-to-test-for-existence-of-cuda-capable-gpu-from-cmake/2297877#2297877
# (Christopher Bruns)

# Detect the native compute architecture if not set externally
if(NOT ${CMAKE_CUDA_ARCHITECTURES_SET_EXTERNALLY})
  set(CUDA_COMPUTE_CAPABILITIES_SOURCE
      "${CMAKE_CURRENT_LIST_DIR}/compute_capability.cpp")
  message(STATUS "Detecting CCs of GPUs : ${CUDA_COMPUTE_CAPABILITIES_SOURCE}")

  find_package(CUDAToolkit REQUIRED QUIET MODULE)

  try_run(
    RUN_RESULT_VAR COMPILE_RESULT_VAR ${CMAKE_BINARY_DIR}
    ${CUDA_COMPUTE_CAPABILITIES_SOURCE} LINK_LIBRARIES CUDA::cudart
    COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
    RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)

  # COMPILE_RESULT_VAR is TRUE when compile succeeds RUN_RESULT_VAR is zero when
  # a GPU is found
  if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
    message(
      STATUS
        "Detecting CCs of GPUs : ${CUDA_COMPUTE_CAPABILITIES_SOURCE} - Success (found CCs : ${RUN_OUTPUT_VAR})"
    )
    set(CMAKE_CUDA_ARCHITECTURES
        ${RUN_OUTPUT_VAR}
        CACHE STRING "Compute capabilities of CUDA-capable GPUs" FORCE)
    mark_as_advanced(CUDA_COMPUTE_CAPABILITIES)
  elseif(NOT COMPILE_RESULT_VAR)
    message(
      FATAL_ERROR
        "ERROR: Detecting CCs of GPUs : ${CUDA_COMPUTE_CAPABILITIES_SOURCE} - Failed to compile"
    )
  else()
    message(
      FATAL_ERROR
        "ERROR: Detecting CCs of GPUs : ${CUDA_COMPUTE_CAPABILITIES_SOURCE} - No CUDA-capable GPU found. Use CMAKE_CUDA_ARCHITECTURES=XY to set target architecture"
    )
  endif()

  message(STATUS "Building for cuda architectues: ${CMAKE_CUDA_ARCHITECTURES}")
endif()

# We need to provide cuda architectures to torch as well. TORCH_CUDA_ARCH_LIST
# require the numbers to be dot separated, e.g. "8.6" instead of "86".
set(TORCH_CUDA_ARCH_LIST "")
# Loop through each number in the list
foreach(ARCH IN LISTS CMAKE_CUDA_ARCHITECTURES)

  # The numbers can be 2-3 digits, e.g. 90 -> 9.0 or 120 -> 1.20
  string(LENGTH "${ARCH}" num_digits)

  string(SUBSTRING "${ARCH}" 0 1 FIRST_DIGIT)
  string(SUBSTRING "${ARCH}" 1 1 SECOND_DIGIT)
  if(num_digits EQUAL 2)
    set(DOT_SEPARATED "${FIRST_DIGIT}.${SECOND_DIGIT}")
  else()
    # Currently, Caffe2 only support Cuda architectues up to 9.0
    message(
      WARNING
        "Cuda architecture ${ARCH} not supported for nvblox_torch. Reverting to 9.0 and hoping for backward compatibility"
    )
    set(DOT_SEPARATED "9.0")
  endif()
  # Append to the result list
  list(APPEND TORCH_CUDA_ARCH_LIST "${DOT_SEPARATED}")
endforeach()

message(STATUS "TORCH_CUDA_ARCH_LIST: ${TORCH_CUDA_ARCH_LIST}")
