add_nvblox_executable(
  test_bench
  SOURCE_FILES
  src/test_bench.cpp
  LINK_LIBRARIES_PUBLIC
  nvblox_datasets
  nvblox_stdgpu
  nvblox_gpu_hash)
