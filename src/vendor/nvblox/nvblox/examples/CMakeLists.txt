# ESDF query executable
add_nvblox_executable(
  esdf_query
  SOURCE_FILES
  src/esdf_query.cu
  LINK_LIBRARIES_PUBLIC
  nvblox_lib
  LINK_LIBRARIES_PRIVATE
  nvblox_stdgpu)

# An example for map loading
add_nvblox_executable(
  load_map_and_mesh
  SOURCE_FILES
  src/load_map_and_mesh.cpp
  LINK_LIBRARIES_PUBLIC
  nvblox_lib
  nvblox_stdgpu
  nvblox_gpu_hash)
