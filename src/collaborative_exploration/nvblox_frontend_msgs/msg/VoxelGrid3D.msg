# Sparse 3D occupancy grid for frontier exploration.
#
# Layout: dense int8 array, indexed as data[z * size_x * size_y + y * size_x + x].
# Each cell is:  -1 = unknown (voxel not allocated in nvblox layer)
#                 0 = free    (occupancy log_odds < 0)
#               100 = occupied (occupancy log_odds > 0)
#
# This is intentionally aligned with nav_msgs/OccupancyGrid value semantics so
# downstream consumers (CFPA2 IG sampler) can reuse the same unknown_value
# convention across 2D and 3D code paths.

std_msgs/Header header

# World-frame position of voxel (0, 0, 0)'s corner. The voxel center is at
# origin + 0.5*voxel_size*(1,1,1).
geometry_msgs/Point origin

# Edge length of each voxel, in meters.
float32 voxel_size

# Grid dimensions.
uint32 size_x
uint32 size_y
uint32 size_z

# Row-major dense data. Total length == size_x * size_y * size_z.
int8[] data
