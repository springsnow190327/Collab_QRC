#!/usr/bin/python3

import argparse
import numpy as np
import open3d as o3d
from pathlib import Path
import matplotlib.pyplot as plt
import sys

parser = argparse.ArgumentParser(
    description='Visualize multiple PLY pointclouds with different colors.')
parser.add_argument('ply_paths', type=Path, nargs='+', help='Paths to the ply files to visualize.')
parser.add_argument(
    '--max-range',
    type=float,
    default=None,
    help=
    'Maximum range in meters - points beyond this distance are excluded (default: no filtering)',
)

args = parser.parse_args()

# Get colormap from matplotlib
cmap = plt.get_cmap('rainbow')

pointclouds = []
num_clouds = len(args.ply_paths)

for i, ply_path in enumerate(args.ply_paths):
    if not ply_path.exists():
        print(f'ERROR: PLY file does not exist: {ply_path}')
        sys.exit(1)

    pcd = o3d.io.read_point_cloud(str(ply_path))

    # Apply max-range filter if specified
    if args.max_range is not None:
        points = np.asarray(pcd.points)
        distances = np.linalg.norm(points, axis=1)
        mask = distances <= args.max_range
        pcd = pcd.select_by_index(np.where(mask)[0])

    # Get color from colormap (normalize index to 0-1 range)
    # For single pointcloud, use middle of colormap; for multiple, spread across colormap
    if num_clouds == 1:
        norm_idx = 0.5
    else:
        norm_idx = i / (num_clouds - 1)

    color = cmap(norm_idx)[:3]    # Get RGB, ignore alpha
    pcd.paint_uniform_color(color)
    pointclouds.append(pcd)

    print(f'Visualizing {ply_path.name} with {len(pcd.points)} points '
          f'(RGB: {tuple(int(c * 255) for c in color)})')

# Create visualizer with custom render options
vis = o3d.visualization.Visualizer()
vis.create_window()

for pcd in pointclouds:
    vis.add_geometry(pcd)

# Set render options - reduce point size
render_option = vis.get_render_option()
render_option.point_size = 1.0    # Reduced point size (default is usually 5.0)

vis.run()
vis.destroy_window()
