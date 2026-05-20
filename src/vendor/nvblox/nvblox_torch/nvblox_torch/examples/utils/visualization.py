# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#

from typing import Tuple, Optional, Dict
import types
import cv2
import open3d as o3d
import numpy as np
import numpy.typing as npt
import torch
import time

from nvblox_torch.visualization import get_open3d_coordinate_frame
from nvblox_torch.mesh import FeatureMesh, ColorMesh

PCAParams = Tuple[torch.Tensor, torch.Tensor, torch.Tensor]


class ViewPointController:
    """
    Stores and manages camera viewpoint information for Open3D visualization.

    Supports interactive viewpoint control and maintains view across updates.
    """

    def __init__(self, lookat: npt.NDArray[np.float64]) -> None:
        self.lookat = lookat
        self.up: npt.NDArray[np.float64] = np.array([0, -1, 0])
        self.front: npt.NDArray[np.float64] = np.array([0, 0, 1])
        self.zoom: float = 0.7
        self._camera_params: Optional[o3d.camera.PinholeCameraParameters] = None
        self._view_initialized: bool = False

    def store_camera_pose(self, visualizer: o3d.visualization.Visualizer) -> None:
        """Store the current camera viewpoint after user interaction."""
        view_control = visualizer.get_view_control()
        self._camera_params = view_control.convert_to_pinhole_camera_parameters()
        self._view_initialized = True

    def restore_viewpoint(self, visualizer: o3d.visualization.Visualizer) -> None:
        """Restore the stored viewpoint after adding new geometry."""
        view_control = visualizer.get_view_control()

        if not self._view_initialized:
            # First time setup of view
            view_control.set_lookat(self.lookat)
            view_control.set_up(self.up)
            view_control.set_front(self.front)
            view_control.set_zoom(self.zoom)
            view_control.camera_local_translate(0, 0, 1.0)
            self._view_initialized = True
        elif self._camera_params is not None:
            # Restore previous interactive view
            view_control.convert_from_pinhole_camera_parameters(self._camera_params, True)


class Visualizer:
    """
    Visualizer for 3D reconstruction with optional feature visualization.

    This class provides functionality to visualize:
    - Color meshes
    - Feature meshes
    - Camera viewpoints

    To visulize a feature mesh the color performs a PCA on the features and uses the
    first three principal components as RGB colors.

    """

    visualizers: Dict[str, o3d.visualization.VisualizerWithKeyCallback]
    view_controllers: Dict[str, ViewPointController]
    embedding_dim: Optional[int]
    visualize_features: bool
    pca_params: Optional[PCAParams]
    color_bounds_initialized: bool
    camera_pose_per_visualizer: Dict[str, torch.Tensor]

    def __init__(self, deep_feature_embedding_dim: Optional[int] = None) -> None:
        """Visualize reconstruction results.

        Args:
            deep_feature_embedding_dim: Dimension of feature embeddings
        """
        self.visualizers = {}
        self.view_controllers = {}
        self.pca_params = None    # Store PCA params from first frame
        self.color_bounds_initialized = False    # Track if we've initialized bounds
        self.embedding_dim = deep_feature_embedding_dim
        self.camera_pose_per_visualizer = {}
        self.pause = False

    def visualize(self,
                  color_mesh: Optional[ColorMesh] = None,
                  feature_mesh: Optional[FeatureMesh] = None,
                  camera_pose: Optional[torch.Tensor] = None) -> None:
        """
        Visualize reconstruction outputs. Maintains camera viewpoint across updates.

        Args:
            color_mesh: Color mesh to visualize
            feature_mesh: Feature mesh to visualize
            camera_pose: Camera pose to visualize
        """
        # Initialize view controllers if needed
        self._initialize_view_controllers_if_needed()

        # Store current views before updating
        for name, visualizer in self.visualizers.items():
            if name in self.view_controllers:
                self.view_controllers[name].store_camera_pose(visualizer)

        if color_mesh is not None:
            self._visualize_nvblox_mesh(color_mesh)

        if feature_mesh is not None:
            self._visualize_nvblox_feature_mesh(feature_mesh)

        if camera_pose is not None:
            self._visualize_camera_pose(camera_pose)

        # Restore views and update
        for name, visualizer in self.visualizers.items():
            if name in self.view_controllers:
                self.view_controllers[name].restore_viewpoint(visualizer)
            self._update_visualization(visualizer)

        # Handle pausing
        if self.pause:
            self._loop_while_paused()

    def __enter__(self) -> 'Visualizer':
        return self

    def __exit__(self, exc_type: Optional[type], exc_value: Optional[Exception],
                 traceback: Optional[types.TracebackType]) -> None:
        for visualizer in self.visualizers:
            visualizer.destroy_window()
        cv2.destroyAllWindows()

    def _update_visualization(self,
                              visualizer: o3d.visualization.VisualizerWithKeyCallback) -> None:
        visualizer.poll_events()
        visualizer.update_renderer()

    def _create_visualizer(self, window_name: str) -> o3d.visualization.VisualizerWithKeyCallback:
        visualizer = o3d.visualization.VisualizerWithKeyCallback()
        visualizer.create_window(width=800, height=600, window_name=window_name)
        visualizer.get_render_option().line_width = 50
        visualizer.get_render_option().point_size = 3
        visualizer.get_render_option().background_color = np.asarray([0, 0, 0])
        visualizer.register_key_callback(ord(' '), lambda vis: self._toggle_pause(vis))
        return visualizer

    def _toggle_pause(self, _: o3d.visualization.VisualizerWithKeyCallback) -> None:
        # Toggle the pause state
        if self.pause:
            self.pause = False
        else:
            self.pause = True

    def _loop_while_paused(self) -> None:
        while self.pause:
            for visualizer in self.visualizers.values():
                visualizer.poll_events()
                visualizer.update_renderer()
            time.sleep(0.01)

    def _visualize_nvblox_mesh(self, color_mesh: ColorMesh, name: str = 'color_mesh') -> None:
        if name not in self.visualizers:
            self.visualizers[name] = self._create_visualizer(name)
        self.visualizers[name].clear_geometries()
        self.visualizers[name].add_geometry(color_mesh.to_open3d())
        self.visualizers[name].update_renderer()

    def _visualize_camera_pose(self, camera_pose: torch.Tensor) -> None:
        # Add the camera pose to all visualizers
        for name, visualizer in self.visualizers.items():
            # We first have to remove the camera pose from the last time step
            if name in self.camera_pose_per_visualizer:
                visualizer.remove_geometry(self.camera_pose_per_visualizer[name])
            camera_frame_o3d = get_open3d_coordinate_frame(camera_pose)
            self.camera_pose_per_visualizer[name] = camera_frame_o3d
            visualizer.add_geometry(self.camera_pose_per_visualizer[name])
            visualizer.update_renderer()

    def _visualize_nvblox_feature_mesh(self,
                                       feature_mesh: FeatureMesh,
                                       name: str = 'feature_mesh') -> None:
        """
        Visualize the current NVblox 3D feature grid as a pointcloud with PCA-projected colors.
        """
        if name not in self.visualizers:
            self.visualizers[name] = self._create_visualizer(name)
        self.visualizers[name].clear_geometries()

        # PCA projection of the features to RGB colors.
        feature_colors = self._get_normalized_colors_from_features(feature_mesh.vertex_features())

        # To open3d
        mesh_o3d = o3d.geometry.TriangleMesh()
        mesh_o3d.vertices = o3d.utility.Vector3dVector(feature_mesh.vertices().cpu().numpy())
        mesh_o3d.vertex_colors = o3d.utility.Vector3dVector(feature_colors)
        mesh_o3d.triangles = o3d.utility.Vector3iVector(feature_mesh.triangles().cpu().numpy())

        self.visualizers[name].add_geometry(mesh_o3d)
        self.visualizers[name].update_renderer()

    def _initialize_view_controllers_if_needed(self) -> None:
        """Initialize view controllers for any visualizers that don't have one."""
        for visualizer_name in self.visualizers:
            if visualizer_name not in self.view_controllers:
                self.view_controllers[visualizer_name] = ViewPointController(lookat=(0, 0, 0))

    def _apply_pca_return_projection(
        self,
        tensor_flat: torch.Tensor,
        projection_matrix: Optional[torch.Tensor] = None,
        lower_bound: Optional[torch.Tensor] = None,
        upper_bound: Optional[torch.Tensor] = None,
        num_iterations: int = 5,
        target_dimension: int = 3,
    ) -> Tuple[torch.Tensor, PCAParams]:
        """
        Perform Principal Component Analysis (PCA) on high-dimensional features.

        Args:
            tensor_flat: Input tensor of shape (N, D) containing high-dimensional features
            projection_matrix: Optional pre-computed PCA projection matrix
            lower_bound: Optional pre-computed lower bounds for normalization
            upper_bound: Optional pre-computed upper bounds for normalization
            num_iterations: Number of iterations for PCA computation
            target_dimension: Target dimensionality for projection

        Returns:
            Tuple containing:
                - low_rank: Tensor of shape (N, target_dimension) containing projected features
                - pca_params: Tuple of (projection_matrix, lower_bound, upper_bound) for future use
        """
        # Modified from https://github.com/pfnet-research/distilled-feature-fields/blob/master/
        # train.py

        if projection_matrix is None:
            # Remove empty features when computing the basis
            valid_mask = ~torch.all(tensor_flat == 0, axis=-1)
            tensor_nonzero = tensor_flat[valid_mask]

            mean = tensor_nonzero.mean(0)
            with torch.no_grad():
                _, _, pca_v = torch.pca_lowrank(tensor_nonzero - mean, niter=num_iterations)
            projection_matrix = pca_v[:, :target_dimension]
        low_rank = tensor_flat @ projection_matrix
        if lower_bound is None:
            lower_bound = torch.quantile(low_rank, 0.01, dim=0)
        if upper_bound is None:
            upper_bound = torch.quantile(low_rank, 0.99, dim=0)

        low_rank = (low_rank - lower_bound) / (upper_bound - lower_bound)
        low_rank = torch.clamp(low_rank, 0, 1)
        return low_rank, (projection_matrix, lower_bound, upper_bound)

    def _get_normalized_colors_from_features(self, features: torch.Tensor) -> npt.NDArray:
        # Remove the excess features caused by nvblox having a longer feature length than required.
        # We don't want these extra values to affect the PCA computation.
        assert self.embedding_dim is not None, 'If visualizing deep features, ' \
            'embedding_dim must be provided to the Visualizer constructor.'
        if features.shape[-1] > self.embedding_dim:
            features = features[..., :self.embedding_dim]

        # Check if we have valid features
        if features.shape[0] == 0:
            return torch.zeros(0, 3)

        # Apply PCA with stored parameters if available
        if not self.color_bounds_initialized:
            # First frame - compute and store PCA params
            features_pca, self.pca_params = self._apply_pca_return_projection(
                features.float(),
                num_iterations=10,    # More iterations for better initial projection
                target_dimension=3)
            self.color_bounds_initialized = True
        else:
            # Use stored PCA params for consistent colors
            if self.pca_params is None:
                raise ValueError('PCA params are not initialized')
            else:
                projection_matrix, lower_bound, upper_bound = self.pca_params
                features_pca = self._apply_pca_return_projection(
                    features.float(),
                    projection_matrix=projection_matrix,
                    lower_bound=lower_bound,
                    upper_bound=upper_bound,
                    target_dimension=3)[0]

        features_colors_normalized = cv2.normalize(features_pca.cpu().detach().numpy(),
                                                   None,
                                                   alpha=0,
                                                   beta=1,
                                                   norm_type=cv2.NORM_MINMAX,
                                                   dtype=cv2.CV_64F)
        return features_colors_normalized
