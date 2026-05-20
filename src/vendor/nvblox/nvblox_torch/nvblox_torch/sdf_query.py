#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
from typing import Tuple, Any, Optional

import torch


class EsdfQuery(torch.autograd.Function):
    """Queries the ESDF at a set of locations.

    Supports auto differentiation of the distance field with respect to the
    query points' x,y,z locations.

    """

    @staticmethod
    def forward(
        ctx: Any,
        query_spheres: torch.Tensor,
        c_mapper_instance: Any,
        out_tensor: Optional[torch.Tensor],
        mapper_id: int = -1,
    ) -> torch.Tensor:
        """Query the ESDF at a set of locations.

        The query looks up the closest ESDF voxel and returns its signed distance,
        minus the sphere radius.

        NOTE:
        - If out_tensor is not provided, it is temporarily allocated. For fastest
            performance, provide an appropriately sized out_tensor.

        Args:
            query_spheres: Nx4 tensor [x,y,z,radius] or Nx3 tensor [x,y,z] for each query point.
                           If radii are not given, they will be assumed to be zero
            c_mapper_instance: The C++ Mapper instance to query.
            out_tensor: Optional Nx4 tensor [gx,gy,gz,distance] which stores the query results.
            mapper_id: The ID of the mapper to query. If -1, all mappers are queried and the
                minimum distance is returned.

        Returns
            A N dimensional tensor containing the distances from the spheres to the surface defined
            by the ESDF.
        """
        # Add zero radii to the query tensor if not given.  TODO(dtingdahl) Avoid re-allocation here
        # by making the underlying kernel flexible to receive both Nx3 and Nx4 input.
        if query_spheres.shape[1] == 3:
            num_queries = query_spheres.shape[0]
            zeros = torch.zeros(num_queries,
                                1,
                                device=query_spheres.device,
                                dtype=query_spheres.dtype)
            query_spheres = torch.cat([query_spheres, zeros], dim=1)

        assert query_spheres.ndim == 2 and query_spheres.shape[1] == 4
        if out_tensor is None:
            out_tensor = torch.zeros(num_queries, 4)
        assert out_tensor.shape == torch.Size([query_spheres.shape[0], 4])
        if mapper_id >= 0 or c_mapper_instance.num_mappers() == 1:
            mapper_id = 0 if mapper_id == -1 else mapper_id
            query_xyzd = c_mapper_instance.query_esdf(
                out_tensor,
                query_spheres,
                mapper_id,
            )
        else:
            query_xyzd = c_mapper_instance.query_multi_esdf(
                out_tensor,
                query_spheres,
            )
        if len(query_xyzd) == 0:
            raise ValueError('Query failed.')
        # Extract distances for return
        distance = query_xyzd[:, 3]
        # Save the distances and gradient directions for backward pass.
        ctx.save_for_backward(query_xyzd)
        assert distance.ndim == 1
        assert distance.shape[0] == query_spheres.shape[0]
        return distance

    @staticmethod
    def backward(ctx: Any,
                 grad_output: torch.Tensor) -> Tuple[torch.Tensor, None, None, None, None]:
        """Backward pass for the ESDF query.

        Args:
            ctx: Context object.
            grad_output: Output from cost function.

        Returns:
            Gradient of the query points with respect to the cost function output.
        """
        grad_sph = None
        if ctx.needs_input_grad[0]:
            # Extracting the saved gradient directions and distances from the forward pass.
            (query_xyzd, ) = ctx.saved_tensors
            # NOTE(alexmillane): In the original cuRobo implementation, the gradient
            # with respect to the sphere radius was set to 0.0. In my opinion, it should
            # be -1.0.
            query_xyzd[:, 3] = -1.0
            grad_sph = grad_output.unsqueeze(-1) * query_xyzd
        # We only provide gradients with respect to the query points.
        return grad_sph, None, None, None, None
