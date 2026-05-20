/*
Copyright 2026 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <gtest/gtest.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "nvblox/core/indexing.h"
#include "nvblox/core/types.h"
#include "nvblox/integrators/esdf_integrator.h"
#include "nvblox/io/ply_writer.h"
#include "nvblox/io/pointcloud_io.h"
#include "nvblox/map/accessors.h"
#include "nvblox/map/common_names.h"
#include "nvblox/map/layer.h"
#include "nvblox/map/voxels.h"
#include "nvblox/primitives/scene.h"
#include "nvblox/tests/utils.h"

using namespace nvblox;

constexpr float kFloatEpsilon = 1e-4;

// Calls callback(neighbor_voxel) for each of the 26 neighbors of the
// voxel at (block_idx, voxel_idx), skipping neighbors in unallocated blocks.
template <typename VoxelType, typename Callback>
void forEachNeighborVoxel(const BlockLayer<VoxelBlock<VoxelType>>& layer,
                          const Index3D& block_idx, const Index3D& voxel_idx,
                          Callback&& callback) {
  constexpr int kVoxelsPerSide = VoxelBlock<VoxelType>::kVoxelsPerSide;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;

        Index3D neighbor_block_idx = block_idx;
        Index3D neighbor_voxel_idx = voxel_idx + Index3D(dx, dy, dz);

        for (int axis = 0; axis < 3; ++axis) {
          if (neighbor_voxel_idx(axis) < 0) {
            neighbor_block_idx(axis)--;
            neighbor_voxel_idx(axis) += kVoxelsPerSide;
          } else if (neighbor_voxel_idx(axis) >= kVoxelsPerSide) {
            neighbor_block_idx(axis)++;
            neighbor_voxel_idx(axis) -= kVoxelsPerSide;
          }
        }

        auto block_ptr = layer.getBlockAtIndex(neighbor_block_idx);
        if (!block_ptr) continue;

        callback(
            block_ptr->voxels[neighbor_voxel_idx.x()][neighbor_voxel_idx.y()]
                             [neighbor_voxel_idx.z()]);
      }
    }
  }
}

class UnobservedEsdfPolicyTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void addSphereToScene();

  // Helper for UnobservedEsdfPolicy tests
  template <typename UnobservedVoxelChecker>
  void testUnobservedEsdfPolicy(
      UnobservedEsdfPolicy policy, bool add_negative_truncation_band_sites,
      const TsdfLayer& tsdf_observed_half,
      const std::vector<Index3D>& all_blocks_in_aabb,
      float tsdf_truncation_distance_vox,
      UnobservedVoxelChecker&& unobserved_voxel_checker);

  float voxel_size_ = 0.10f;
  float max_distance_ = 4.0f;

  TsdfLayer::Ptr tsdf_layer_;
  EsdfLayer::Ptr esdf_layer_;

  EsdfIntegrator esdf_integrator_;
  primitives::Scene scene_;
};

void UnobservedEsdfPolicyTest::SetUp() {
  std::srand(0);

  tsdf_layer_.reset(new TsdfLayer(voxel_size_, MemoryType::kUnified));
  esdf_layer_.reset(new EsdfLayer(voxel_size_, MemoryType::kUnified));

  esdf_integrator_.max_esdf_distance_m(max_distance_);
  esdf_integrator_.min_weight(1.0f);
}

void UnobservedEsdfPolicyTest::addSphereToScene() {
  scene_.aabb() = AxisAlignedBoundingBox(Vector3f(-3.0f, -3.0f, 0.0f),
                                         Vector3f(3.0f, 3.0f, 3.0f));
  scene_.addPrimitive(
      std::make_unique<primitives::Sphere>(Vector3f(0.0, 0.0, 2.0), 1.0f));
}

template <typename UnobservedVoxelChecker>
void UnobservedEsdfPolicyTest::testUnobservedEsdfPolicy(
    UnobservedEsdfPolicy policy, bool add_negative_truncation_band_sites,
    const TsdfLayer& tsdf_layer, const std::vector<Index3D>& all_blocks_in_aabb,
    float tsdf_truncation_distance_vox,
    UnobservedVoxelChecker&& unobserved_voxel_checker) {
  // Set the unobserved ESDF policy.
  esdf_integrator_.unobserved_esdf_policy(policy);
  esdf_integrator_.add_negative_truncation_band_sites(
      add_negative_truncation_band_sites);
  esdf_integrator_.truncation_distance_vox(tsdf_truncation_distance_vox);

  // Clear the ESDF layer and integrate the TSDF layer.
  // We update all blocks of the AABB, although the TSDF layer
  // is not available in the full AABB.
  esdf_layer_->clear();
  EXPECT_GT(all_blocks_in_aabb.size(), tsdf_layer.getAllBlockIndices().size());
  esdf_integrator_.integrateBlocks(tsdf_layer, all_blocks_in_aabb,
                                   esdf_layer_.get());

  // Check all blocks of the AABB are allocated in ESDF
  for (const Index3D& block_idx : all_blocks_in_aabb) {
    EXPECT_TRUE(esdf_layer_->getBlockAtIndex(block_idx));
  }

  // Check behavior across the entire layer
  const float min_weight = esdf_integrator_.min_weight();
  const float voxel_size = tsdf_layer.voxel_size();
  int observed_tsdf_count = 0;
  int low_weight_tsdf_count = 0;
  int non_allocated_tsdf_count = 0;

  callFunctionOnAllVoxels<EsdfVoxel>(*esdf_layer_, [&, min_weight](
                                                       const Index3D& block_idx,
                                                       const Index3D& voxel_idx,
                                                       const EsdfVoxel* voxel) {
    // Get the TSDF block for the current block index.
    TsdfBlock::ConstPtr tsdf_block = tsdf_layer.getBlockAtIndex(block_idx);

    // Check if the block is observed in the TSDF layer.
    if (tsdf_block) {
      const TsdfVoxel& tsdf_voxel =
          tsdf_block->voxels[voxel_idx.x()][voxel_idx.y()][voxel_idx.z()];

      // Check if the TSDF voxel weight is high enough to be
      // considered observed in ESDF integration.
      if (tsdf_voxel.weight >= min_weight) {
        // Valid TSDF voxel: ESDF should be observed
        EXPECT_TRUE(voxel->observed);

        // Inside truncation band: ESDF should be ≤ |TSDF| + tolerance
        // (ESDF is the closest distance to the surface, while TSDF is view
        // dependent)
        const float tsdf_dist_vox = std::abs(tsdf_voxel.distance) / voxel_size;
        if (tsdf_dist_vox < tsdf_truncation_distance_vox) {
          const float esdf_dist_vox = std::sqrt(voxel->squared_distance_vox);
          constexpr float kDiscretizationTolerance =
              1.0f;  // 1 voxel for quantization
          EXPECT_LE(esdf_dist_vox, tsdf_dist_vox + kDiscretizationTolerance);
          observed_tsdf_count++;
        }
      } else {
        // Low weight TSDF voxel:
        // Voxel is unobserved and esdf depends on policy
        unobserved_voxel_checker(block_idx, voxel_idx, voxel,
                                 esdf_layer_.get());
        low_weight_tsdf_count++;
      }
    } else {
      // Non-allocated TSDF block:
      // Voxel is unobserved and esdf depends on policy
      ASSERT_FALSE(tsdf_block);
      unobserved_voxel_checker(block_idx, voxel_idx, voxel, esdf_layer_.get());
      non_allocated_tsdf_count++;
    }
  });

  EXPECT_GT(observed_tsdf_count, 0);
  EXPECT_GT(low_weight_tsdf_count, 0);
  EXPECT_GT(non_allocated_tsdf_count, 0);

  if (FLAGS_nvblox_test_file_output) {
    const std::string policy_name = toString(policy);
    const std::string trunc_suffix =
        add_negative_truncation_band_sites ? "_neg_trunc_sites" : "";
    io::outputVoxelLayerToPly(
        *esdf_layer_, "esdf_policy_" + policy_name + trunc_suffix + ".ply");
  }
}

// Test that UnobservedEsdfPolicy produces the expected ESDF output for
// unobserved blocks (blocks where the TSDF has no data), and that distance
// propagation behaves correctly: kFree propagates into unobserved from
// neighbors; kOccupied propagates from unobserved to neighbors; kIgnore
// does neither.
TEST_F(UnobservedEsdfPolicyTest, AllModes) {
  // Generating a scene with a sphere.
  addSphereToScene();
  constexpr float kTsdfTruncationDistanceVox = 4.0f;
  scene_.generateLayerFromScene(kTsdfTruncationDistanceVox * voxel_size_,
                                tsdf_layer_.get());

  // Full set of block indices for ESDF integration.
  std::vector<Index3D> all_block_indices = tsdf_layer_->getAllBlockIndices();

  // Remove the right half (x >= 0) from the TSDF layer to test the policies on
  // unobserved voxels (missing in tsdf).
  std::vector<Index3D> blocks_to_remove;
  for (const Index3D& idx : all_block_indices) {
    if (idx.x() >= 0) {
      blocks_to_remove.push_back(idx);
    }
  }
  tsdf_layer_->clearBlocks(blocks_to_remove);
  ASSERT_GT(tsdf_layer_->getAllBlockIndices().size(), 0);
  ASSERT_EQ(tsdf_layer_->getAllBlockIndices().size(),
            all_block_indices.size() / 2);

  if (FLAGS_nvblox_test_file_output) {
    io::outputVoxelLayerToPly(*tsdf_layer_, "esdf_policy_tsdf_input.ply");
  }

  // ---- kIgnore: unobserved voxels stay unobserved
  testUnobservedEsdfPolicy(
      UnobservedEsdfPolicy::kIgnore, false, *tsdf_layer_, all_block_indices,
      kTsdfTruncationDistanceVox,
      [](const Index3D&, const Index3D&, const EsdfVoxel* voxel,
         const EsdfLayer*) {
        // Unobserved voxels stay unobserved in the esdf.
        EXPECT_FALSE(voxel->observed);
        EXPECT_FALSE(voxel->is_inside);
        EXPECT_FALSE(voxel->is_site);
        EXPECT_TRUE(voxel->parent_direction == Index3D::Zero());
      });

  // ---- kFree: unobserved voxels are set to free; distances propagate in
  // Test with both values of add_negative_truncation_band_sites.
  for (const bool add_negative_truncation_band_sites : {false, true}) {
    uint32_t neighbor_is_site = 0, neighbors_are_not_sites = 0;
    testUnobservedEsdfPolicy(
        UnobservedEsdfPolicy::kFree, add_negative_truncation_band_sites,
        *tsdf_layer_, all_block_indices, kTsdfTruncationDistanceVox,
        [&](const Index3D& block_idx, const Index3D& voxel_idx,
            const EsdfVoxel* voxel, const EsdfLayer* layer) {
          // Unobserved voxels are set to free in the esdf.
          EXPECT_TRUE(voxel->observed);
          EXPECT_FALSE(voxel->is_inside);
          if (!add_negative_truncation_band_sites) {
            EXPECT_FALSE(voxel->is_site);
          }

          // Verify that distances propagate into the (previously unobserved)
          // free region. Check if the esdf is smaller than the max distance
          // to one of the voxels neighbors (sqrt(3)).
          if (voxel->squared_distance_vox < 3.0 + kFloatEpsilon) {
            // The distance is propagated into the free region.
            // According to the esdf, one of the neighbors must be a site.
            bool has_site_neighbor = false;
            forEachNeighborVoxel<EsdfVoxel>(
                *layer, block_idx, voxel_idx,
                [&has_site_neighbor](const EsdfVoxel& neighbor) {
                  if (neighbor.is_site) has_site_neighbor = true;
                });
            EXPECT_TRUE(has_site_neighbor);
            neighbor_is_site++;
          } else {
            // According to the esdf, none of the neighbors must be a site.
            forEachNeighborVoxel<EsdfVoxel>(
                *layer, block_idx, voxel_idx, [&](const EsdfVoxel& neighbor) {
                  EXPECT_FALSE(neighbor.is_site);
                  if (add_negative_truncation_band_sites) {
                    // With boundary sites, a free voxel this far from any
                    // site cannot have an inside neighbor.
                    // If the flag is set, there should be no inside/free
                    // interface
                    EXPECT_FALSE(neighbor.is_inside);
                  }
                });
            neighbors_are_not_sites++;
          }
        });
    EXPECT_GT(neighbor_is_site, 0);
    EXPECT_GT(neighbors_are_not_sites, 0);
  }

  // ---- kOccupied: unobserved becomes sites
  testUnobservedEsdfPolicy(
      UnobservedEsdfPolicy::kOccupied, false, *tsdf_layer_, all_block_indices,
      kTsdfTruncationDistanceVox,
      [this](const Index3D& block_idx, const Index3D& voxel_idx,
             const EsdfVoxel* voxel, const EsdfLayer*) {
        // Unobserved voxels are set to occupied in the esdf.
        EXPECT_TRUE(voxel->observed);
        EXPECT_TRUE(voxel->is_site);
        EXPECT_TRUE(voxel->is_inside);
        EXPECT_NEAR(voxel->squared_distance_vox, 0.0f, kFloatEpsilon);
        EXPECT_TRUE(voxel->parent_direction == Index3D::Zero());

        // Next we verify that distances propagate from the occupied voxel to
        // its neighbors. Check that the esdf of all observed neighbors is
        // smaller than the max distance to the site under test (sqrt(3)).
        forEachNeighborVoxel<EsdfVoxel>(
            *esdf_layer_, block_idx, voxel_idx, [](const EsdfVoxel& neighbor) {
              if (!neighbor.observed) return;
              EXPECT_LE(neighbor.squared_distance_vox, 3.0f);
            });
      });
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  return RUN_ALL_TESTS();
}
