// Minimal ROS 2 wrapper around nvblox::Mapper for 3D frontier exploration.
//
// Inputs:
//   - sensor_msgs/PointCloud2 in body frame (e.g. /cloud_registered_body)
//   - nav_msgs/Odometry        (e.g. /Odometry from Point-LIO / Fast-LIO2)
//
// Outputs:
//   - nav_msgs/OccupancyGrid              traversability_grid    (2.5D, for BFS reachability)
//   - nvblox_frontend_msgs/VoxelGrid3D    voxels_3d              (sparse 3D, for CFPA2 IG sampling)
//
// Layer: nvblox occupancy log-odds. Unknown == voxel not in hash map (block not allocated).
//
// Performance: block-based iteration (getAllBlockIndices + one cudaMemcpy per 8³ block)
// instead of per-voxel getVoxel calls. For a 20×20×3 m window, ~200 allocated blocks
// = ~10 ms per publish vs. ~24 s with per-voxel queries.

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <nvblox/nvblox.h>

#include "nvblox_frontend_msgs/msg/voxel_grid3_d.hpp"

namespace nvbf {

using sensor_msgs::msg::PointCloud2;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::OccupancyGrid;
using nvblox_frontend_msgs::msg::VoxelGrid3D;

// Block memory layout constants (must match nvblox's VoxelBlock<T>)
static constexpr int kVPS = 8;  // kVoxelsPerSide
using OccBlock = nvblox::VoxelBlock<nvblox::OccupancyVoxel>;

class MapperNode : public rclcpp::Node {
public:
  MapperNode() : Node("nvblox_frontend_mapper") {
    voxel_size_m_ = declare_parameter<double>("voxel_size_m", 0.10);
    // Use relative topic names so the node's namespace (e.g. /robot/) is
    // automatically prepended, matching Fast-LIO + slam_odom_relay outputs.
    cloud_topic_  = declare_parameter<std::string>("cloud_topic", "cloud_registered_body");
    odom_topic_   = declare_parameter<std::string>("odom_topic",  "odom/nav");
    world_frame_  = declare_parameter<std::string>("world_frame", "map");

    publish_period_s_ = declare_parameter<double>("publish_period_s", 0.5);

    trav_xy_extent_m_   = declare_parameter<double>("trav_xy_extent_m", 20.0);
    trav_z_min_m_       = declare_parameter<double>("trav_z_min_m",     -0.5);
    trav_z_max_m_       = declare_parameter<double>("trav_z_max_m",      2.5);
    robot_clearance_m_  = declare_parameter<double>("robot_clearance_m", 0.5);
    slope_max_deg_      = declare_parameter<double>("slope_max_deg",     30.0);
    step_max_m_         = declare_parameter<double>("step_max_m",         0.20);
    // Ground filter: strip LiDAR returns at world_z < this threshold before
    // nvblox integration. Removes floor-level litter that creates false surfaces.
    // Default 0.15 m keeps obstacles above shin height while filtering flat floor.
    ground_z_max_m_     = declare_parameter<double>("ground_z_max_m",    0.15);
    // Minimum log-odds to count a voxel as "occupied surface" in traversability.
    // Requires >1 consistent hit (nvblox typically adds ~0.85 per hit), so stray
    // single-scan returns (lo ≈ 0.3–0.85) don't become phantom surfaces.
    occ_lo_thresh_      = declare_parameter<double>("occ_lo_thresh",      0.7);
    // Free-space fallback for columns with no occupied surface voxel. We only
    // infer a traversable ground surface when the column contains a tall,
    // low-starting contiguous run of FREE voxels. This recovers flat floor /
    // ramp support that the ground-point pre-filter intentionally strips from
    // occupancy, without reverting to the old "any free voxel => FREE" dome.
    free_surface_max_start_z_m_ =
        declare_parameter<double>("free_surface_max_start_z_m", 0.25);
    free_surface_min_run_voxels_ =
        declare_parameter<int>("free_surface_min_run_voxels", 10);

    voxel_xy_extent_m_ = declare_parameter<double>("voxel_xy_extent_m", 20.0);
    voxel_z_extent_m_  = declare_parameter<double>("voxel_z_extent_m",   3.0);
    voxel_z_origin_m_  = declare_parameter<double>("voxel_z_origin_m", -0.5);

    int num_azim   = declare_parameter<int>("lidar_num_azimuth",  1024);
    int num_elev   = declare_parameter<int>("lidar_num_elevation", 128);
    double min_rng = declare_parameter<double>("lidar_min_range",  0.3);
    double vert_fov_deg = declare_parameter<double>("lidar_vfov_deg", 59.0);

    mapper_ = std::make_unique<nvblox::Mapper>(
        static_cast<float>(voxel_size_m_),
        nvblox::BlockMemoryPoolParams(),
        nvblox::ProjectiveLayerType::kOccupancy);

    const float vert_fov_rad = static_cast<float>(vert_fov_deg * M_PI / 180.0);
    lidar_ = std::make_unique<nvblox::Lidar>(
        num_azim, num_elev, static_cast<float>(min_rng), vert_fov_rad);

    // Use best_effort QoS to match sensor publisher defaults.
    // RELIABLE → BEST_EFFORT is DDS-compatible (subscriber's requirement is weaker).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    odom_sub_ = create_subscription<Odometry>(
        odom_topic_, qos,
        std::bind(&MapperNode::odom_cb, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<PointCloud2>(
        cloud_topic_, qos,
        std::bind(&MapperNode::cloud_cb, this, std::placeholders::_1));

    // TRANSIENT_LOCAL so nav2's StaticLayer gets the last traversability_grid
    // immediately on subscribe, not on the next 0.5 s publish tick.
    // This eliminates the startup window where the costmap has no map and
    // reports "Robot is out of bounds of the costmap!" on first planning.
    auto trav_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    trav_pub_        = create_publisher<OccupancyGrid>("traversability_grid", trav_qos);
    voxels_pub_      = create_publisher<VoxelGrid3D>("voxels_3d", 1);
    voxel_cloud_pub_ = create_publisher<PointCloud2>("voxels_cloud", 1);

    pub_timer_ = create_wall_timer(
        std::chrono::duration<double>(publish_period_s_),
        std::bind(&MapperNode::publish_outputs, this));

    RCLCPP_INFO(get_logger(),
        "nvblox_frontend_mapper started. voxel=%.3fm cloud=%s odom=%s frame=%s period=%.2fs",
        voxel_size_m_, cloud_topic_.c_str(), odom_topic_.c_str(),
        world_frame_.c_str(), publish_period_s_);
  }

private:
  // ============================================================
  // Callbacks
  // ============================================================
  void odom_cb(const Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  void cloud_cb(const PointCloud2::SharedPtr msg) {
    Odometry odom;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_odom_) {
        if ((++missing_odom_warn_) % 50 == 1)
          RCLCPP_WARN(get_logger(), "cloud arrived but no odom yet on %s", odom_topic_.c_str());
        return;
      }
      odom = latest_odom_;
    }

    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation() = Eigen::Vector3f(
        static_cast<float>(odom.pose.pose.position.x),
        static_cast<float>(odom.pose.pose.position.y),
        static_cast<float>(odom.pose.pose.position.z));
    Eigen::Quaternionf q(
        static_cast<float>(odom.pose.pose.orientation.w),
        static_cast<float>(odom.pose.pose.orientation.x),
        static_cast<float>(odom.pose.pose.orientation.y),
        static_cast<float>(odom.pose.pose.orientation.z));
    T.linear() = q.toRotationMatrix();

    // NOTE: ground-point pre-filter removed (was: skip points with world_z <
    // ground_z_max_m_). Filtering points before integrateDepth also suppresses
    // free-space carving along those rays → air columns above the floor stay
    // UNKNOWN forever → frontier_mask (FREE & dilate(UNKNOWN)) is empty.
    // Noise suppression is now handled at voxel query time:
    //   (a) occ_lo_thresh_=0.7 requires ≥2 consistent hits to count as surface
    //   (b) traversability builder ignores voxels below trav_z_min_m_
    //   (c) 3×3 median filter removes isolated OCC noise cells
    std::vector<nvblox::Vector3f> pts;
    pts.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg,"x"), iy(*msg,"y"), iz(*msg,"z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
      pts.emplace_back(*ix, *iy, *iz);
    }
    if (pts.empty()) {
      if ((++empty_cloud_warn_) % 50 == 1)
        RCLCPP_WARN(get_logger(), "empty / all-NaN cloud on %s", cloud_topic_.c_str());
      return;
    }

    nvblox::Pointcloud pc(static_cast<int>(pts.size()), nvblox::MemoryType::kDevice);
    pc.copyPointsFromAsync(pts, nvblox::CudaStreamOwning());

    mapper_->integrateDepth(pc, T, *lidar_, /*motion_comp=*/false);
    mapper_->updateEsdf(nvblox::UpdateFullLayer::kNo);

    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      latest_robot_xyz_ = T.translation();
      have_map_data_ = true;
    }
    if ((++cloud_count_) % 100 == 1)
      RCLCPP_INFO(get_logger(), "integrated cloud #%u (%zu pts) @ (%.1f,%.1f,%.1f)",
          cloud_count_, pts.size(),
          T.translation().x(), T.translation().y(), T.translation().z());
  }

  // ============================================================
  // Periodic publisher
  // ============================================================
  void publish_outputs() {
    Eigen::Vector3f robot_xyz;
    rclcpp::Time stamp;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_map_data_) return;
      robot_xyz = latest_robot_xyz_;
      stamp = now();
    }
    publish_traversability(robot_xyz, stamp);
    publish_voxels_3d(robot_xyz, stamp);

    if ((++publish_count_) % 20 == 1)
      RCLCPP_INFO(get_logger(), "published traversability+voxels_3d #%u", publish_count_);
  }

  // ============================================================
  // Block-based traversability publisher (2.5D)
  // ============================================================
  void publish_traversability(const Eigen::Vector3f& robot_xyz, const rclcpp::Time& stamp) {
    const float vs = static_cast<float>(voxel_size_m_);
    const float bs = vs * kVPS;  // block size in metres (8 voxels × 0.1 m = 0.8 m)
    const int   nxy = static_cast<int>(std::lround(trav_xy_extent_m_ / vs));
    const float half = nxy * vs * 0.5f;
    const float ox = robot_xyz.x() - half;
    const float oy = robot_xyz.y() - half;

    OccupancyGrid g;
    g.header.stamp = stamp;
    g.header.frame_id = world_frame_;
    g.info.resolution = vs;
    g.info.width  = nxy;
    g.info.height = nxy;
    g.info.origin.position.x = ox;
    g.info.origin.position.y = oy;
    g.info.origin.orientation.w = 1.0;

    const size_t ncells = static_cast<size_t>(nxy) * nxy;
    std::vector<float>   H(ncells, std::numeric_limits<float>::quiet_NaN());
    std::vector<int8_t>  cls(ncells, -1);
    std::vector<uint64_t> free_bits(ncells, 0ULL);

    // --- Pass 1: find HIGHEST occupied z per column from allocated blocks ---
    // We want the top surface, not the underside. Thin inclined slabs (like
    // the ramp, 0.1 m thick) show both faces to the LiDAR; picking the highest
    // occupied voxel gives the walkable top surface. Walls (1 m step) are still
    // rejected downstream by the step/slope filter.
    const auto& occ_layer = mapper_->occupancy_layer();
    const auto  block_indices = occ_layer.getAllBlockIndices();

    OccBlock block_cpu;  // 2 KB buffer reused per block

    const float z_min = static_cast<float>(trav_z_min_m_);
    const float z_max = static_cast<float>(trav_z_max_m_);
    const float clearance = static_cast<float>(robot_clearance_m_);

    for (const auto& bidx : block_indices) {
      // Block world-space corner
      const float bx = bidx.x() * bs;
      const float by = bidx.y() * bs;
      const float bz = bidx.z() * bs;

      // Skip blocks entirely outside the XY window (plus one block margin)
      if (bx + bs < ox - vs || bx > ox + nxy * vs + vs) continue;
      if (by + bs < oy - vs || by > oy + nxy * vs + vs) continue;
      // Skip blocks entirely outside z range (include clearance headroom)
      if (bz + bs < z_min || bz > z_max + clearance) continue;

      auto block_ptr = occ_layer.getBlockAtIndex(bidx);
      if (!block_ptr) continue;
      cudaMemcpy(&block_cpu, block_ptr.get(), sizeof(OccBlock), cudaMemcpyDeviceToHost);

      for (int vx = 0; vx < kVPS; ++vx) {
        const float wx = bx + (vx + 0.5f) * vs;
        const int ci = static_cast<int>((wx - ox) / vs);
        if (ci < 0 || ci >= nxy) continue;

        for (int vy = 0; vy < kVPS; ++vy) {
          const float wy = by + (vy + 0.5f) * vs;
          const int cj = static_cast<int>((wy - oy) / vs);
          if (cj < 0 || cj >= nxy) continue;

          for (int vz = 0; vz < kVPS; ++vz) {
            const float wz = bz + (vz + 0.5f) * vs;
            if (wz < z_min || wz > z_max) continue;

            const float lo = block_cpu.voxels[vx][vy][vz].log_odds;
            const size_t idx = static_cast<size_t>(cj) * nxy + ci;

            const float occ_lo_thresh = static_cast<float>(occ_lo_thresh_);
            if (lo > occ_lo_thresh) {
              // Track HIGHEST occupied voxel as the surface candidate.
              // Require lo > occ_lo_thresh (≥2 consistent hits) so stray
              // single-scan returns don't create phantom surfaces.
              if (std::isnan(H[idx]) || wz > H[idx])
                H[idx] = wz;
            } else if (lo < 0.0f) {
              // Cache FREE occupancy as a vertical bitmask per (x,y) column.
              // Later, if a column has no occupied surface at all, a grounded
              // contiguous FREE run lets us infer a floor/ramp support plane.
              const int kz = static_cast<int>(
                  std::lround((wz - z_min) / vs - 0.5f));
              if (kz >= 0 && kz < 64) {
                free_bits[idx] |= (uint64_t{1} << kz);
              }
            }
          }
        }
      }
    }

    // --- Pass 2: clearance check --- same block iteration, check z in [H, H+clearance]
    for (const auto& bidx : block_indices) {
      const float bx = bidx.x() * bs;
      const float by = bidx.y() * bs;
      const float bz = bidx.z() * bs;

      if (bx + bs < ox - vs || bx > ox + nxy * vs + vs) continue;
      if (by + bs < oy - vs || by > oy + nxy * vs + vs) continue;
      if (bz + bs < z_min || bz > z_max + clearance) continue;

      auto block_ptr = occ_layer.getBlockAtIndex(bidx);
      if (!block_ptr) continue;
      cudaMemcpy(&block_cpu, block_ptr.get(), sizeof(OccBlock), cudaMemcpyDeviceToHost);

      for (int vx = 0; vx < kVPS; ++vx) {
        const float wx = bx + (vx + 0.5f) * vs;
        const int ci = static_cast<int>((wx - ox) / vs);
        if (ci < 0 || ci >= nxy) continue;

        for (int vy = 0; vy < kVPS; ++vy) {
          const float wy = by + (vy + 0.5f) * vs;
          const int cj = static_cast<int>((wy - oy) / vs);
          if (cj < 0 || cj >= nxy) continue;

          const size_t idx = static_cast<size_t>(cj) * nxy + ci;
          const float h0 = H[idx];
          if (std::isnan(h0)) continue;  // no surface in this column

          for (int vz = 0; vz < kVPS; ++vz) {
            const float wz = bz + (vz + 0.5f) * vs;
            // Clearance zone: above surface, below surface+clearance
            if (wz <= h0 || wz > h0 + clearance) continue;

            if (block_cpu.voxels[vx][vy][vz].log_odds > static_cast<float>(occ_lo_thresh_))
              cls[idx] = 100;  // blocked — don't break, keep checking siblings
          }
        }
      }
    }

    // --- Classify cells ---
    const float free_surface_max_start_z =
        static_cast<float>(free_surface_max_start_z_m_);
    const int free_surface_min_run_voxels =
        std::max(1, free_surface_min_run_voxels_);
    for (int j = 0; j < nxy; ++j) {
      for (int i = 0; i < nxy; ++i) {
        const size_t idx = static_cast<size_t>(j) * nxy + i;
        if (std::isnan(H[idx])) {
          // No occupied surface voxel found in this (x,y) column. Fall back to
          // a grounded FREE-run test: if the column contains a long contiguous
          // stack of FREE voxels that starts low enough, infer that the support
          // surface lives just below the first FREE voxel.
          const uint64_t bits = free_bits[idx];
          if (bits != 0ULL) {
            const int first_k = __builtin_ctzll(bits);
            uint64_t shifted = bits >> first_k;
            int run_voxels = 0;
            while ((shifted & 1ULL) != 0ULL) {
              ++run_voxels;
              shifted >>= 1;
            }
            const float first_free_z = z_min + (static_cast<float>(first_k) + 0.5f) * vs;
            if (first_free_z <= free_surface_max_start_z &&
                run_voxels >= free_surface_min_run_voxels) {
              H[idx] = std::max(z_min, first_free_z - vs);
              cls[idx] = 0;
            }
          }
          continue;
        }
        if (cls[idx] != 100) cls[idx] = 0; // has surface, not blocked → traversable
      }
    }

    // --- Slope + step filter ---
    // Step check uses adjacent cells (true vertical discontinuities = curbs/stairs).
    // Slope check uses a wider baseline (slope_window_cells * vs) so the 0.10 m
    // z-discretization of a smooth ramp doesn't read as a 45° cliff at each
    // voxel boundary. With 3 cells × 0.10 m = 0.30 m baseline, a 14° ramp's
    // 0.10 m discrete rise gives slope 0.10/0.30 = 0.33 < tan(30°)=0.577 → free.
    const float step_max  = static_cast<float>(step_max_m_);
    const float tan_smax  = std::tan(static_cast<float>(slope_max_deg_ * M_PI / 180.0));
    const int   sw        = 3;  // slope baseline in cells (0.30 m at vs=0.10)
    for (int j = 0; j < nxy; ++j) {
      for (int i = 0; i < nxy; ++i) {
        const size_t idx = static_cast<size_t>(j) * nxy + i;
        if (cls[idx] != 0) continue;
        const float h0 = H[idx];
        if (std::isnan(h0)) continue;
        bool blocked = false;
        // Step check: |dh| > step_max between adjacent cells = curb/stair.
        const int di1[] = {-1, 1, 0, 0};
        const int dj1[] = { 0, 0,-1, 1};
        for (int n = 0; n < 4 && !blocked; ++n) {
          const int ii = i + di1[n], jj = j + dj1[n];
          if (ii < 0 || ii >= nxy || jj < 0 || jj >= nxy) continue;
          const float hn = H[static_cast<size_t>(jj) * nxy + ii];
          if (std::isnan(hn)) continue;
          if (std::abs(hn - h0) > step_max) blocked = true;
        }
        if (blocked) { cls[idx] = 100; continue; }
        // Slope check: dh / (sw*vs) > tan(slope_max) over a 3-cell baseline.
        const int diS[] = {-sw, sw, 0, 0};
        const int djS[] = { 0, 0,-sw, sw};
        const float baseline = sw * vs;
        for (int n = 0; n < 4 && !blocked; ++n) {
          const int ii = i + diS[n], jj = j + djS[n];
          if (ii < 0 || ii >= nxy || jj < 0 || jj >= nxy) continue;
          const float hn = H[static_cast<size_t>(jj) * nxy + ii];
          if (std::isnan(hn)) continue;
          if (std::abs(hn - h0) / baseline > tan_smax) blocked = true;
        }
        if (blocked) cls[idx] = 100;
      }
    }

    // --- Force robot footprint cells to FREE ---
    // Prevents self-occupancy: after ground-filter and threshold tuning some
    // residual hits from the robot body could put the start cell in lethal
    // space. Clearing a 1-cell (0.10 m) radius around the robot guarantees
    // SmacPlannerLattice always finds a valid start.
    {
      const int ci_r = static_cast<int>((robot_xyz.x() - ox) / vs);
      const int cj_r = static_cast<int>((robot_xyz.y() - oy) / vs);
      const int fp = 1;  // 1 cell = voxel_size_m (0.10 m)
      for (int dj = -fp; dj <= fp; ++dj)
        for (int di = -fp; di <= fp; ++di) {
          int ci = ci_r + di, cj = cj_r + dj;
          if (ci >= 0 && ci < nxy && cj >= 0 && cj < nxy)
            cls[static_cast<size_t>(cj) * nxy + ci] = 0;
        }
    }

    // --- 3×3 median filter: remove salt-and-pepper noise ---
    // Isolated OCCUPIED (100) cells surrounded by FREE (0) are noise from
    // stray LiDAR returns; a median filter removes them while preserving
    // real obstacle clusters. int8_t sort order: -1 < 0 < 100.
    {
      std::vector<int8_t> filtered(cls);
      for (int j = 1; j < nxy - 1; ++j) {
        for (int i = 1; i < nxy - 1; ++i) {
          int8_t nb[9];
          int n = 0;
          for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di)
              nb[n++] = cls[static_cast<size_t>(j + dj) * nxy + (i + di)];
          std::sort(nb, nb + 9);
          filtered[static_cast<size_t>(j) * nxy + i] = nb[4];
        }
      }
      cls = std::move(filtered);
    }

    g.data.assign(cls.begin(), cls.end());
    trav_pub_->publish(g);
  }

  // ============================================================
  // Block-based 3D sparse occupancy publisher
  // ============================================================
  void publish_voxels_3d(const Eigen::Vector3f& robot_xyz, const rclcpp::Time& stamp) {
    const float vs  = static_cast<float>(voxel_size_m_);
    const float bs  = vs * kVPS;
    const int   nxy = static_cast<int>(std::lround(voxel_xy_extent_m_ / vs));
    const int   nz  = static_cast<int>(std::lround(voxel_z_extent_m_  / vs));
    const float half_xy = nxy * vs * 0.5f;
    const float oz  = static_cast<float>(voxel_z_origin_m_);

    VoxelGrid3D msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = world_frame_;
    msg.voxel_size = vs;
    msg.origin.x = robot_xyz.x() - half_xy;
    msg.origin.y = robot_xyz.y() - half_xy;
    msg.origin.z = oz;
    msg.size_x = static_cast<uint32_t>(nxy);
    msg.size_y = static_cast<uint32_t>(nxy);
    msg.size_z = static_cast<uint32_t>(nz);
    msg.data.assign(static_cast<size_t>(nxy) * nxy * nz, -1);  // -1 = unknown

    const float gox = msg.origin.x;
    const float goy = msg.origin.y;

    const auto& occ_layer   = mapper_->occupancy_layer();
    const auto  block_indices = occ_layer.getAllBlockIndices();

    OccBlock block_cpu;

    for (const auto& bidx : block_indices) {
      const float bx = bidx.x() * bs;
      const float by = bidx.y() * bs;
      const float bz = bidx.z() * bs;

      if (bx + bs < gox || bx > gox + nxy * vs) continue;
      if (by + bs < goy || by > goy + nxy * vs) continue;
      if (bz + bs < oz  || bz > oz  + nz  * vs) continue;

      auto block_ptr = occ_layer.getBlockAtIndex(bidx);
      if (!block_ptr) continue;
      cudaMemcpy(&block_cpu, block_ptr.get(), sizeof(OccBlock), cudaMemcpyDeviceToHost);

      for (int vx = 0; vx < kVPS; ++vx) {
        const float wx = bx + (vx + 0.5f) * vs;
        const int xi = static_cast<int>((wx - gox) / vs);
        if (xi < 0 || xi >= nxy) continue;

        for (int vy = 0; vy < kVPS; ++vy) {
          const float wy = by + (vy + 0.5f) * vs;
          const int yi = static_cast<int>((wy - goy) / vs);
          if (yi < 0 || yi >= nxy) continue;

          for (int vz = 0; vz < kVPS; ++vz) {
            const float wz = bz + (vz + 0.5f) * vs;
            const int zi = static_cast<int>((wz - oz) / vs);
            if (zi < 0 || zi >= nz) continue;

            const float lo = block_cpu.voxels[vx][vy][vz].log_odds;
            if (lo == 0.0f) continue;
            const size_t off =
                (static_cast<size_t>(zi) * nxy + yi) * nxy + xi;
            msg.data[off] = (lo > 0.0f) ? static_cast<int8_t>(100)
                                        : static_cast<int8_t>(0);
            if (lo > 0.0f) occ_pts_.emplace_back(wx, wy, wz);
          }
        }
      }
    }
    voxels_pub_->publish(msg);

    // PointCloud2 of occupied voxels for RViz 3D visualization (z-colored)
    PointCloud2 cloud;
    cloud.header = msg.header;
    cloud.height = 1;
    cloud.width  = static_cast<uint32_t>(occ_pts_.size());
    cloud.is_bigendian = false;
    cloud.is_dense     = true;
    cloud.point_step   = 12;
    cloud.row_step     = cloud.width * 12;
    cloud.fields.resize(3);
    auto mk = [](const char* n, uint32_t off) {
      sensor_msgs::msg::PointField f;
      f.name = n; f.offset = off;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
      return f;
    };
    cloud.fields[0] = mk("x", 0);
    cloud.fields[1] = mk("y", 4);
    cloud.fields[2] = mk("z", 8);
    cloud.data.resize(occ_pts_.size() * 12);
    float* p = reinterpret_cast<float*>(cloud.data.data());
    for (const auto& pt : occ_pts_) { *p++ = pt.x(); *p++ = pt.y(); *p++ = pt.z(); }
    voxel_cloud_pub_->publish(cloud);
    occ_pts_.clear();
  }

  // ============================================================
  // Members
  // ============================================================
  double voxel_size_m_;
  std::string cloud_topic_, odom_topic_, world_frame_;
  double publish_period_s_;
  double trav_xy_extent_m_, trav_z_min_m_, trav_z_max_m_;
  double robot_clearance_m_, slope_max_deg_, step_max_m_;
  double voxel_xy_extent_m_, voxel_z_extent_m_, voxel_z_origin_m_;
  double ground_z_max_m_, occ_lo_thresh_;
  double free_surface_max_start_z_m_;
  int free_surface_min_run_voxels_;

  std::unique_ptr<nvblox::Mapper> mapper_;
  std::unique_ptr<nvblox::Lidar>  lidar_;

  std::mutex state_mtx_;
  Odometry latest_odom_;
  bool have_odom_{false};
  Eigen::Vector3f latest_robot_xyz_{0.f, 0.f, 0.f};
  bool have_map_data_{false};

  std::vector<Eigen::Vector3f> occ_pts_;  // scratch buffer for PointCloud2 build

  int missing_odom_warn_{0};
  int empty_cloud_warn_{0};
  unsigned int cloud_count_{0};
  unsigned int publish_count_{0};

  rclcpp::Subscription<Odometry>::SharedPtr    odom_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr  trav_pub_;
  rclcpp::Publisher<VoxelGrid3D>::SharedPtr    voxels_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr    voxel_cloud_pub_;
  rclcpp::TimerBase::SharedPtr                 pub_timer_;
};

}  // namespace nvbf

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nvbf::MapperNode>());
  rclcpp::shutdown();
  return 0;
}
