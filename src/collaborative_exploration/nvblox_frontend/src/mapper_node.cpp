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

    trav_xy_extent_m_   = declare_parameter<double>("trav_xy_extent_m", 40.0);
    // trav_grid origin: NaN ⇒ lock from first odom (robot center). Otherwise
    // use this world-fixed origin every frame (caller knows scene extent).
    trav_world_origin_x_ = declare_parameter<double>("trav_world_origin_x",
        std::numeric_limits<double>::quiet_NaN());
    trav_world_origin_y_ = declare_parameter<double>("trav_world_origin_y",
        std::numeric_limits<double>::quiet_NaN());
    trav_z_min_m_       = declare_parameter<double>("trav_z_min_m",     -0.5);
    trav_z_max_m_       = declare_parameter<double>("trav_z_max_m",      2.5);
    robot_clearance_m_  = declare_parameter<double>("robot_clearance_m", 0.5);
    slope_max_deg_      = declare_parameter<double>("slope_max_deg",     30.0);
    step_max_m_         = declare_parameter<double>("step_max_m",         0.20);
    // 5x5 plane-fit residual threshold (RMS metres) above which the local
    // surface is NOT a plane → slope verdict skipped, only step filter
    // applies. Catches the "ramp-edge vs floor" case where a 5-cell window
    // straddles a cliff: the cliff registers a huge residual, slope filter
    // backs off and lets step_max do the cliff classification correctly.
    // 5 cm: roughly 1/2 voxel — well within Mid-360 noise envelope on a
    // smooth ramp at 8 m range, comfortably outside a 50 cm cliff.
    slope_roughness_max_m_ = declare_parameter<double>("slope_roughness_max_m", 0.05);
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
    // Grounded-FREE-run test thresholds. With the new octomap-style projection
    // (no fan-fill shortcut), these gate FREE cells in columns without an
    // explicit occupied surface in trav z range. Permissive defaults:
    //   max_start_z = 0.30 m: first free voxel must be near the floor
    //   min_run_voxels = 1:   one free voxel is enough evidence
    // The previous strict defaults (0.25, 10) were tuned for the leaky fan
    // path where any column with even faint free evidence got marked free;
    // with leak-proof octomap-style query we can be permissive on the floor
    // side without re-introducing leak.
    free_surface_max_start_z_m_ =
        declare_parameter<double>("free_surface_max_start_z_m", 0.30);
    free_surface_min_run_voxels_ =
        declare_parameter<int>("free_surface_min_run_voxels", 1);

    // Legacy 2D-projection gate. When false (default), publish_traversability
    // is skipped entirely so an external pipeline (elevation_mapping_cupy +
    // grid_map_filters + grid_map_to_occupancy_grid adapter) can own
    // /<ns>/traversability_grid without competing publishers. voxels_3d and
    // voxel_cloud stay on unconditionally — CFPA2 3D frontier extraction
    // depends on them. Set true at launch for A/B comparison or fallback.
    // See docs/claude/plans/2026-05-14-trav-grid-rewrite.md Phase 0.
    enable_legacy_2d_proj_ = declare_parameter<bool>(
        "enable_legacy_2d_proj", false);

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

    // Build world-frame scan for ray-casting in traversability publisher.
    // Done outside the lock (pure math, no shared state) then swapped in.
    std::vector<nvblox::Vector3f> cloud_world;
    cloud_world.reserve(pts.size());
    for (const auto& p : pts)
      cloud_world.push_back(T * p);

    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      latest_cloud_world_ = std::move(cloud_world);
      latest_sensor_world_ = T.translation();
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
    Eigen::Vector3f robot_xyz, sensor_world;
    rclcpp::Time stamp;
    std::vector<nvblox::Vector3f> cloud_world;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_map_data_) return;
      robot_xyz    = latest_robot_xyz_;
      sensor_world = latest_sensor_world_;
      stamp        = now();
      cloud_world  = latest_cloud_world_;  // ~5000 × 12 bytes, fast copy
    }
    if (enable_legacy_2d_proj_) {
      publish_traversability(robot_xyz, stamp, cloud_world, sensor_world);
    }
    publish_voxels_3d(robot_xyz, stamp);

    if ((++publish_count_) % 20 == 1)
      RCLCPP_INFO(get_logger(),
                  "published %svoxels_3d #%u",
                  enable_legacy_2d_proj_ ? "traversability+" : "",
                  publish_count_);
  }

  // ============================================================
  // Block-based traversability publisher (2.5D)
  // ============================================================
  void publish_traversability(
      const Eigen::Vector3f& robot_xyz, const rclcpp::Time& stamp,
      const std::vector<nvblox::Vector3f>& cloud_world,
      const nvblox::Vector3f& sensor_world) {
    const float vs = static_cast<float>(voxel_size_m_);
    const float bs = vs * kVPS;  // block size in metres (8 voxels × 0.1 m = 0.8 m)
    const int   nxy = static_cast<int>(std::lround(trav_xy_extent_m_ / vs));
    const float half = nxy * vs * 0.5f;
    // World-fixed origin: pin once on first publish (or from launch params).
    // Subsequent frames use the SAME origin so cls_persist_ aligns 1:1 with
    // the published grid and historical observations don't get scrolled
    // out of view as the robot moves.
    if (!cls_persist_origin_locked_) {
      if (std::isfinite(trav_world_origin_x_) && std::isfinite(trav_world_origin_y_)) {
        // Caller-provided origin.
      } else {
        // Lock from first robot pose, centred on current location.
        trav_world_origin_x_ = robot_xyz.x() - half;
        trav_world_origin_y_ = robot_xyz.y() - half;
      }
      cls_persist_.assign(static_cast<size_t>(nxy) * nxy, -1);
      cls_persist_origin_locked_ = true;
      RCLCPP_INFO(get_logger(),
          "trav_grid world-fixed origin locked: (%.2f, %.2f) extent=%.1fm (%dx%d cells)",
          trav_world_origin_x_, trav_world_origin_y_, trav_xy_extent_m_, nxy, nxy);
    }
    const float ox = static_cast<float>(trav_world_origin_x_);
    const float oy = static_cast<float>(trav_world_origin_y_);

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
    // occ_bits[idx] is a z-bitmap of OCC voxels in the column (bit k ↔
    // z slice z_min + (k+0.5)*vs). Filled in Pass 1 alongside free_bits;
    // the "lowest stable surface" pass below derives H from this bitmap.
    std::vector<uint64_t> occ_bits(ncells, 0ULL);

    // --- Octomap-style query: 2D class purely from nvblox 3D occupancy ---
    //
    // Previous version used a 2D polar fan-fill shortcut (build r_min[bin]
    // from this scan, mark cells r < r_min as FREE). The fan-fill leaked
    // through walls whenever a bin had no hits — r_min stayed infinite,
    // fan extended to infinity, behind-wall cells got falsely marked FREE.
    // Patching with persistent-OCC blockers + dilation helped but still
    // a 2D approximation of 3D occlusion.
    //
    // The right thing (octomap-style): trust nvblox's persistent 3D
    // occupancy_layer entirely. It already does proper 3D raycasting per
    // scan: free voxels get carved along ray paths up to the hit, hit
    // voxels get OCC log-odds. Behind-wall voxels never receive any rays,
    // so they stay log_odds=0 (UNKNOWN). When we project to 2D, walls
    // naturally block — no leak possible.
    //
    // The classify below (existing per-block pass that builds H[idx] and
    // free_bits[idx]) reads directly from occupancy_layer. So all we need
    // to do here is REMOVE the fan-fill (no ray_covered needed) and let
    // the classify decide FREE vs UNK based on:
    //   - free_bits[idx] != 0 (column has carved free voxels) → FREE
    //   - free_bits[idx] == 0 AND H is NaN → UNK (no observation at all)
    //
    // This is the same logic octomap_server uses for its projected_map.
    // Walls block rays in 3D ⇒ free_bits == 0 behind walls ⇒ stays UNK.
    // No 2D approximation in the leak-prone direction.

    // --- Pass 1: accumulate OCC and FREE z-bitmaps per (x,y) column ---
    //
    // Surface-height definition (see Pass 1b below):
    // H[i,j] is the LOWEST OCC voxel z such that the next robot_clearance_m
    // worth of voxels above are non-OCC. This is the octomap-style
    // projected_map definition — what the robot can actually stand on with
    // headroom — and avoids the previous H = max{OCC} bug that picked
    // overhangs / ceilings / ramp-top voxels above an unrelated obstacle
    // stack and called them "surface".
    //
    // Pass 1 just fills occ_bits[idx] and free_bits[idx]; Pass 1b walks each
    // column's occ_bits from the LSB upward and applies the clearance test
    // to pick H. Decoupling collection from selection lets us change the
    // surface policy (lowest-stable, lowest-stable-with-FREE-support, etc.)
    // without touching the block-iteration code below.
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
            // Voxel z bin (column index 0..63). Both occ_bits and free_bits
            // use the same indexing so Pass 1b can intersect masks directly.
            const int kz = static_cast<int>(
                std::lround((wz - z_min) / vs - 0.5f));
            if (kz < 0 || kz >= 64) continue;
            if (lo > occ_lo_thresh) {
              // OCC voxel — require lo > occ_lo_thresh (≥2 consistent hits)
              // so stray single-scan returns don't create phantom surfaces.
              occ_bits[idx] |= (uint64_t{1} << kz);
            } else if (lo < 0.0f) {
              // FREE voxel. If a column has no occupied surface at all, a
              // grounded contiguous FREE run lets us infer floor support.
              free_bits[idx] |= (uint64_t{1} << kz);
            }
          }
        }
      }
    }

    // --- Pass 1b: derive H from occ_bits using "lowest stable surface" ---
    //
    // For each column, walk OCC bits from the LSB (lowest z) upward. The
    // first OCC voxel whose next `clearance_voxels` slices above are all
    // non-OCC (FREE or UNK) is the surface the robot can stand on. Voxels
    // higher up in the same column may be additional OCC (e.g. ceiling,
    // overhang, ramp meeting platform from below) — they don't affect H
    // because the robot is "below" them.
    //
    // Why "non-OCC" instead of "FREE": in the first few frames after the
    // robot starts, voxels above the floor are largely UNK (nvblox hasn't
    // accumulated enough hits yet). Requiring FREE evidence would leave
    // most columns with H=NaN and stall classification. Requiring only the
    // absence of OCC is permissive at startup and tightens as exploration
    // accumulates evidence — and the downstream step/slope filter still
    // catches genuinely-blocked surfaces.
    //
    // The old Pass 2 (clearance check that flipped cls to OCC if any OCC
    // existed in z ∈ (H, H+clearance]) is now redundant: by construction
    // the new H has clearance-of-non-OCC above it. Pass 2 deleted.
    {
      const int clearance_voxels = std::max(1,
          static_cast<int>(std::ceil(clearance / vs)));
      const uint64_t clearance_mask = (clearance_voxels >= 63)
          ? ~uint64_t{0}
          : ((uint64_t{1} << clearance_voxels) - uint64_t{1});

      for (size_t k = 0; k < ncells; ++k) {
        uint64_t bits = occ_bits[k];
        if (bits == 0ULL) continue;  // no OCC in column → H stays NaN
        while (bits != 0ULL) {
          const int bot = __builtin_ctzll(bits);
          // Mask of `clearance_voxels` slices ABOVE bot. Bits past 63
          // (out of column range) shift out and become 0, which is fine —
          // treats out-of-range z as "no OCC".
          const uint64_t above_mask = (bot + 1 >= 64)
              ? uint64_t{0}
              : (clearance_mask << (bot + 1));
          if ((occ_bits[k] & above_mask) == 0ULL) {
            // bot is the lowest OCC with non-OCC clearance above.
            H[k] = z_min + (static_cast<float>(bot) + 0.5f) * vs;
            break;
          }
          bits &= bits - 1;  // clear lowest set bit, try the next OCC up
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
          // No occupied surface in trav z range. Use ONLY nvblox's
          // persistent 3D occupancy state (octomap-style projection — see
          // comment block above the fan-fill removal). If nvblox has carved
          // any FREE voxels in this column, the rays actually traversed
          // the cell in 3D and we know it's open. If no FREE voxels,
          // either no rays reached this column (behind a wall in 3D) or
          // nvblox hasn't accumulated enough hits yet — either way stay
          // UNK. This is leak-proof because nvblox respects 3D occlusion.
          const uint64_t bits = free_bits[idx];
          if (bits == 0ULL) continue;  // UNK: no observation
          // Grounded FREE-run test: prefer "real floor" (low-z continuous
          // free run) over "air pocket above platform top". With dense
          // Mid-360 (vt=96), even one free voxel near the ground is solid
          // evidence; min_run_voxels=1 makes this permissive.
          const int first_k = __builtin_ctzll(bits);
          uint64_t shifted = bits >> first_k;
          int run_voxels = 0;
          while ((shifted & 1ULL) != 0ULL) { ++run_voxels; shifted >>= 1; }
          const float first_free_z = z_min + (static_cast<float>(first_k) + 0.5f) * vs;
          if (first_free_z <= free_surface_max_start_z &&
              run_voxels >= free_surface_min_run_voxels) {
            cls[idx] = 0;  // FREE (leave H as NaN; slope/step skips it)
          }
          continue;
        }
        if (cls[idx] != 100) cls[idx] = 0; // has surface, not blocked → traversable
      }
    }

    // --- Step + plane-fit slope filter ---
    //
    // The previous slope check compared H along 4 axis-aligned 5-cell
    // baselines and treated any |ΔH|/baseline > tan(slope_max) as OCC. That
    // formulation has no awareness of WHAT the baseline is crossing — at the
    // ramp's y-edge it can land on the floor 0.5 m below and report a 45°
    // "slope" that is in fact a cliff. The step filter catches cliffs at
    // 1-cell distance but not at 5-cell.
    //
    // Replacement: least-squares plane fit on the 5×5 window of valid H,
    // z(dx, dy) = a·dx + b·dy + c (dx, dy in CELL units). Two outputs:
    //   slope_tan  = √(a² + b²) / vs   ← world-space height gradient
    //   roughness  = RMS residual of the fit (metres)
    //
    // If roughness > slope_roughness_max_m, the local neighbourhood isn't a
    // plane (cliff / surface mix / sensor splatter) — skip the slope verdict
    // and let the step filter handle it. Otherwise OCC if slope_tan > tan(σ_max).
    //
    // Why this fixes ramp y-edges: at a ramp y-edge, half the window is ramp
    // and half is floor 0.5 m below. No plane fits both → residual ~ 0.25 m
    // → roughness >> 0.05 m threshold → slope verdict suppressed → step
    // filter cleanly flags the adjacent ramp-vs-floor cells (|ΔH| > 0.20 m).
    // Inside a smooth ramp the window is one plane → residual ≈ 0 →
    // slope_tan ≈ tan(14°) ≈ 0.249 < tan(30°) = 0.577 → FREE.
    const float step_max  = static_cast<float>(step_max_m_);
    const float tan_smax  = std::tan(static_cast<float>(slope_max_deg_ * M_PI / 180.0));
    const float roughness_max = static_cast<float>(slope_roughness_max_m_);
    const int   half_win  = 2;        // 5×5 window
    const int   N_min     = 10;       // need ≥10 valid H samples for a stable fit

    for (int j = 0; j < nxy; ++j) {
      for (int i = 0; i < nxy; ++i) {
        const size_t idx = static_cast<size_t>(j) * nxy + i;
        if (cls[idx] != 0) continue;
        const float h0 = H[idx];
        if (std::isnan(h0)) continue;

        // Step check: |dh| > step_max between adjacent cells = curb/stair/cliff.
        bool blocked = false;
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

        // Plane fit on 5×5 valid-H window.
        // Accumulate normal-equation sums S_* over (dx, dy, h) samples.
        float Sx = 0, Sy = 0, Sh = 0;
        float Sxx = 0, Syy = 0, Sxy = 0;
        float Sxh = 0, Syh = 0;
        int   N   = 0;
        for (int dj = -half_win; dj <= half_win; ++dj) {
          const int jj = j + dj;
          if (jj < 0 || jj >= nxy) continue;
          for (int di = -half_win; di <= half_win; ++di) {
            const int ii = i + di;
            if (ii < 0 || ii >= nxy) continue;
            const float hn = H[static_cast<size_t>(jj) * nxy + ii];
            if (std::isnan(hn)) continue;
            const float x = static_cast<float>(di);
            const float y = static_cast<float>(dj);
            Sx  += x;     Sy  += y;     Sh  += hn;
            Sxx += x * x; Syy += y * y; Sxy += x * y;
            Sxh += x * hn; Syh += y * hn;
            ++N;
          }
        }
        if (N < N_min) continue;  // too sparse — leave cls=0, trust step filter

        // Solve  [Sxx Sxy Sx] [a]   [Sxh]
        //        [Sxy Syy Sy] [b] = [Syh]   (Eigen 3×3 QR, robust to near-degeneracy)
        //        [Sx  Sy  N ] [c]   [Sh ]
        Eigen::Matrix3f A;
        A << Sxx, Sxy, Sx,
             Sxy, Syy, Sy,
             Sx,  Sy,  static_cast<float>(N);
        Eigen::Vector3f rhs(Sxh, Syh, Sh);
        // Cheap singularity guard: a perfectly collinear window has a near-
        // zero determinant on the upper-left 2×2 plus N — skip rather than
        // pivot through numerical noise.
        const float det2 = Sxx * Syy - Sxy * Sxy;
        if (std::abs(det2) < 1e-6f) continue;
        const Eigen::Vector3f abc = A.colPivHouseholderQr().solve(rhs);
        const float a = abc(0), b = abc(1), c = abc(2);

        // RMS residual (metres). Walk the same window once more.
        float resid_sq = 0;
        for (int dj = -half_win; dj <= half_win; ++dj) {
          const int jj = j + dj;
          if (jj < 0 || jj >= nxy) continue;
          for (int di = -half_win; di <= half_win; ++di) {
            const int ii = i + di;
            if (ii < 0 || ii >= nxy) continue;
            const float hn = H[static_cast<size_t>(jj) * nxy + ii];
            if (std::isnan(hn)) continue;
            const float pred = a * static_cast<float>(di) + b * static_cast<float>(dj) + c;
            const float r = hn - pred;
            resid_sq += r * r;
          }
        }
        const float roughness = std::sqrt(resid_sq / static_cast<float>(N));
        if (roughness > roughness_max) continue;  // not a plane — defer to step filter

        // dx, dy were in cell units; convert gradient to world frame.
        const float slope_tan = std::sqrt(a * a + b * b) / vs;
        if (slope_tan > tan_smax) cls[idx] = 100;
      }
    }

    // --- 3×3 median filter (FREE/UNK only — never touch OCC) ---
    // Earlier versions ran a vanilla 3×3 median that ERODED OCC cells:
    // a 2-cell-thick wall (outer walls in demo_ramp at 0.2 m thickness)
    // has only 2 of 9 neighbours OCC → median = FREE → silent wall erase.
    // Even the wall-preserving variant erased ISOLATED OCC dots, which
    // matters when a wall is sparsely sampled (e.g. west/east walls at
    // 21 % OCC coverage have many singleton OCC cells along their length).
    // Eroded walls let the blind-disk pass below leak FREE past them.
    //
    // Fix: only smooth FREE↔UNK cells; OCC is sacred and stays. That
    // means we lose the "salt-and-pepper OCC removal" property, but the
    // OCC observations are themselves rare on partially-observed walls
    // and we cannot afford to throw any of them away.
    {
      std::vector<int8_t> filtered(cls);
      for (int j = 1; j < nxy - 1; ++j) {
        for (int i = 1; i < nxy - 1; ++i) {
          const size_t idx = static_cast<size_t>(j) * nxy + i;
          if (cls[idx] == 100) { filtered[idx] = 100; continue; }
          int8_t nb[9];
          int n = 0;
          for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di)
              nb[n++] = cls[static_cast<size_t>(j + dj) * nxy + (i + di)];
          std::sort(nb, nb + 9);
          filtered[idx] = nb[4];
        }
      }
      cls = std::move(filtered);
    }

    // --- Mid-360 blind-zone fill (flood-fill from FREE within disk) ---
    // Mid-360 V-FOV is -7° to +52°. With the sensor at world z≈0.4 m, the
    // lowest beam intersects the floor at horizontal distance
    //   d = z_sensor / tan(7°) ≈ 0.4 / 0.123 ≈ 3.25 m.
    // So there is a geometric donut hole — no ground returns possible
    // inside ~3.25 m of the robot. We force-FREE cells in this donut so
    // CFPA2's BFS reachability and Nav2's path planning don't break.
    //
    // The previous Bresenham-occlusion approach broke when walls are
    // SPARSELY observed: west/east walls measured at only 21–22 % OCC
    // coverage, so a Bresenham ray easily threads through unobserved
    // (UNK) wall cells and force-FREEs the space behind. Worse, that
    // leaked FREE is then persisted into cls_persist_ and stays forever.
    //
    // Replacement algorithm: **only grow FREE into UNK from existing
    // FREE neighbours, restricted to within the disk**. Conceptually:
    // start from the FREE ring at the disk's outer edge (where nvblox
    // has carved the floor at r ≈ 3.25 m+), and flood-fill inward
    // through UNK. Walls — even sparsely observed ones — block expansion
    // because their OCC cells are never crossed. Past-wall regions have
    // no contact with the FREE seed → stay UNK. The donut itself fills
    // because every donut cell eventually touches the outer FREE ring.
    //
    // Limitation: with ~30-cell disk radius, the flood-fill needs up to
    // ~30 passes to converge. That's O(r·r²) = O(r³) ≈ 27 000 ops at
    // r=30 — cheap. We iterate to a fixed cap to bound worst-case cost.
    {
      const float blind_radius_m = 3.0f;
      const int   blind_r_cells  = static_cast<int>(blind_radius_m / vs);
      const int   blind_r2_cells = blind_r_cells * blind_r_cells;
      const int   ci_r = static_cast<int>((robot_xyz.x() - ox) / vs);
      const int   cj_r = static_cast<int>((robot_xyz.y() - oy) / vs);
      const bool  have_persist = (cls_persist_.size() == ncells);

      // Robot's own cell is provably FREE (we're standing on it). Seed it
      // so the flood-fill has a kernel even when nvblox has carved nothing
      // around the robot yet.
      if (ci_r >= 0 && ci_r < nxy && cj_r >= 0 && cj_r < nxy) {
        const size_t k = static_cast<size_t>(cj_r) * nxy + ci_r;
        if (cls[k] != 100) cls[k] = 0;
      }

      auto in_disk = [&](int di, int dj) -> bool {
        return di * di + dj * dj <= blind_r2_cells;
      };

      // is_free reads current frame cls OR persistent (historical FREE
      // observations contribute to the seed set even if the cell hasn't
      // been carved this frame).
      auto is_free = [&](int ii, int jj) -> bool {
        if (ii < 0 || ii >= nxy || jj < 0 || jj >= nxy) return false;
        const size_t k = static_cast<size_t>(jj) * nxy + ii;
        if (cls[k] == 0) return true;
        if (have_persist && cls_persist_[k] == 0) return true;
        return false;
      };

      // Bounded-iteration flood-fill. Each pass propagates FREE one cell
      // inward; cap at blind_r_cells passes (diameter / 2 is enough to
      // saturate the disk).
      const int max_passes = blind_r_cells + 2;
      for (int pass = 0; pass < max_passes; ++pass) {
        bool changed = false;
        for (int dj = -blind_r_cells; dj <= blind_r_cells; ++dj) {
          const int cj = cj_r + dj;
          if (cj < 0 || cj >= nxy) continue;
          for (int di = -blind_r_cells; di <= blind_r_cells; ++di) {
            if (!in_disk(di, dj)) continue;
            const int ci = ci_r + di;
            if (ci < 0 || ci >= nxy) continue;
            const size_t idx = static_cast<size_t>(cj) * nxy + ci;
            if (cls[idx] == 0 || cls[idx] == 100) continue;  // already FREE or OCC
            // Promote UNK → FREE if any 4-neighbour is FREE (and still in disk
            // for clean boundary — neighbours outside the disk shouldn't seed).
            if (is_free(ci + 1, cj) || is_free(ci - 1, cj) ||
                is_free(ci, cj + 1) || is_free(ci, cj - 1)) {
              cls[idx] = 0;
              changed = true;
            }
          }
        }
        if (!changed) break;
      }
    }

    // --- Merge this frame's classify into the persistent world-fixed grid ---
    // Persistence policy: any non-UNK class from this frame wins (latest
    // observation rules — walls can be cleared if a frontier walks through;
    // FREE accumulates as the robot explores). UNK from this frame means
    // "no new info" — keep whatever cls_persist_ already had. This is the
    // "sticky" map that retains historical FREE observations as the robot
    // moves away. Cells outside the robot's current sensor footprint stay
    // at their last known value.
    //
    // If a wall has been cleared (e.g. door opened mid-sim), it stays OCC
    // here until a new observation explicitly downgrades it via the
    // classify loop — slope/step filter could mark it FREE if the surface
    // beneath becomes traversable. That's the right behaviour for the
    // exploration use case.
    if (cls_persist_.size() == ncells) {
      for (size_t k = 0; k < ncells; ++k) {
        if (cls[k] != -1) cls_persist_[k] = cls[k];
      }
      g.data.assign(cls_persist_.begin(), cls_persist_.end());
    } else {
      g.data.assign(cls.begin(), cls.end());
    }
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
            // Filter ground-level occupied voxels from the visual cloud only.
            // They remain in voxels_3d (CFPA2 IG) and the occupancy map
            // (free-space carving) but cluttering the 3D display.
            if (lo > 0.0f && wz >= static_cast<float>(ground_z_max_m_))
              occ_pts_.emplace_back(wx, wy, wz);
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
  double trav_world_origin_x_, trav_world_origin_y_;
  // Persistent trav_grid cell classes (world-fixed, accumulates across
  // frames). Values: -1 UNK / 0 FREE / 100 OCC. Same size as nxy*nxy.
  // Origin pinned on first publish from robot pose or from params.
  std::vector<int8_t> cls_persist_;
  bool cls_persist_origin_locked_{false};
  bool enable_legacy_2d_proj_{false};
  double robot_clearance_m_, slope_max_deg_, step_max_m_;
  double slope_roughness_max_m_;
  double voxel_xy_extent_m_, voxel_z_extent_m_, voxel_z_origin_m_;
  double ground_z_max_m_, occ_lo_thresh_;
  double free_surface_max_start_z_m_;
  int free_surface_min_run_voxels_;

  // Persistent ray_covered buffer (per-frame fan-fill OR'd into here).
  // Robot-centric, shifts cell-wise as the grid origin moves. Cells that
  // scroll off the grid are forgotten; cells that scroll on start false.
  // Fills the angular gaps that any single Mid-360 scan leaves (outer
  // ring of bins only seeing ceiling, dropped by the z filter).
  std::vector<uint8_t> ray_covered_persist_;
  float persist_ox_{std::numeric_limits<float>::quiet_NaN()};
  float persist_oy_{std::numeric_limits<float>::quiet_NaN()};
  int   persist_nxy_{0};

  std::unique_ptr<nvblox::Mapper> mapper_;
  std::unique_ptr<nvblox::Lidar>  lidar_;

  std::mutex state_mtx_;
  Odometry latest_odom_;
  bool have_odom_{false};
  Eigen::Vector3f latest_robot_xyz_{0.f, 0.f, 0.f};
  bool have_map_data_{false};

  std::vector<Eigen::Vector3f> occ_pts_;  // scratch buffer for PointCloud2 build

  // World-frame scan + sensor position, updated each cloud_cb; copied by
  // publish_outputs for 2D Bresenham ray-coverage computation.
  std::vector<nvblox::Vector3f> latest_cloud_world_;
  nvblox::Vector3f latest_sensor_world_{0.f, 0.f, 0.f};

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
  try {
    rclcpp::spin(std::make_shared<nvbf::MapperNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mapper_node"),
                 "Unhandled exception in MapperNode: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
