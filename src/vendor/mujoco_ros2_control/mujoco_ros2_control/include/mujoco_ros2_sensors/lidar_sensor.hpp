/**
 * @file lidar_sensor.hpp
 * @brief LiDAR raycast sensor using mj_multiRay inside the mujoco_ros2_control plugin.
 *
 * Casts rays from a MuJoCo site using the live mjData, eliminating the sync
 * lag that occurs when raycasting in a separate process.  Publishes
 * sensor_msgs/PointCloud2 on ~/registered_scan.
 */
#ifndef MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP

#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <random>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace mujoco_ros2_sensors {

/// Configuration passed from MujocoRos2Sensors to the LidarSensor constructor.
struct LidarSensorConfig {
    std::string site_name  = "livox_mid360";  ///< MuJoCo site for origin + orientation
    std::string body_name  = "base_link";     ///< Body to exclude from raycasts
    std::string frame_id   = "livox_mid360";  ///< PointCloud2 header.frame_id
    int    hz_samples      = 720;             ///< Horizontal ray count
    int    vt_samples      = 16;              ///< Vertical ray count
    double h_fov_deg       = 360.0;           ///< Horizontal FOV (degrees)
    double v_min_deg       = -7.0;            ///< Vertical FOV lower bound (degrees)
    double v_max_deg       = 52.0;            ///< Vertical FOV upper bound (degrees)
    double range_min       = 0.05;            ///< Minimum range (m)
    double range_max       = 20.0;            ///< Maximum range (m)
    double publish_rate    = 10.0;            ///< Publish rate (Hz)

    /// Optional path to Livox-style scan-pattern CSV (Time, Azimuth deg, Zenith deg).
    /// When non-empty, the sensor switches from uniform hz*vt grid to per-frame
    /// non-repetitive replay of `n_rays_per_frame` directions sliced from the CSV.
    /// This reproduces the real Risley-prism behaviour (coverage fills in over
    /// multiple frames), which is required for ETH elevation_mapping_cupy's
    /// visibility cleanup + drift compensation to behave the same in sim and real.
    std::string scan_pattern_csv = "";
    int    n_rays_per_frame      = 20000;     ///< Rays per published frame (CSV mode). Real Mid-360 = 200k pts/s / 10 Hz.
    double range_noise_stddev    = 0.0;       ///< Gaussian noise σ added to each range (m). Real Mid-360 ≈ 0.02.
};

class LidarSensor {
public:
    /**
     * @param node   ROS 2 node (owns timer, publisher, parameters)
     * @param model  Shared MuJoCo model pointer (same as physics sim)
     * @param data   Shared MuJoCo data pointer  (same as physics sim)
     * @param config LiDAR configuration
     */
    LidarSensor(rclcpp::Node::SharedPtr &node,
                mjModel *model, mjData *data,
                const LidarSensorConfig &config,
                std::mutex *sim_step_mtx = nullptr);

private:
    void update();

    rclcpp::Node::SharedPtr       nh_;
    rclcpp::TimerBase::SharedPtr  timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    mjModel *model_  = nullptr;
    mjData  *data_   = nullptr;

    // MuJoCo IDs
    int site_id_ = -1;
    int body_id_ = -1;

    // Ray configuration
    int    n_rays_;
    double range_min_;
    double range_max_;
    std::string frame_id_;

    // Ray directions in LiDAR-local frame for the CURRENT frame  (n_rays_ x 3, row-major).
    // In uniform-grid mode this is precomputed once in the ctor.
    // In CSV mode it is refilled per-frame from `csv_dirs_local_`.
    std::vector<double> ray_dirs_local_;

    // Reusable buffers for mj_multiRay output
    std::vector<double> ray_dist_;
    std::vector<int>    ray_geomid_;

    // CSV scan-pattern replay state (empty / 0 when uniform-grid mode is in use).
    std::vector<double> csv_dirs_local_;       ///< All CSV unit-vectors, row-major (total_samples x 3).
    int                 csv_total_samples_ = 0;
    int                 csv_offset_        = 0; ///< Wrap-around cursor over csv_dirs_local_.
    bool                csv_mode_          = false;

    // Range-noise Gaussian (σ from cfg.range_noise_stddev). When σ=0 we leave the
    // distribution constructed but skip the noise add to save the RNG call.
    double                                 range_noise_stddev_ = 0.0;
    std::mt19937                           rng_;
    std::normal_distribution<double>       noise_dist_;

    // Mutex shared with physics loop to prevent concurrent mjData access
    std::mutex *sim_step_mtx_ = nullptr;
};

}  // namespace mujoco_ros2_sensors

#endif  // MUJOCO_ROS2_CONTROL_LIDAR_SENSOR_HPP
