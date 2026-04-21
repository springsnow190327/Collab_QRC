#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"

enum CloudType {
    FREE_CLOUD = 0,
    OBS_CLOUD  = 1
};

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float floor_height;
    float cell_length;
    float cell_height;
    float grid_max_length;
    float grid_max_height;
    // local terrain height map
    float height_voxel_dim;
};

class MapHandler {

public:
    MapHandler() = default;
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos);

    void AdjustNodesHeight(const NodePtrStack& nodes);

    void AdjustCTNodeHeight(const CTNodeStack& ctnodes);

    static float TerrainHeightOfPoint(const Point3D& p, 
                                      bool& is_matched, 
                                      const bool& is_search);

    static bool IsNavPointOnTerrainNeighbor(const Point3D& p, const bool& is_extend);

    static float NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated);

    /**
     * @brief Multi-layer: return the layer-representative height nearest to ref_z
     * at the xy cell of `point`.
     * Only meaningful when FARUtil::IsMultiLayer is true. For single-layer mode,
     * falls back to the existing single-height cell value.
     */
    static float NearestLayerHeightForZ(const Point3D& point, const float& ref_z, bool& is_matched);

    /**
     * @brief Multi-layer: fetch all layer-representative heights at the xy cell
     * of `point`, sorted ascending. Empty vector if cell is unoccupied or
     * multi-layer snapshot has not been populated.
     */
    static std::vector<float> GetLayerHeights(const Point3D& point);

    /**
     * @brief Calculate the terrain height of a given point and radius around it
     * @param p A given position
     * @param radius The radius distance around the given posiitn p
     * @param minH[out] The mininal terrain height in the radius
     * @param maxH[out] The maximal terrain height in the radius
     * @param is_match[out] Whether or not find terrain association in radius
     * @return The average terrain height
     */
    template <typename Position>
    static inline float NearestHeightOfRadius(const Position& p, const float& radius, float& minH, float& maxH, bool& is_matched) {
        std::vector<int> pIdxK;
        std::vector<float> pdDistK;
        PCLPoint pcl_p;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        minH = maxH = p.z;
        is_matched = false;
        if (kdtree_terrain_clould_->radiusSearch(pcl_p, radius, pIdxK, pdDistK) > 0) {
            float avgH = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]].intensity;
            minH = maxH = avgH;
            for (std::size_t i=1; i<pIdxK.size(); i++) {
                const float temp = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[i]].intensity;
                if (temp < minH) minH = temp;
                if (temp > maxH) maxH = temp;
                avgH += temp;
            }
            avgH /= (float)pIdxK.size();
            is_matched = true;
            return avgH;
        }
        return p.z;
    }

    /** Update global cloud grid with incoming clouds 
     * @param CloudInOut incoming cloud ptr and output valid in range points
    */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    /** Extract Surrounding Free & Obs clouds
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);

    /**
     * @brief Multi-layer: partition the surround obstacle cloud into per-layer
     * point clouds based on the layer heights discovered by
     * SnapshotMultilayerHeights and anchored to the robot's current z. Layers
     * are returned sorted ascending by z. When IsMultiLayer is false this
     * returns a single layer containing the full obs cloud (with z = robot.z).
     * @param obsLayersOut    output vector of per-layer PCL clouds
     * @param layerCentersOut output vector of layer-representative z values
     */
    void GetSurroundObsCloudLayers(std::vector<PointCloudPtr>& obsLayersOut,
                                   std::vector<float>& layerCentersOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param center the position of the grid that want to extract
     * @param cloudOut output cloud ptr
     * @param type choose free or obstacle cloud for extraction
     * @param is_large whether or not using the surrounding cells
    */
    void GetCloudOfPoint(const Point3D& center, 
                         const PointCloudPtr& CloudOut, 
                         const CloudType& type,
                         const bool& is_large);

    /**
     * Get neihbor cells center positions
     * @param neighbor_centers[out] neighbor centers stack
    */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor cells center positions
     * @param occupancy_centers[out] occupanied cells center stack
    */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param obsCloud obstacle cloud points that need to be removed
    */ 
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /**
     * @brief Reset Current Grip Map Clouds
     */
    void ResetGripMapCloud();

    /**
     * @brief Clear the cells that from the robot position to the given position
     * @param point Give point location
     */
    void ClearObsCellThroughPosition(const Point3D& point);

private:
    MapHandlerParams map_params_;
    int neighbor_Lnum_, neighbor_Hnum_;
    Eigen::Vector3i robot_cell_sub_;
    int INFLATE_N;
    bool is_init_ = false;
    PointCloudPtr flat_terrain_cloud_;
    static PointKdTreePtr kdtree_terrain_clould_;

    template <typename Position>
    static inline float NearestHeightOfPoint(const Position& p, float& dist_square) {
        // Find the nearest node in graph
        std::vector<int> pIdxK(1);
        std::vector<float> pdDistK(1);
        PCLPoint pcl_p;
        dist_square = FARUtil::kINF;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        if (kdtree_terrain_clould_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
            pcl_p = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]];
            dist_square = pdDistK[0];
            return pcl_p.intensity;
        }
        return p.z;
    }

    void SetTerrainHeightGridOrigin(const Point3D& robot_pos);

    void TraversableAnalysis(const PointCloudPtr& terrainHeightOut);

    /**
     * @brief Populate `terrain_multilayer_heights_` from the current contents
     * of `terrain_height_grid_` BEFORE TraversableAnalysis collapses cells.
     * Groups each cell's z-hits into layer clusters using `floor_height / 2`
     * as the clustering tolerance, stores sorted cluster means.
     * No-op unless FARUtil::IsMultiLayer is true.
     */
    void SnapshotMultilayerHeights();

    inline void AssignFlatTerrainCloud(const PointCloudPtr& terrainRef, PointCloudPtr& terrainFlatOut) {
        const int N = terrainRef->size();
        terrainFlatOut->resize(N);
        for (int i = 0; i<N; i++) {
            PCLPoint pcl_p = terrainRef->points[i];
            pcl_p.intensity = pcl_p.z, pcl_p.z = 0.0f;
            terrainFlatOut->points[i] = pcl_p;
        }
    }

    inline void Expansion2D(const Eigen::Vector3i& csub, std::vector<Eigen::Vector3i>& subs, const int& n) {
        subs.clear();
        for (int ix=-n; ix<=n; ix++) {
            for (int iy=-n; iy<=n; iy++) {
                Eigen::Vector3i sub = csub;
                sub.x() += ix, sub.y() += iy;
                subs.push_back(sub); 
            }
        }
    }

    void ObsNeighborCloudWithTerrain(std::unordered_set<int>& neighbor_obs,
                                     std::unordered_set<int>& extend_terrain_obs);

    std::unordered_set<int> neighbor_free_indices_;        // surrounding free cloud grid indices stack
    static std::unordered_set<int> neighbor_obs_indices_;  // surrounding obs cloud grid indices stack
    static std::unordered_set<int> extend_obs_indices_;    // extended surrounding obs cloud grid indices stack

    std::vector<int> global_visited_induces_;
    std::vector<int> util_obs_modified_list_;
    std::vector<int> util_free_modified_list_;
    std::vector<int> util_remove_check_list_;
    static std::vector<int> terrain_grid_occupy_list_;
    static std::vector<int> terrain_grid_traverse_list_;

    
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_free_cloud_grid_;
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_obs_cloud_grid_;
    static std::unique_ptr<grid_ns::Grid<std::vector<float>>> terrain_height_grid_;
    // Multi-layer snapshot: per-cell sorted layer-representative heights,
    // preserved across TraversableAnalysis. Only populated when IsMultiLayer.
    static std::unique_ptr<grid_ns::Grid<std::vector<float>>> terrain_multilayer_heights_;

};

#endif