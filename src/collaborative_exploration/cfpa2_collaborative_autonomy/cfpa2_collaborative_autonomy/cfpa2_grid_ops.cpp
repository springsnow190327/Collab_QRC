/*
 * cfpa2_grid_ops.cpp — Fast grid operations for CFPA2 frontier exploration.
 *
 * Compiled as a shared library, loaded by Python via ctypes.
 * Provides: frontier extraction, BFS distance transform, batch info-gain.
 *
 * Build:
 *   g++ -O2 -shared -fPIC -o cfpa2_grid_ops.so cfpa2_grid_ops.cpp
 */

#include <cstdint>
#include <cstring>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

/* 8-connected neighbor offsets */
static constexpr int DX8[8] = {1, -1, 0, 0, 1, -1, 1, -1};
static constexpr int DY8[8] = {0, 0, 1, -1, 1, 1, -1, -1};

/* 4-connected neighbor offsets */
static constexpr int DX4[4] = {1, -1, 0, 0};
static constexpr int DY4[4] = {0, 0, 1, -1};

extern "C" {

/* ------------------------------------------------------------------ */
/*  extract_frontiers                                                  */
/*                                                                     */
/*  free_val:      grid value that means "free" (exact match)          */
/*  unknown_val:   grid value that means "unknown" (exact match)       */
/*  occ_threshold: grid values >= this are "occupied"                  */
/* ------------------------------------------------------------------ */

int extract_frontiers(
    const int8_t *grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    int stride,
    float min_cluster_area,
    int clearance_cells,
    int8_t free_val, int8_t unknown_val, int8_t occ_threshold,
    float *out_x, float *out_y,
    int max_out
) {
    if (!grid || W <= 2 || H <= 2 || max_out <= 0) return 0;

    int N = W * H;
    std::vector<uint8_t> frontier_mask(N, 0);
    std::vector<int> frontier_indices;
    frontier_indices.reserve(N / 10);

    /* Step 1: Identify frontier cells (free cells adjacent to unknown) */
    for (int gy = 1; gy < H - 1; gy++) {
        int row = gy * W;
        for (int gx = 1; gx < W - 1; gx++) {
            int idx = row + gx;
            /* Treat any non-unknown cell below occ_threshold as free */
            {
                int8_t v = grid[idx];
                if (v == unknown_val || v < 0 || v >= occ_threshold) continue;
            }

            bool found_unknown = false;
            for (int d = 0; d < 8; d++) {
                int ni = (gy + DY8[d]) * W + (gx + DX8[d]);
                if (grid[ni] == unknown_val) {
                    found_unknown = true;
                    break;
                }
            }
            if (!found_unknown) continue;

            frontier_mask[idx] = 1;
            frontier_indices.push_back(idx);
        }
    }

    if (frontier_indices.empty()) return 0;

    /* Step 2: BFS clustering + output */
    std::vector<uint8_t> visited(N, 0);
    std::queue<int> q;
    std::vector<int> component;
    int out_count = 0;
    float cell_area = res * res;

    for (int seed_idx : frontier_indices) {
        if (visited[seed_idx] || !frontier_mask[seed_idx]) continue;

        visited[seed_idx] = 1;
        q.push(seed_idx);
        component.clear();

        while (!q.empty()) {
            int ci = q.front(); q.pop();
            component.push_back(ci);

            int cx = ci % W;
            int cy = ci / W;
            for (int d = 0; d < 8; d++) {
                int nx = cx + DX8[d];
                int ny = cy + DY8[d];
                if (nx <= 0 || ny <= 0 || nx >= W - 1 || ny >= H - 1) continue;
                int ni = ny * W + nx;
                if (visited[ni] || !frontier_mask[ni]) continue;
                visited[ni] = 1;
                q.push(ni);
            }
        }

        /* Filter by cluster area */
        float cluster_area = static_cast<float>(component.size()) * cell_area;
        if (cluster_area + 1e-9f < min_cluster_area) continue;

        /* Output subsampled frontier points with clearance check */
        for (int i = 0; i < static_cast<int>(component.size()); i++) {
            if ((i % stride) != 0) continue;

            int ci = component[i];
            int gx = ci % W;
            int gy = ci / W;

            /* Obstacle clearance check */
            if (clearance_cells > 0) {
                bool too_close = false;
                for (int dy = -clearance_cells; dy <= clearance_cells && !too_close; dy++) {
                    int ny = gy + dy;
                    if (ny < 0 || ny >= H) continue;
                    for (int dx = -clearance_cells; dx <= clearance_cells; dx++) {
                        int nx = gx + dx;
                        if (nx < 0 || nx >= W) continue;
                        if (grid[ny * W + nx] >= occ_threshold) {
                            too_close = true;
                            break;
                        }
                    }
                }
                if (too_close) continue;
            }

            out_x[out_count] = origin_x + (static_cast<float>(gx) + 0.5f) * res;
            out_y[out_count] = origin_y + (static_cast<float>(gy) + 0.5f) * res;
            out_count++;

            if (out_count >= max_out) return out_count;
        }
    }

    return out_count;
}

/* ------------------------------------------------------------------ */
/*  distance_transform (BFS)                                           */
/*                                                                     */
/*  Shortest-path distance (cells) from start to every reachable       */
/*  free cell.  Output: flat int array, -1 = unreachable.              */
/* ------------------------------------------------------------------ */

/* Forward declaration */
void distance_transform_range(
    const int8_t *grid, int W, int H, int sx, int sy,
    int8_t unknown_val, int8_t occ_threshold, int *dist_out);

void distance_transform(
    const int8_t *grid,
    int W, int H,
    int sx, int sy,
    int8_t free_val,
    int *dist_out
) {
    /* Legacy exact-match signature — forward to range-based version.
       Treat any cell with 0 <= v < 50 as traversable (matches frontier
       extraction logic). */
    distance_transform_range(grid, W, H, sx, sy, /*unknown_val=*/ -1, /*occ_threshold=*/ 50, dist_out);
}

/* Range-based BFS: traversable = not unknown AND 0 <= v < occ_threshold */
void distance_transform_range(
    const int8_t *grid,
    int W, int H,
    int sx, int sy,
    int8_t unknown_val, int8_t occ_threshold,
    int *dist_out
) {
    int N = W * H;
    std::memset(dist_out, 0xFF, N * sizeof(int));  /* -1 (0xFFFFFFFF) = unreachable */

    if (sx < 0 || sx >= W || sy < 0 || sy >= H) return;

    auto is_free = [&](int idx) -> bool {
        int8_t v = grid[idx];
        return v != unknown_val && v >= 0 && v < occ_threshold;
    };

    int sidx = sy * W + sx;

    /* If start is not free, search nearby for a free cell */
    if (!is_free(sidx)) {
        bool found = false;
        for (int r = 1; r <= 12 && !found; r++) {
            for (int dy = -r; dy <= r && !found; dy++) {
                int ny = sy + dy;
                if (ny < 0 || ny >= H) continue;
                for (int dx = -r; dx <= r; dx++) {
                    int nx = sx + dx;
                    if (nx < 0 || nx >= W) continue;
                    int ni = ny * W + nx;
                    if (is_free(ni)) {
                        sx = nx; sy = ny; sidx = ni;
                        found = true;
                        break;
                    }
                }
            }
        }
        if (!found) return;
    }

    std::queue<int> q;
    dist_out[sidx] = 0;
    q.push(sidx);

    while (!q.empty()) {
        int ci = q.front(); q.pop();
        int cx = ci % W;
        int cy = ci / W;

        for (int d = 0; d < 4; d++) {
            int nx = cx + DX4[d];
            int ny = cy + DY4[d];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            int ni = ny * W + nx;
            if (dist_out[ni] != -1) continue;
            if (!is_free(ni)) continue;
            dist_out[ni] = dist_out[ci] + 1;
            q.push(ni);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  batch_info_gain                                                    */
/*                                                                     */
/*  Computes info gain for N frontier goals.                           */
/*  Gain = count of unknown cells within a radius-R box of each goal.  */
/* ------------------------------------------------------------------ */

void batch_info_gain(
    const int8_t *grid,
    int W, int H,
    float res, float origin_x, float origin_y,
    const float *goal_x, const float *goal_y, int n_goals,
    int radius,
    int8_t unknown_val,
    float *gains_out
) {
    for (int i = 0; i < n_goals; i++) {
        /* World → grid */
        int gx = static_cast<int>((goal_x[i] - origin_x) / res);
        int gy = static_cast<int>((goal_y[i] - origin_y) / res);

        float gain = 0.0f;
        int y0 = std::max(0, gy - radius);
        int y1 = std::min(H, gy + radius + 1);
        int x0 = std::max(0, gx - radius);
        int x1 = std::min(W, gx + radius + 1);

        for (int yy = y0; yy < y1; yy++) {
            int row = yy * W;
            for (int xx = x0; xx < x1; xx++) {
                if (grid[row + xx] == unknown_val) {
                    gain += 1.0f;
                }
            }
        }
        gains_out[i] = gain;
    }
}

} /* extern "C" */
