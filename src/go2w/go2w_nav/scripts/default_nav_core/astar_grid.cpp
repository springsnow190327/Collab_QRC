/*
 * astar_grid.cpp — Fast A* on a 2D grid with optional proximity cost.
 *
 * Compiled as a shared library, loaded by Python via ctypes.
 * Provides extern "C" interface for grid-based A* pathfinding.
 *
 * Build:
 *   g++ -O2 -shared -fPIC -o astar_grid.so astar_grid.cpp
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <queue>
#include <vector>

/* ------------------------------------------------------------------ */
/*  A* on a flat grid with proximity cost (row-major)                 */
/* ------------------------------------------------------------------ */

struct Node {
    float f;   // f = g + h
    float g;
    int x, y;
    bool operator>(const Node &o) const { return f > o.f; }
};

static inline float heuristic(int ax, int ay, int gx, int gy) {
    float dx = static_cast<float>(ax - gx);
    float dy = static_cast<float>(ay - gy);
    return std::sqrt(dx * dx + dy * dy);
}

static constexpr int DX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
static constexpr int DY[] = {0, 0, -1, 1, -1, 1, -1, 1};
static constexpr float COST[] = {1.0f, 1.0f, 1.0f, 1.0f,
                                  1.4142135f, 1.4142135f, 1.4142135f, 1.4142135f};

extern "C" {

/*
 * astar_grid — Run A* from (sx,sy) to (gx,gy) on a grid.
 *
 * Parameters:
 *   blocked      — row-major uint8 grid (H rows × W cols), nonzero = blocked
 *   W, H         — grid dimensions
 *   sx, sy       — start cell
 *   gx, gy       — goal cell
 *   path_x_out   — output buffer for x-coordinates of path
 *   path_y_out   — output buffer for y-coordinates of path
 *   max_path_len — capacity of output buffers
 *   max_cells    — search budget (max nodes to expand)
 *   cells_explored_out — output: number of cells expanded
 *   cost_grid    — optional row-major uint8 cost array (NULL = no extra cost).
 *                  Values 0-252 are added to edge cost as cost_grid[ni] / 252.0.
 *                  This adds up to 1.0 extra cost per cell traversed.
 *
 * Returns:
 *   path length (number of cells), or 0 if no path found.
 *   Path is from start to goal (inclusive), written to path_x_out/path_y_out.
 */
int astar_grid(
    const uint8_t *blocked,
    int W, int H,
    int sx, int sy,
    int gx, int gy,
    int *path_x_out, int *path_y_out,
    int max_path_len,
    int max_cells,
    int *cells_explored_out,
    const uint8_t *cost_grid   /* nullable — proximity cost */
) {
    if (sx < 0 || sx >= W || sy < 0 || sy >= H) return 0;
    if (gx < 0 || gx >= W || gy < 0 || gy >= H) return 0;

    int N = W * H;

    // g-score array (flat), initialized to infinity
    std::vector<float> g_score(N, 1e30f);
    // closed set (flat bool array)
    std::vector<uint8_t> closed(N, 0);
    // came_from: encode parent as flat index, -1 = no parent
    std::vector<int> came_from(N, -1);

    auto idx = [W](int x, int y) { return y * W + x; };

    g_score[idx(sx, sy)] = 0.0f;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    open.push({heuristic(sx, sy, gx, gy), 0.0f, sx, sy});

    int explored = 0;
    bool found = false;

    // Precompute 1/252 for cost scaling
    constexpr float COST_SCALE = 1.0f / 252.0f;

    while (!open.empty() && explored < max_cells) {
        Node cur = open.top();
        open.pop();

        int ci = idx(cur.x, cur.y);
        if (closed[ci]) continue;
        closed[ci] = 1;
        explored++;

        if (cur.x == gx && cur.y == gy) {
            found = true;
            break;
        }

        for (int d = 0; d < 8; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            int ni = idx(nx, ny);
            if (blocked[ni] || closed[ni]) continue;

            float ng = cur.g + COST[d];
            // Add proximity cost: cost_grid value scaled to [0, 1.0]
            if (cost_grid) {
                ng += static_cast<float>(cost_grid[ni]) * COST_SCALE;
            }
            if (ng < g_score[ni]) {
                g_score[ni] = ng;
                came_from[ni] = ci;
                open.push({ng + heuristic(nx, ny, gx, gy), ng, nx, ny});
            }
        }
    }

    if (cells_explored_out) *cells_explored_out = explored;

    if (!found) return 0;

    // Reconstruct path (goal → start), then reverse
    std::vector<int> path_indices;
    int c = idx(gx, gy);
    while (c != -1) {
        path_indices.push_back(c);
        c = came_from[c];
    }

    int path_len = static_cast<int>(path_indices.size());
    if (path_len > max_path_len) path_len = max_path_len;

    // Reverse into output (start → goal)
    for (int i = 0; i < path_len; i++) {
        int pi = path_indices[path_indices.size() - 1 - i];
        path_x_out[i] = pi % W;
        path_y_out[i] = pi / W;
    }

    return path_len;
}

} // extern "C"
