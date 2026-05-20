// grid_offsets.hpp — shared neighbour offsets for 2D grid walks.
//
// 4-connectivity: cardinal directions (E, W, N, S).
// 8-connectivity: cardinal + diagonal.
// Order matches the original cfpa2_grid_ops.cpp so any unit-test that
// asserts on traversal order keeps passing across the modular refactor.

#pragma once

namespace cfpa2 {
namespace ops {

inline constexpr int DX4[4] = {1, -1, 0, 0};
inline constexpr int DY4[4] = {0, 0, 1, -1};

inline constexpr int DX8[8] = {1, -1, 0, 0, 1, -1, 1, -1};
inline constexpr int DY8[8] = {0, 0, 1, -1, 1, 1, -1, -1};

}  // namespace ops
}  // namespace cfpa2
