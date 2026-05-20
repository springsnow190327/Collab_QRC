// Copyright 2026 Collab_QRC
// SPDX-License-Identifier: Apache-2.0
//
// All of the ParametersHandler implementation under ROS 1 is header-only
// (see include/nav_algo_core/mppi/tools/parameters_handler.hpp). This file
// exists only to keep CMake target globs stable; no out-of-line definitions
// are needed because dynamic-reconfigure plumbing has been removed and the
// remaining API is trivial template/inline code.

#include "nav_algo_core/compat.hpp"
#include "nav_algo_core/mppi/tools/parameters_handler.hpp"
