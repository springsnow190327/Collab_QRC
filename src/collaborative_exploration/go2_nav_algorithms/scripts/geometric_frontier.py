#!/usr/bin/env python3
"""Compatibility alias for simple frontier explorer.

Deprecated: use `simple_frontier_explorer.py` in go2_nav_algorithms.
"""

import os
import runpy
from pathlib import Path

if os.environ.get("GO2_NAV_ALGOS_GEOMETRIC_FRONTIER_WARNED") != "1":
    print("[go2_nav_algorithms] DEPRECATED executable 'geometric_frontier.py'; use 'simple_frontier_explorer.py'.")
    os.environ["GO2_NAV_ALGOS_GEOMETRIC_FRONTIER_WARNED"] = "1"

TARGET = Path(__file__).resolve().parent / "simple_frontier_explorer.py"

if __name__ == "__main__":
    runpy.run_path(str(TARGET), run_name="__main__")
