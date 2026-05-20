#!/usr/bin/env python3
"""generate_cfpa2_ros1_yaml.py — flatten CFPA2 ROS 2 param yamls for ROS 1.

CFPA2's config yamls are written in the ROS 2 layout:

    /**:
      ros__parameters:
        robot_namespace: robot
        publish_rate: 2.0
        ...

ROS 1's `rosparam load <file> <ns>` cannot use that wrapper — it would load
literal keys `/**` → `ros__parameters` → params, so the node's private-ns
`getParam("publish_rate")` lookups (via the ros1 param_facade) would miss.

This script unwraps `/**: ros__parameters:` and emits a FLAT yaml that
`rosparam load` drops straight into the node's private namespace. Run it at
deploy time (or it's invoked inline by onboard_autonomy_noetic.sh as a
fallback). The ROS 2 build is unaffected — it keeps using the original
wrapped yamls.

Usage:
    generate_cfpa2_ros1_yaml.py <config_dir>
        # converts every cfpa2_*.yaml lacking a _ros1 suffix → <name>_ros1.yaml
    generate_cfpa2_ros1_yaml.py <src.yaml> <dst.yaml>
        # convert a single file
"""
import sys
import os
import glob

import yaml


def flatten(src: str, dst: str) -> int:
    with open(src) as f:
        data = yaml.safe_load(f)
    node = data
    # Unwrap a single top-level "/**" (or any "*"-ending) key + ros__parameters.
    if isinstance(data, dict) and len(data) == 1:
        only_key = next(iter(data))
        if only_key.endswith("**") or only_key == "/**":
            inner = data[only_key]
            node = inner.get("ros__parameters", inner) if isinstance(inner, dict) else inner
    with open(dst, "w") as f:
        yaml.safe_dump(node, f, default_flow_style=False, sort_keys=False)
    return len(node) if isinstance(node, dict) else 0


def main(argv) -> int:
    if len(argv) == 2 and os.path.isdir(argv[1]):
        cfg_dir = argv[1]
        srcs = [
            p for p in glob.glob(os.path.join(cfg_dir, "cfpa2_*.yaml"))
            if not p.endswith("_ros1.yaml")
        ]
        for src in srcs:
            base = os.path.splitext(os.path.basename(src))[0]
            dst = os.path.join(cfg_dir, f"{base}_ros1.yaml")
            n = flatten(src, dst)
            print(f"  {os.path.basename(src)} → {os.path.basename(dst)} ({n} keys)")
        return 0
    if len(argv) == 3:
        n = flatten(argv[1], argv[2])
        print(f"  {argv[1]} → {argv[2]} ({n} keys)")
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
