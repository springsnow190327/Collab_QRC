#!/usr/bin/env python3
"""
convert_bag_for_erasor.py — relabel `/livox/lidar` topic in a ROS 1 .bag
from `livox_ros_driver2/CustomMsg` to `livox_ros_driver/CustomMsg`.

Background:
  Our onboard recorder uses livox_ros_driver2 (the newer SDK2-based driver).
  Stock FAST_LIO_SLAM (HKU FAST-LIO + SC-A-LOAM) was written for the older
  livox_ros_driver. The two message types have IDENTICAL wire format — only
  the namespace differs — so we can rewrite the message type metadata in-place
  without touching the binary payload.

Usage:
  python3 convert_bag_for_erasor.py <input.bag> [<output.bag>]

  If <output.bag> is omitted, writes alongside input with `_v1` suffix.

  Multiple chunks can be merged at the same time:
    python3 convert_bag_for_erasor.py ops2_0_0.bag ops2_0_1.bag merged_v1.bag

This script uses the `rosbags` Python package (pip install rosbags) which
reads/writes ROS 1 bags without needing roscore.
"""
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

LIVOX2 = 'livox_ros_driver2/msg/CustomMsg'
LIVOX2_POINT = 'livox_ros_driver2/msg/CustomPoint'
LIVOX1 = 'livox_ros_driver/msg/CustomMsg'
LIVOX1_POINT = 'livox_ros_driver/msg/CustomPoint'

CUSTOM_MSG_DEF = """\
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver/CustomPoint[] points
"""
CUSTOM_POINT_DEF = """\
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""


def register_v1_types(ts):
    """Register livox_ros_driver/CustomMsg + CustomPoint in the typestore."""
    from rosbags.typesys import get_types_from_msg
    types = {}
    types.update(get_types_from_msg(CUSTOM_POINT_DEF, LIVOX1_POINT))
    types.update(get_types_from_msg(CUSTOM_MSG_DEF, LIVOX1))
    ts.register(types)


# md5sum for livox_ros_driver/CustomMsg + CustomPoint (computed via gendeps --md5).
# These are well-known constants from the livox_ros_driver v1 release.
LIVOX1_MSG_MD5 = "e4d6829bdfe657cb6c21a746c86b21a6"
LIVOX1_POINT_MD5 = "109a3cc548bb1f96626be89a5008bd6d"
# Concatenated msgdef text expected by ROS 1 bag readers for CustomMsg + Header + CustomPoint
LIVOX1_MSGDEF_BUNDLED = """\
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver/CustomPoint[] points
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: livox_ros_driver/CustomPoint
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""


def convert(inputs: list, output: Path):
    ts = get_typestore(Stores.ROS1_NOETIC)
    register_v1_types(ts)
    # rosbags uses normalized typenames internally with /msg/ separator
    LIVOX2_ROS1 = "livox_ros_driver2/msg/CustomMsg"
    LIVOX1_ROS1 = "livox_ros_driver/msg/CustomMsg"

    out_conns = {}
    with Writer(output) as writer:
        for src in inputs:
            print(f"  reading {src}")
            with Reader(src) as reader:
                for conn in reader.connections:
                    key = (conn.topic, conn.msgtype)
                    if key in out_conns:
                        continue
                    new_type = conn.msgtype
                    if new_type == LIVOX2_ROS1:
                        new_type = LIVOX1_ROS1
                        msgdef = LIVOX1_MSGDEF_BUNDLED
                        md5sum = LIVOX1_MSG_MD5
                    else:
                        msgdef = conn.msgdef.data
                        md5sum = conn.digest
                    out_conn = writer.add_connection(
                        topic=conn.topic,
                        msgtype=new_type,
                        msgdef=msgdef,
                        md5sum=md5sum,
                        callerid=getattr(conn.ext, 'callerid', None) or '',
                        latching=getattr(conn.ext, 'latching', 0) or 0,
                    )
                    out_conns[key] = out_conn

                count = {}
                for conn, ts_ns, raw in reader.messages():
                    writer.write(out_conns[(conn.topic, conn.msgtype)], ts_ns, raw)
                    count[conn.topic] = count.get(conn.topic, 0) + 1
            for topic, n in count.items():
                print(f"    {topic}: {n} msgs")
    print(f"\n✓ wrote {output}")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    args = [Path(a) for a in sys.argv[1:]]
    # If last arg looks like an output (does not exist as input bag), treat as output
    if len(args) >= 2 and not args[-1].exists():
        inputs, output = args[:-1], args[-1]
    else:
        if len(args) > 1:
            sys.exit("ERROR: multiple inputs require explicit output bag as last arg")
        inputs = args
        output = args[0].with_name(args[0].stem + "_v1.bag")

    for p in inputs:
        if not p.exists():
            sys.exit(f"ERROR: {p} not found")

    print(f"converting {len(inputs)} bag(s) → {output}")
    convert(inputs, output)


if __name__ == "__main__":
    main()
