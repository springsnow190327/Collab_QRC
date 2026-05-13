#!/usr/bin/env python3
"""
onboard_projector.py — Jetson-side binary multiplexer for stream_map_live.sh

Binary protocol (all little-endian):
    Frame = [type:uint8][len:uint32][payload]
    'C' 0x43  Current scan XY  [N:uint32][x0 y0 … float32]      Z-filtered, decimated
    'P' 0x50  Pose             [x y z yaw : 4×float32] = 16 bytes
    'M' 0x4D  Map increment    [N:uint32][x0 y0 z0 … float32]   new voxblox voxels only
    'T' 0x54  Path XY          [N:uint32][x0 y0 … float32]      gbplanner3 waypoints
"""

import sys, os, struct, math, threading, queue

sys.path = [p for p in sys.path if 'miniconda' not in p and 'conda' not in p]
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import rospy, numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory

# ── Defaults / arg parsing ────────────────────────────────────────────────────
CLOUD_TOPIC = '/robot/cloud_registered_body'
MAP_TOPIC   = '/gbplanner_node/surface_pointcloud'
ODOM_TOPIC  = '/robot/Odometry'
PATH_TOPIC  = '/pci_command_path'
DECIMATE    = 5
Z_MIN       = 0.10    # for current-scan left panel only
Z_MAX       = 1.80
ROS_MASTER  = 'http://192.168.123.18:11311'
ROS_IP      = '192.168.123.18'
MAP_VOX_RES = 0.15    # must match voxblox tsdf_voxel_size

for _arg in sys.argv[1:]:
    if '=' not in _arg:
        continue
    k, v = _arg.split('=', 1)
    if   k == 'cloud_topic': CLOUD_TOPIC = v
    elif k == 'map_topic':   MAP_TOPIC   = v
    elif k == 'odom_topic':  ODOM_TOPIC  = v
    elif k == 'path_topic':  PATH_TOPIC  = v
    elif k == 'decimate':    DECIMATE    = max(1, int(v))
    elif k == 'z_min':       Z_MIN       = float(v)
    elif k == 'z_max':       Z_MAX       = float(v)
    elif k == 'ros_master':  ROS_MASTER  = v
    elif k == 'ros_ip':      ROS_IP      = v

os.environ.setdefault('ROS_MASTER_URI', ROS_MASTER)
os.environ.setdefault('ROS_IP', ROS_IP)

print(f'[projector] cloud={CLOUD_TOPIC}  map={MAP_TOPIC}\n'
      f'[projector] odom={ODOM_TOPIC}  path={PATH_TOPIC}\n'
      f'[projector] decimate=1/{DECIMATE}  z=[{Z_MIN},{Z_MAX}]  vox={MAP_VOX_RES}m',
      file=sys.stderr, flush=True)

# ── Thread-safe writer ────────────────────────────────────────────────────────
_write_q: queue.Queue = queue.Queue(maxsize=64)

def _writer():
    out = sys.stdout.buffer
    while True:
        item = _write_q.get()
        if item is None:
            break
        tb, payload = item
        try:
            out.write(struct.pack('<BI', tb, len(payload)))
            out.write(payload)
            out.flush()
        except (BrokenPipeError, OSError):
            rospy.signal_shutdown('pipe closed')
            break

_wt = threading.Thread(target=_writer, daemon=True)
_wt.start()

def _enqueue(tb: int, payload: bytes):
    try:
        _write_q.put_nowait((tb, payload))
    except queue.Full:
        pass

# ── Current scan → 'C' (left panel) ─────────────────────────────────────────
_cloud_n = [0]

def _cloud_cb(msg: PointCloud2):
    pts = np.array(list(pc2.read_points(msg, field_names=('x','y','z'),
                                         skip_nans=True)), dtype=np.float32)
    if pts.size == 0:
        return
    pts = pts[(pts[:,2] >= Z_MIN) & (pts[:,2] <= Z_MAX)]
    if pts.size == 0:
        return
    if DECIMATE > 1:
        pts = pts[::DECIMATE]
    xy = np.ascontiguousarray(pts[:,:2], dtype=np.float32)
    _enqueue(ord('C'), struct.pack('<I', len(xy)) + xy.tobytes())
    _cloud_n[0] += 1
    if _cloud_n[0] % 50 == 0:
        print(f'[projector] {_cloud_n[0]} scan frames  last={len(xy)}pts',
              file=sys.stderr, flush=True)

# ── Voxblox surface → 'M' (3D map, incremental XYZ) ─────────────────────────
_sent_cells: set = set()   # (ix, iy, iz) tuples of already-transmitted voxels
_map_n = [0]

def _map_cb(msg: PointCloud2):
    pts = np.array(list(pc2.read_points(msg, field_names=('x','y','z'),
                                         skip_nans=True)), dtype=np.float32)
    if pts.size == 0:
        return
    # Voxelise — no Z filter; send full 3D surface
    ix = np.floor(pts[:,0] / MAP_VOX_RES).astype(np.int32)
    iy = np.floor(pts[:,1] / MAP_VOX_RES).astype(np.int32)
    iz = np.floor(pts[:,2] / MAP_VOX_RES).astype(np.int32)
    current = set(zip(ix.tolist(), iy.tolist(), iz.tolist()))
    new = current - _sent_cells
    if not new:
        return
    _sent_cells.update(new)
    half = MAP_VOX_RES * 0.5
    arr = np.array([(cx*MAP_VOX_RES+half, cy*MAP_VOX_RES+half, cz*MAP_VOX_RES+half)
                    for cx,cy,cz in new], dtype=np.float32)
    _enqueue(ord('M'), struct.pack('<I', len(arr)) + arr.tobytes())
    _map_n[0] += 1
    print(f'[projector] map +{len(new)} vox (total={len(_sent_cells)}, update={_map_n[0]})',
          file=sys.stderr, flush=True)

# ── Odometry → 'P' (x y z yaw, 16 bytes) ────────────────────────────────────
def _odom_cb(msg: Odometry):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    q = msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    _enqueue(ord('P'), struct.pack('<ffff', x, y, z, yaw))

# ── Path → 'T' ────────────────────────────────────────────────────────────────
_last_path_hash = [None]

def _path_cb(msg: MultiDOFJointTrajectory):
    pts = []
    for pt in msg.points:
        if pt.transforms:
            t = pt.transforms[0].translation
            pts.append((t.x, t.y))
    if not pts:
        return
    h = hash(tuple(pts))
    if h == _last_path_hash[0]:
        return
    _last_path_hash[0] = h
    xy = np.array(pts, dtype=np.float32)
    _enqueue(ord('T'), struct.pack('<I', len(xy)) + xy.tobytes())
    print(f'[projector] path: {len(xy)} waypoints', file=sys.stderr, flush=True)

# ── Main ──────────────────────────────────────────────────────────────────────
rospy.init_node('map_projector', anonymous=True, disable_signals=True)
rospy.Subscriber(CLOUD_TOPIC, PointCloud2, _cloud_cb, queue_size=1, buff_size=2**24)
rospy.Subscriber(MAP_TOPIC,   PointCloud2, _map_cb,   queue_size=1, buff_size=2**24)
rospy.Subscriber(ODOM_TOPIC,  Odometry,               _odom_cb, queue_size=1)
rospy.Subscriber(PATH_TOPIC,  MultiDOFJointTrajectory, _path_cb, queue_size=1)
print('[projector] subscribed', file=sys.stderr, flush=True)

try:
    rospy.spin()
except KeyboardInterrupt:
    pass
_write_q.put(None)
_wt.join(timeout=1.0)
