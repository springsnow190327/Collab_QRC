#!/usr/bin/env python3
"""Real-time web dashboard for the multi-agent VLM door-task stack.

Subscribes to /vlm_debug/state (std_msgs/String JSON) and serves a
single-page browser dashboard at http://127.0.0.1:8080 showing:

  - Latest 2x2 composite image the VLM is actually looking at
  - Planner panel: model, call count, reason, plan, world memory
  - Executer panel: model, call count, reason, actions, report
  - Perception panel: live /perception/world_dict entries with
    semantic_label, semantic_conf, hits, position, out-of-bounds flag
  - Recent executer→planner reports
  - Robot pose + button state

The HTML page auto-polls /state at 4 Hz for updates. Drive targets
outside the door-task scene bounds (x ∈ [0, 8], y ∈ [0, 4]) are
highlighted in red so hallucinated waypoints are visually obvious.

Run in a separate terminal while door_demo_mujoco.sh is running:

    source /opt/ros/humble/setup.bash
    source ~/Collab_QRC/install/setup.bash
    python3 ~/Collab_QRC/scripts/vlm_debug_web.py

Then open http://127.0.0.1:8080 in a browser.
"""

from __future__ import annotations

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class _ReuseAddrThreadingHTTPServer(ThreadingHTTPServer):
    """Port 8080 is often left in TIME_WAIT when the previous dashboard
    instance was killed with SIGKILL. Setting SO_REUSEADDR lets the
    replacement process bind immediately instead of hitting
    ``OSError: [Errno 98] Address already in use``."""
    allow_reuse_address = True

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:
    print("rclpy not found — source your ROS 2 setup first.", file=sys.stderr)
    raise


_HOST = "127.0.0.1"
_PORT = 8080


_INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>VLM multi-agent debug</title>
<style>
  :root {
    --bg: #0b0f14;
    --fg: #d7e0ea;
    --muted: #7a8796;
    --accent-p: #5cc4ff;
    --accent-e: #d98cff;
    --accent-s: #ffd166;
    --accent-r: #80ffa3;
    --panel: #131a23;
    --border: #2a3441;
  }
  * { box-sizing: border-box; }
  body { background: var(--bg); color: var(--fg); margin: 0; padding: 0; font-family: "JetBrains Mono", "Menlo", monospace; font-size: 12px; line-height: 1.35; }
  header { padding: 6px 12px; background: #08101a; border-bottom: 1px solid var(--border); display: flex; gap: 18px; align-items: baseline; }
  header h1 { margin: 0; font-size: 13px; letter-spacing: 1px; text-transform: uppercase; }
  header .status { color: var(--muted); }
  main { display: grid; grid-template-columns: 1.4fr 1fr 1fr; gap: 8px; padding: 8px; height: calc(100vh - 32px); }
  main .col { display: flex; flex-direction: column; gap: 8px; min-height: 0; }
  .panel { background: var(--panel); border: 1px solid var(--border); border-radius: 4px; padding: 8px 10px; overflow: auto; }
  .panel h2 { margin: 0 0 6px 0; font-size: 11px; letter-spacing: 1px; color: var(--muted); text-transform: uppercase; }
  .panel.planner h2 { color: var(--accent-p); }
  .panel.executer h2 { color: var(--accent-e); }
  .panel.state h2 { color: var(--accent-s); }
  .panel.reports h2 { color: var(--accent-r); }
  .panel.perception h2 { color: #80e0ff; }
  .kv { display: grid; grid-template-columns: 120px 1fr; column-gap: 10px; row-gap: 2px; }
  .kv .k { color: var(--muted); }
  .kv .v { white-space: pre-wrap; word-break: break-word; }
  .v.oob { color: #ff8080; font-weight: 600; }
  .reason { background: #0f1720; padding: 6px; border-radius: 3px; margin-top: 4px; color: #bcd2ea; white-space: pre-wrap; }
  .image-panel img { width: 100%; display: block; border-radius: 3px; }
  .image-panel { padding: 6px; }
  .dim { color: var(--muted); }
  .sep { height: 1px; background: var(--border); margin: 6px -10px; }
  .chip { display: inline-block; padding: 1px 6px; border-radius: 2px; background: #1b2632; color: var(--fg); margin-right: 4px; font-size: 10px; }
  .chip.ok { background: #163b29; color: #80ffa3; }
  .chip.err { background: #3b1d1d; color: #ff8c8c; }
  .chip.sem { background: #15283b; color: #80e0ff; }
  .wd-table { width: 100%; border-collapse: collapse; font-size: 11px; }
  .wd-table th { text-align: left; color: var(--muted); font-weight: 400; padding: 2px 4px; border-bottom: 1px solid var(--border); }
  .wd-table td { padding: 2px 4px; white-space: nowrap; }
  .wd-table tr.oob { background: #3b1d1d; color: #ffb0b0; }
  .wd-table tr.hot { background: #15283b; }
</style>
</head>
<body>
<header>
  <h1>VLM multi-agent debug</h1>
  <span class="status" id="status">waiting for /vlm_debug/state…</span>
</header>
<main>
  <div class="col">
    <div class="panel image-panel">
      <h2>What the VLM sees (2x2 composite)</h2>
      <img id="vlm-img" alt="vlm image" src="">
      <div id="img-caption" class="dim" style="margin-top:6px;"></div>
    </div>
    <div class="panel state">
      <h2>State</h2>
      <div class="kv" id="state-kv"></div>
    </div>
    <div class="panel perception">
      <h2>Detector views (yolo + clip overlay, per camera)</h2>
      <div id="det-status" class="dim">waiting for /perception/debug_image…</div>
      <div id="det-imgs" style="display:flex; gap:6px; margin-top:6px;"></div>
    </div>
    <div class="panel perception">
      <h2>Perception world_dict (yolo + clip + tracker)</h2>
      <div id="wd-status" class="dim">waiting for /perception/world_dict…</div>
      <table class="wd-table" id="wd-table">
        <thead>
          <tr>
            <th>id</th><th>world xy</th><th>color</th><th>semantic</th>
            <th>sem_conf</th><th>hits</th><th>conf</th><th>age</th>
          </tr>
        </thead>
        <tbody id="wd-body"></tbody>
      </table>
    </div>
    <div class="panel reports">
      <h2>Reports (exec → planner)</h2>
      <div id="reports"></div>
    </div>
  </div>

  <div class="panel planner">
    <h2>Planner (slow)</h2>
    <div class="kv" id="planner-kv"></div>
    <div class="sep"></div>
    <div class="dim">reason</div>
    <div class="reason" id="planner-reason"></div>
    <div class="sep"></div>
    <div class="dim">plan</div>
    <div class="kv" id="plan-kv"></div>
    <div class="sep"></div>
    <div class="dim">world_memory</div>
    <div class="kv" id="memory-kv"></div>
  </div>

  <div class="panel executer">
    <h2>Executer (fast)</h2>
    <div class="kv" id="executer-kv"></div>
    <div class="sep"></div>
    <div class="dim">reason</div>
    <div class="reason" id="executer-reason"></div>
    <div class="sep"></div>
    <div class="dim">actions</div>
    <div class="kv" id="action-kv"></div>
    <div class="sep"></div>
    <div class="dim">latest report</div>
    <div class="kv" id="report-kv"></div>
  </div>
</main>

<script>
function kvRow(tbl, k, v) {
  const d1 = document.createElement('div');
  d1.className = 'k';
  d1.textContent = k;
  const d2 = document.createElement('div');
  d2.className = 'v';
  d2.textContent = v;
  tbl.appendChild(d1);
  tbl.appendChild(d2);
}
function setKv(id, pairs) {
  const el = document.getElementById(id);
  el.innerHTML = '';
  for (const [k, v] of pairs) kvRow(el, k, v);
}
function trim(s, n) {
  if (s == null) return '—';
  s = String(s);
  if (s.length <= n) return s;
  return s.slice(0, n - 1) + '…';
}
// Door-task scene bounds — any world_xy outside this rectangle is
// either a projection error (assumed_depth overshoot) or a VLM
// hallucination. Rendered in red so the bug jumps out.
const SCENE_BOUNDS = { x_min: 0.0, x_max: 8.0, y_min: 0.0, y_max: 4.0 };
function isOOB(xy) {
  if (!xy || xy.length !== 2) return false;
  const [x, y] = xy;
  return x < SCENE_BOUNDS.x_min || x > SCENE_BOUNDS.x_max
      || y < SCENE_BOUNDS.y_min || y > SCENE_BOUNDS.y_max;
}
function parseDriveTarget(fmt) {
  // "drive(+7.3,+4.5)@0.40" or "rel(...)→(+7.0,+4.1)" or "stop"
  if (!fmt) return null;
  const m = fmt.match(/[\(→]\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*\)/);
  if (!m) return null;
  return [parseFloat(m[1]), parseFloat(m[2])];
}
async function poll() {
  try {
    const r = await fetch('/state');
    if (!r.ok) return;
    const s = await r.json();
    render(s);
    const wdN = (s.world_dict?.entries || []).length;
    document.getElementById('status').textContent =
      `t=${(s.t || 0).toFixed(1)}s  planner ${s.planner?.successes||0}/${s.planner?.calls||0}  exec ${s.executer?.successes||0}/${s.executer?.calls||0}  world_dict ${wdN}`;
  } catch (e) {}
}

function render(s) {
  if (s.image_b64) {
    document.getElementById('vlm-img').src = 'data:image/jpeg;base64,' + s.image_b64;
  }
  document.getElementById('img-caption').textContent =
    `updated ${(s.t || 0).toFixed(1)}s into episode`;

  const p = s.planner || {};
  const pl = p.last || {};
  setKv('planner-kv', [
    ['model', `${p.model || '?'}  period=${p.period_s || '?'}s`],
    ['calls', `${p.successes || 0} / ${p.calls || 0}`],
    ['sys prompt', `${p.system_prompt_chars || '?'} chars`],
  ]);
  document.getElementById('planner-reason').textContent = pl.reason || '—';
  const plan = s.plan || {};
  setKv('plan-kv', [
    ['A phase', plan.robot_a?.phase || '—'],
    ['A intent', trim(plan.robot_a?.intent_text, 200)],
    ['A target', JSON.stringify(plan.robot_a?.world_target_xy || '—')],
    ['B phase', plan.robot_b?.phase || '—'],
    ['B intent', trim(plan.robot_b?.intent_text, 200)],
    ['B target', JSON.stringify(plan.robot_b?.world_target_xy || '—')],
  ]);
  const mem = s.world_memory || {};
  const pillar = mem.pillar || {};
  const door = mem.door || {};
  setKv('memory-kv', [
    ['pillar.known', String(pillar.known ?? '—')],
    ['pillar.xy', JSON.stringify(pillar.world_xy || '—')],
    ['pillar.conf', pillar.confidence || '—'],
    ['pillar.evid', trim(pillar.evidence, 200)],
    ['door.known', String(door.known ?? '—')],
    ['door.xy', JSON.stringify(door.world_xy || '—')],
    ['notes', trim(mem.notes, 200)],
  ]);

  const e = s.executer || {};
  const el = e.last || {};
  setKv('executer-kv', [
    ['model', `${e.model || '?'}  period=${e.period_s || '?'}s`],
    ['calls', `${e.successes || 0} / ${e.calls || 0}`],
    ['sys prompt', `${e.system_prompt_chars || '?'} chars`],
  ]);
  document.getElementById('executer-reason').textContent = el.reason || '—';
  const actEl = document.getElementById('action-kv');
  actEl.innerHTML = '';
  for (const robot of ['robot_a', 'robot_b']) {
    const fmt = el[robot]?.fmt || '—';
    const xy = parseDriveTarget(fmt);
    const oob = isOOB(xy);
    const k = document.createElement('div'); k.className = 'k';
    k.textContent = robot === 'robot_a' ? 'A' : 'B';
    const v = document.createElement('div');
    v.className = 'v' + (oob ? ' oob' : '');
    v.textContent = (oob ? '⚠ OOB ' : '') + trim(fmt, 120);
    actEl.appendChild(k);
    actEl.appendChild(v);
  }
  const rep = el.report || {};
  const disc = rep.discoveries || [];
  setKv('report-kv', [
    ['uncertain', String(rep.uncertain ?? '—')],
    ['request_help', trim(rep.request_help, 200)],
    ['discoveries', disc.map(d => `[${d.robot}] ${d.what} @ ${d.where}`).join('\n') || '—'],
  ]);

  setKv('state-kv', [
    ['t', `${(s.t || 0).toFixed(1)}s`],
    ['A pose', s.pose?.robot_a
      ? `(${s.pose.robot_a.x.toFixed(2)}, ${s.pose.robot_a.y.toFixed(2)})  yaw=${s.pose.robot_a.yaw_deg.toFixed(0)}°`
      : '—'],
    ['B pose', s.pose?.robot_b
      ? `(${s.pose.robot_b.x.toFixed(2)}, ${s.pose.robot_b.y.toFixed(2)})  yaw=${s.pose.robot_b.yaw_deg.toFixed(0)}°`
      : '—'],
    ['button', `pressed=${s.button_pressed}  ever=${s.button_ever_pressed}`],
  ]);

  // ── Detector views panel ──
  const det = s.debug_image || {};
  const cams = det.cameras || {};
  const detStatus = document.getElementById('det-status');
  const detImgs = document.getElementById('det-imgs');
  const namespaces = Object.keys(cams);
  if (!namespaces.length) {
    detStatus.textContent = 'waiting for /perception/debug_image…';
    detImgs.innerHTML = '';
  } else {
    detStatus.textContent = `t=${(det.t || 0).toFixed(1)}s  cameras=${namespaces.join(', ')}`;
    detImgs.innerHTML = namespaces.map(ns => `
      <div style="flex:1; min-width:0;">
        <div class="dim" style="margin-bottom:2px;">${ns}</div>
        <img src="data:image/jpeg;base64,${cams[ns]}" style="width:100%; display:block; border-radius:3px;">
      </div>
    `).join('');
  }

  // ── Perception world_dict panel ──
  const wd = s.world_dict || {};
  const entries = wd.entries || [];
  const wdStatus = document.getElementById('wd-status');
  const wdBody = document.getElementById('wd-body');
  if (!entries.length) {
    wdStatus.textContent = wd.now != null
      ? `t=${wd.now.toFixed(1)}s decay=${wd.decay_sec}s  (no entries)`
      : 'waiting for /perception/world_dict…';
    wdBody.innerHTML = '';
  } else {
    wdStatus.textContent =
      `t=${(wd.now || 0).toFixed(1)}s decay=${wd.decay_sec}s  ${entries.length} entries`;
    wdBody.innerHTML = entries.map(e => {
      const oob = isOOB(e.world_xy);
      const semHot = (e.semantic_label && e.semantic_label.includes('red') && e.semantic_conf >= 0.5);
      const cls = oob ? 'oob' : (semHot ? 'hot' : '');
      const xy = e.world_xy ? `(${e.world_xy[0].toFixed(2)}, ${e.world_xy[1].toFixed(2)})` : '—';
      return `<tr class="${cls}">
        <td>${e.entry_id}</td>
        <td>${oob ? '⚠ ' : ''}${xy}</td>
        <td>${e.color_label || '—'}</td>
        <td>${e.semantic_label || '—'}</td>
        <td>${(e.semantic_conf ?? 0).toFixed(2)}</td>
        <td>${e.hits}</td>
        <td>${(e.confidence ?? 0).toFixed(2)}</td>
        <td>${(e.age_sec ?? 0).toFixed(1)}s</td>
      </tr>`;
    }).join('');
  }

  const reps = s.recent_reports || [];
  const repDiv = document.getElementById('reports');
  if (reps.length === 0) {
    repDiv.innerHTML = '<div class="dim">(none pending)</div>';
  } else {
    repDiv.innerHTML = reps.slice(-5).map(r => {
      const b = r.report || {};
      const d = (b.discoveries || []).map(x => `[${x.robot}] ${x.what}`).join('; ');
      return `<div>${(r.t || 0).toFixed(1)}s
        <span class="chip ${b.uncertain ? 'err' : 'ok'}">uncertain=${b.uncertain}</span>
        ${trim(b.request_help, 80)}
        <span class="dim">${d}</span>
      </div>`;
    }).join('');
  }
}

setInterval(poll, 250);
poll();
</script>
</body>
</html>
"""


_STATE_LOCK = threading.Lock()
_STATE: dict = {}
_WORLD_DICT: dict = {}
_DEBUG_IMAGE: dict = {}


class DebugSub(Node):
    def __init__(self) -> None:
        super().__init__("vlm_debug_web")
        self.create_subscription(
            String, "/vlm_debug/state", self._on_state, 10
        )
        self.create_subscription(
            String, "/perception/world_dict", self._on_world_dict, 10
        )
        # debug_image publisher uses BEST_EFFORT to avoid blocking the
        # detect loop on backpressure. Subscriber must match or DDS
        # silently drops every frame.
        dbg_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            String, "/perception/debug_image", self._on_debug_image, dbg_qos
        )
        self.get_logger().info(
            f"vlm_debug_web listening on /vlm_debug/state + /perception/world_dict"
            f" + /perception/debug_image  →  http://{_HOST}:{_PORT}"
        )

    def _on_state(self, msg: String) -> None:
        global _STATE
        try:
            obj = json.loads(msg.data)
        except Exception:
            return
        with _STATE_LOCK:
            _STATE = obj

    def _on_world_dict(self, msg: String) -> None:
        global _WORLD_DICT
        try:
            obj = json.loads(msg.data)
        except Exception:
            return
        with _STATE_LOCK:
            _WORLD_DICT = obj

    def _on_debug_image(self, msg: String) -> None:
        global _DEBUG_IMAGE
        try:
            obj = json.loads(msg.data)
        except Exception:
            return
        with _STATE_LOCK:
            _DEBUG_IMAGE = obj


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # silence noisy default logging
        return

    def do_GET(self) -> None:
        if self.path == "/" or self.path.startswith("/index"):
            self._send_bytes(_INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/state":
            with _STATE_LOCK:
                merged = dict(_STATE)
                if _WORLD_DICT:
                    merged["world_dict"] = _WORLD_DICT
                if _DEBUG_IMAGE:
                    merged["debug_image"] = _DEBUG_IMAGE
                payload = json.dumps(merged, default=str).encode("utf-8")
            self._send_bytes(payload, "application/json")
        else:
            self.send_response(404)
            self.end_headers()

    def _send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    rclpy.init()
    node = DebugSub()

    def spin() -> None:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit):
            pass

    th = threading.Thread(target=spin, daemon=True)
    th.start()

    server = _ReuseAddrThreadingHTTPServer((_HOST, _PORT), _Handler)
    print(f"[vlm_debug_web] serving  http://{_HOST}:{_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
