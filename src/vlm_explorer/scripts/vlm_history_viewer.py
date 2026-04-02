#!/usr/bin/env python3
"""Lightweight live web viewer for VLM coordinator history logs.

Usage:
    python3 vlm_history_viewer.py                     # auto-detect latest run
    python3 vlm_history_viewer.py /path/to/vlm_history/20260401_143000
    python3 vlm_history_viewer.py --port 8502

Opens a browser-friendly page at http://localhost:8501 that auto-refreshes,
showing each VLM cycle: rendered map, prompt, response, latency, tool calls.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>VLM History Viewer</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace;
         background: #0d1117; color: #c9d1d9; padding: 16px; }
  h1 { color: #58a6ff; margin-bottom: 8px; font-size: 20px; }
  .meta { color: #8b949e; font-size: 12px; margin-bottom: 16px; }
  .cycle { border: 1px solid #30363d; border-radius: 8px; margin-bottom: 16px;
           background: #161b22; overflow: hidden; }
  .cycle-header { display: flex; align-items: center; gap: 12px; padding: 10px 14px;
                  background: #21262d; cursor: pointer; user-select: none; }
  .cycle-header:hover { background: #30363d; }
  .cycle-id { color: #58a6ff; font-weight: 600; font-size: 14px; }
  .badge { font-size: 11px; padding: 2px 8px; border-radius: 10px; font-weight: 500; }
  .badge-ok { background: #1b4332; color: #40c057; }
  .badge-err { background: #4c1d1d; color: #f87171; }
  .badge-empty { background: #1c1f26; color: #8b949e; }
  .badge-tool { background: #1a2744; color: #58a6ff; }
  .latency { color: #8b949e; font-size: 12px; margin-left: auto; }
  .cycle-body { display: none; padding: 14px; }
  .cycle-body.open { display: block; }
  .columns { display: flex; gap: 14px; }
  .col-img { flex: 0 0 auto; }
  .col-img img { max-width: 400px; border-radius: 6px; border: 1px solid #30363d; }
  .col-text { flex: 1; min-width: 0; }
  .section { margin-bottom: 12px; }
  .section-title { color: #58a6ff; font-size: 12px; font-weight: 600;
                   text-transform: uppercase; margin-bottom: 4px; }
  pre { background: #0d1117; border: 1px solid #30363d; border-radius: 6px;
        padding: 10px; font-size: 12px; line-height: 1.5; overflow-x: auto;
        white-space: pre-wrap; word-break: break-word; max-height: 300px; overflow-y: auto; }
  .response-raw { color: #7ee787; }
  .prompt-text { color: #d2a8ff; }
  .error-text { color: #f87171; }
  .no-data { color: #8b949e; text-align: center; padding: 60px; font-size: 16px; }
  .auto-tag { font-size: 11px; color: #8b949e; float: right; }
</style>
</head>
<body>
<h1>VLM History Viewer <span class="auto-tag" id="refresh-tag">auto-refresh 2s</span></h1>
<div class="meta" id="run-path"></div>
<div id="cycles"></div>
<script>
let lastLen = 0;
let openSet = new Set();

function toggle(id) {
  if (openSet.has(id)) openSet.delete(id); else openSet.add(id);
  render();
}

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

function renderCycle(entry, i) {
  const id = entry.cycle;
  const isOpen = openSet.has(id);
  const statusBadge = entry.error
    ? `<span class="badge badge-err">ERROR</span>`
    : entry.has_tool_calls
      ? `<span class="badge badge-tool">TOOL CALLS</span>`
      : `<span class="badge badge-empty">NO ACTION</span>`;
  return `
    <div class="cycle">
      <div class="cycle-header" onclick="toggle('${id}')">
        <span class="cycle-id">#${id}</span>
        <span class="badge badge-ok">${entry.model || '?'}</span>
        ${statusBadge}
        <span style="color:#8b949e;font-size:12px">${entry.timestamp || ''}</span>
        <span class="latency">${entry.latency_sec != null ? entry.latency_sec + 's' : ''}</span>
        <span style="color:#8b949e">${isOpen ? '▼' : '▶'}</span>
      </div>
      <div class="cycle-body ${isOpen ? 'open' : ''}" id="body-${id}"></div>
    </div>`;
}

let detailCache = {};
async function loadDetail(id) {
  if (detailCache[id]) return detailCache[id];
  try {
    const [pRes, rRes] = await Promise.all([
      fetch('/api/cycle/' + id + '/prompt.json'),
      fetch('/api/cycle/' + id + '/response.json'),
    ]);
    const prompt = await pRes.json();
    const response = await rRes.json();
    detailCache[id] = { prompt, response };
    return detailCache[id];
  } catch { return null; }
}

async function render() {
  for (const id of openSet) {
    const el = document.getElementById('body-' + id);
    if (!el || el.dataset.loaded) continue;
    el.dataset.loaded = '1';
    const d = await loadDetail(id);
    if (!d) { el.innerHTML = '<div class="no-data">Failed to load</div>'; continue; }
    const imgUrl = '/api/cycle/' + id + '/rendered_map.jpg';
    const sys = escHtml(d.prompt.system_prompt || '');
    const usr = escHtml(d.prompt.user_prompt || '');
    const raw = escHtml(d.response.raw || '');
    const err = d.response.error ? `<div class="section"><div class="section-title">Error</div><pre class="error-text">${escHtml(d.response.error)}</pre></div>` : '';
    const parsed = d.response.parsed ? escHtml(JSON.stringify(d.response.parsed, null, 2)) : '';
    el.innerHTML = `
      <div class="columns">
        <div class="col-img"><img src="${imgUrl}" onerror="this.style.display='none'"></div>
        <div class="col-text">
          ${err}
          <div class="section"><div class="section-title">Response (raw)</div><pre class="response-raw">${raw || '(empty)'}</pre></div>
          ${parsed ? `<div class="section"><div class="section-title">Parsed</div><pre>${parsed}</pre></div>` : ''}
          <div class="section"><div class="section-title">System Prompt</div><pre class="prompt-text">${sys}</pre></div>
          <div class="section"><div class="section-title">User Prompt</div><pre class="prompt-text">${usr}</pre></div>
        </div>
      </div>`;
  }
}

async function poll() {
  try {
    const res = await fetch('/api/index.json');
    const data = await res.json();
    document.getElementById('run-path').textContent = data.run_path || '';
    const entries = data.entries || [];
    if (entries.length !== lastLen) {
      lastLen = entries.length;
      // Auto-open latest
      if (entries.length > 0) openSet.add(entries[entries.length - 1].cycle);
      const container = document.getElementById('cycles');
      if (entries.length === 0) {
        container.innerHTML = '<div class="no-data">Waiting for VLM cycles...</div>';
      } else {
        container.innerHTML = entries.slice().reverse().map(renderCycle).join('');
      }
      render();
    }
  } catch {}
}

poll();
setInterval(poll, 2000);
</script>
</body>
</html>"""


class VLMHistoryHandler(SimpleHTTPRequestHandler):
    log_dir: Path = Path(".")

    def log_message(self, format, *args):
        pass  # quiet

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "":
            self._serve_html()
        elif path == "/api/index.json":
            self._serve_index()
        elif path.startswith("/api/cycle/"):
            self._serve_cycle_file(path)
        else:
            self.send_error(404)

    def _serve_html(self):
        data = HTML_TEMPLATE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_index(self):
        index_path = self.log_dir / "index.json"
        try:
            entries = json.loads(index_path.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            entries = []
        payload = json.dumps({
            "run_path": str(self.log_dir),
            "entries": entries,
        }).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _serve_cycle_file(self, path: str):
        # /api/cycle/0001/prompt.json
        parts = path.split("/")
        if len(parts) < 5:
            self.send_error(404)
            return
        cycle_id = parts[3]
        filename = parts[4]
        if ".." in cycle_id or ".." in filename:
            self.send_error(403)
            return
        filepath = self.log_dir / cycle_id / filename
        if not filepath.is_file():
            self.send_error(404)
            return
        data = filepath.read_bytes()
        content_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def find_latest_run(base_dir: Path) -> Path | None:
    if not base_dir.is_dir():
        return None
    runs = sorted(
        [d for d in base_dir.iterdir() if d.is_dir() and (d / "index.json").exists()],
        key=lambda d: d.name,
        reverse=True,
    )
    return runs[0] if runs else None


def main():
    parser = argparse.ArgumentParser(description="VLM history live viewer")
    parser.add_argument("log_dir", nargs="?", default="",
                        help="Path to a specific run directory (auto-detects latest if omitted)")
    parser.add_argument("--port", type=int, default=8501)
    args = parser.parse_args()

    import time as _time

    if args.log_dir:
        log_dir = Path(args.log_dir)
        # Wait for explicit dir to appear (e.g. launched before coordinator)
        for _ in range(120):
            if (log_dir / "index.json").exists():
                break
            _time.sleep(1)
    else:
        base = Path(os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log"))) / "vlm_history"
        log_dir = None
        print(f"Waiting for VLM history in {base} ...")
        for _ in range(120):
            log_dir = find_latest_run(base)
            if log_dir is not None:
                break
            _time.sleep(2)
        if log_dir is None:
            print("Timed out waiting for VLM history. Is the demo running?")
            sys.exit(1)

    if not (log_dir / "index.json").exists():
        print(f"No index.json in {log_dir}")
        sys.exit(1)

    VLMHistoryHandler.log_dir = log_dir
    server = HTTPServer(("0.0.0.0", args.port), VLMHistoryHandler)
    print(f"VLM History Viewer")
    print(f"  Log dir: {log_dir}")
    print(f"  Open:    http://localhost:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
