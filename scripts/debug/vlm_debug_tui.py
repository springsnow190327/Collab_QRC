#!/usr/bin/env python3
"""Real-time TUI debugger for the multi-agent VLM door-task stack.

Subscribes to /vlm_debug/state (std_msgs/String holding a JSON blob)
and renders a live dashboard in the terminal using `rich`. Shows:

  - Planner panel: call count, model, last reason, plan, world memory
  - Executer panel: call count, model, last reason, action, report
  - State panel: robot poses, button state

Run in a separate terminal while door_demo_mujoco.sh is running:

    source /opt/ros/humble/setup.bash
    source ~/Collab_QRC/install/setup.bash
    python3 ~/Collab_QRC/scripts/vlm_debug_tui.py
"""

from __future__ import annotations

import json
import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError as exc:
    print("rclpy not found — source your ROS 2 setup first.", file=sys.stderr)
    raise

try:
    from rich.console import Console
    from rich.live import Live
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError:
    print(
        "rich not installed — run: pip install --user rich",
        file=sys.stderr,
    )
    sys.exit(1)


class StateBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: dict = {}
        self._last_seen: float = 0.0

    def update(self, state: dict) -> None:
        with self._lock:
            self._state = state
            self._last_seen = time.monotonic()

    def snapshot(self) -> tuple[dict, float]:
        with self._lock:
            return self._state, self._last_seen


class DebugSub(Node):
    def __init__(self, buf: StateBuffer) -> None:
        super().__init__("vlm_debug_tui")
        self._buf = buf
        self.create_subscription(
            String, "/vlm_debug/state", self._on_state, 10
        )

    def _on_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except Exception:
            return
        self._buf.update(state)


def _trim(s: str, n: int) -> str:
    if len(s) <= n:
        return s
    return s[: n - 1] + "…"


def _render_planner(state: dict) -> Panel:
    p = state.get("planner", {})
    last = p.get("last", {}) or {}
    plan = state.get("plan", {}) or {}
    mem = state.get("world_memory", {}) or {}

    t = Table.grid(padding=(0, 1))
    t.add_column(style="bold cyan", no_wrap=True)
    t.add_column()
    t.add_row(
        "model",
        f"{p.get('model', '?')}  period={p.get('period_s', '?')}s",
    )
    t.add_row(
        "calls",
        f"{p.get('successes', 0)} / {p.get('calls', 0)}",
    )
    t.add_row("sys chars", str(p.get("system_prompt_chars", "?")))
    t.add_row("reason", _trim(str(last.get("reason", "—")), 400))
    t.add_row("", "")
    t.add_row(
        "A phase",
        str(plan.get("robot_a", {}).get("phase", "—")),
    )
    t.add_row(
        "A intent",
        _trim(str(plan.get("robot_a", {}).get("intent_text", "—")), 160),
    )
    t.add_row(
        "A target",
        str(plan.get("robot_a", {}).get("world_target_xy", "—")),
    )
    t.add_row("", "")
    t.add_row(
        "B phase",
        str(plan.get("robot_b", {}).get("phase", "—")),
    )
    t.add_row(
        "B intent",
        _trim(str(plan.get("robot_b", {}).get("intent_text", "—")), 160),
    )
    t.add_row(
        "B target",
        str(plan.get("robot_b", {}).get("world_target_xy", "—")),
    )
    t.add_row("", "")
    pillar = mem.get("pillar", {}) if isinstance(mem, dict) else {}
    door = mem.get("door", {}) if isinstance(mem, dict) else {}
    t.add_row(
        "mem.pillar",
        f"known={pillar.get('known', '?')}  xy={pillar.get('world_xy', '—')}  "
        f"conf={pillar.get('confidence', '?')}",
    )
    t.add_row(
        "mem.pillar.ev",
        _trim(str(pillar.get("evidence", "—")), 160),
    )
    t.add_row(
        "mem.door",
        f"known={door.get('known', '?')}  xy={door.get('world_xy', '—')}",
    )
    t.add_row(
        "mem.notes",
        _trim(str(mem.get("notes", "—")), 200),
    )

    return Panel(t, title="[bold]PLANNER (slow)[/]", border_style="cyan")


def _render_executer(state: dict) -> Panel:
    e = state.get("executer", {})
    last = e.get("last", {}) or {}
    report = last.get("report", {}) or {}

    t = Table.grid(padding=(0, 1))
    t.add_column(style="bold magenta", no_wrap=True)
    t.add_column()
    t.add_row(
        "model",
        f"{e.get('model', '?')}  period={e.get('period_s', '?')}s",
    )
    t.add_row(
        "calls",
        f"{e.get('successes', 0)} / {e.get('calls', 0)}",
    )
    t.add_row("sys chars", str(e.get("system_prompt_chars", "?")))
    t.add_row("reason", _trim(str(last.get("reason", "—")), 400))
    t.add_row("", "")
    t.add_row("A action", _trim(str(last.get("robot_a", {}).get("fmt", "—")), 160))
    t.add_row("B action", _trim(str(last.get("robot_b", {}).get("fmt", "—")), 160))
    t.add_row("", "")
    t.add_row(
        "uncertain",
        str(report.get("uncertain", "—")),
    )
    t.add_row(
        "request_help",
        _trim(str(report.get("request_help", "—")), 200),
    )
    discoveries = report.get("discoveries", []) or []
    if discoveries:
        lines = []
        for d in discoveries[:5]:
            if not isinstance(d, dict):
                continue
            lines.append(
                f"[{d.get('robot', '?')}] {d.get('what', '?')} @ {d.get('where', '?')}"
            )
        t.add_row("discoveries", "\n".join(lines) if lines else "—")
    else:
        t.add_row("discoveries", "—")

    return Panel(t, title="[bold]EXECUTER (fast)[/]", border_style="magenta")


def _render_state(state: dict, last_seen: float) -> Panel:
    pose = state.get("pose", {}) or {}
    t = Table.grid(padding=(0, 2))
    t.add_column(style="bold yellow", no_wrap=True)
    t.add_column()
    t.add_row("t", f"{state.get('t', 0.0):.1f}s")
    age = time.monotonic() - last_seen if last_seen else float("inf")
    t.add_row(
        "staleness",
        f"{age:.1f}s" if age != float("inf") else "no data yet",
    )
    for ns, pv in pose.items():
        t.add_row(
            ns,
            f"({pv['x']:+.2f}, {pv['y']:+.2f})  yaw={pv['yaw_deg']:+.0f}°",
        )
    t.add_row(
        "button",
        (
            f"pressed={state.get('button_pressed', '?')}  "
            f"ever={state.get('button_ever_pressed', '?')}"
        ),
    )
    return Panel(t, title="[bold]STATE[/]", border_style="yellow")


def _render_reports(state: dict) -> Panel:
    reports = state.get("recent_reports", []) or []
    if not reports:
        body = Text("(none pending — reports flushed to planner each tick)", style="dim")
    else:
        lines = []
        for r in reports[-5:]:
            t = r.get("t", 0.0)
            body = r.get("report", {})
            unc = body.get("uncertain", False)
            disc = body.get("discoveries", []) or []
            disc_txt = "; ".join(
                f"[{d.get('robot', '?')}] {d.get('what', '?')}" for d in disc[:3]
                if isinstance(d, dict)
            )
            lines.append(
                f"{t:6.1f}s  uncertain={unc}  help={_trim(str(body.get('request_help', '')), 60)}  "
                f"disc=[{disc_txt}]"
            )
        body = "\n".join(lines)
    return Panel(body, title="[bold]REPORTS (exec→planner)[/]", border_style="green")


def build_layout(state: dict, last_seen: float) -> Layout:
    root = Layout()
    root.split_column(
        Layout(name="top", ratio=4),
        Layout(name="mid", size=10),
        Layout(name="bot", size=7),
    )
    root["top"].split_row(
        Layout(_render_planner(state), name="planner"),
        Layout(_render_executer(state), name="executer"),
    )
    root["mid"].update(_render_reports(state))
    root["bot"].update(_render_state(state, last_seen))
    return root


def main() -> None:
    rclpy.init()
    buf = StateBuffer()
    node = DebugSub(buf)

    def spin() -> None:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit):
            pass

    t = threading.Thread(target=spin, daemon=True)
    t.start()

    console = Console()
    try:
        with Live(build_layout({}, 0.0), console=console, refresh_per_second=4) as live:
            while rclpy.ok():
                state, last_seen = buf.snapshot()
                live.update(build_layout(state, last_seen))
                time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
