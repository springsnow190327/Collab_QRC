"""Load planner / executer system prompts from sibling markdown files.

Prompts are version-controlled as text and read once at import time so
they show up in `git diff` cleanly and can be edited without touching
Python.
"""

from __future__ import annotations

from pathlib import Path

_PROMPT_DIR = Path(__file__).resolve().parent


def _read(name: str) -> str:
    return (_PROMPT_DIR / name).read_text(encoding="utf-8")


PLANNER_PROMPT: str = _read("planner.md")
EXECUTER_PROMPT: str = _read("executer.md")
