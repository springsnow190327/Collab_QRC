"""Backwards-compatible entry point.

The implementation lives in ``door_task.ros.controller``. This shim
exists so the launch file's ``executable="vlm_controller_node"`` and any
older imports keep working after the Phase 0 refactor.
"""

from door_task.ros.controller import VLMControllerNode, main

__all__ = ["VLMControllerNode", "main"]


if __name__ == "__main__":
    main()
