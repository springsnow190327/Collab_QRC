"""Canonical dual-Go2W modular Gazebo launch."""

import importlib.util
import os

_DUAL_GO2_PATH = os.path.join(os.path.dirname(__file__), "dual_go2_modular.launch.py")
_DUAL_GO2_SPEC = importlib.util.spec_from_file_location("dual_go2_modular_launch", _DUAL_GO2_PATH)
if _DUAL_GO2_SPEC is None or _DUAL_GO2_SPEC.loader is None:
    raise RuntimeError(f"Unable to load shared launch helper from {_DUAL_GO2_PATH}")

_DUAL_GO2_MODULE = importlib.util.module_from_spec(_DUAL_GO2_SPEC)
_DUAL_GO2_SPEC.loader.exec_module(_DUAL_GO2_MODULE)

generate_fixed_variant_launch_description = _DUAL_GO2_MODULE.generate_fixed_variant_launch_description


def generate_launch_description():
    return generate_fixed_variant_launch_description(
        launch_name="dual_go2w_modular",
        robot_variant="go2w",
        default_nav_profile="default_nav_dual_go2w.yaml",
    )
