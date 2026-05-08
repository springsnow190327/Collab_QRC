from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AdapterContract:
    mode: str
    namespace: str
    robot_topics: tuple[str, ...]
    team_topics: tuple[str, ...]
    production_downstream_depends_on_swarm: bool


def _ns(namespace: str) -> str:
    clean = namespace.strip().strip("/")
    if not clean:
        raise ValueError("namespace must not be empty")
    return clean


def adapter_contract_for_mode(mode: str, *, namespace: str) -> AdapterContract:
    ns = _ns(namespace)
    normalized = mode.strip().lower()
    if normalized == "swarm_lio2_shadow":
        return AdapterContract(
            mode=normalized,
            namespace=ns,
            robot_topics=(
                f"/{ns}/swarm_lio2/Odometry",
                f"/{ns}/swarm_lio2/cloud_static",
                f"/{ns}/swarm_lio2/cloud_map",
                f"/{ns}/swarm_lio2/mutual_state",
                f"/{ns}/swarm_lio2/relative_transform",
            ),
            team_topics=("/team_slam/swarm_lio2_metrics",),
            production_downstream_depends_on_swarm=False,
        )
    if normalized == "swarm_lio2_primary":
        return AdapterContract(
            mode=normalized,
            namespace=ns,
            robot_topics=(
                f"/{ns}/Odometry",
                f"/{ns}/corrected_odom",
                f"/{ns}/odom/nav",
                f"/{ns}/cloud_registered_body",
                f"/{ns}/cloud_static",
                f"/{ns}/cloud_dynamic",
            ),
            team_topics=(
                "/team_slam/swarm_lio2_relative_transform",
                "/team_slam/swarm_lio2_metrics",
                "/tf",
            ),
            production_downstream_depends_on_swarm=True,
        )
    raise ValueError(
        "slam_backend must be 'swarm_lio2_shadow' or 'swarm_lio2_primary', "
        f"got '{mode}'"
    )
