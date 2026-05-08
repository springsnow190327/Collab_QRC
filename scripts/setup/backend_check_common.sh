#!/usr/bin/env bash

backend_check_json_escape() {
  printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'
}

backend_check_bool_cmd() {
  "$@" >/dev/null 2>&1 && printf true || printf false
}

backend_check_join() {
  local IFS=', '
  printf '%s' "$*"
}

backend_check_resolve_strategy() {
  local requested_strategy="$1"
  local deployment_mode="$2"
  if [[ -n "${requested_strategy}" ]]; then
    printf '%s' "${requested_strategy}"
    return
  fi
  case "${deployment_mode}" in
    sim_ros2|sim_hybrid_ros1_slam_ros2_nav)
      printf docker_catkin
      ;;
    real_hybrid_ros1_slam_ros2_nav|real_ros1_only_experimental)
      printf host_noetic_catkin
      ;;
    *)
      printf unknown
      ;;
  esac
}

backend_check_emit() {
  local backend_key="$1"
  local backend_label="$2"
  local strategy="$3"
  local deployment_mode="$4"
  local source_dir="$5"
  local package_xml="$6"
  local launch_file="$7"
  local dependency_hints="$8"
  local workspace_path="$9"

  local compose_file="${ROS1_HYBRID_COMPOSE_FILE:-docker/ros1_hybrid_slam/docker-compose.yml}"
  local docker_image_candidates=()
  if [[ -n "${ROS1_HYBRID_SLAM_IMAGE:-}" ]]; then
    docker_image_candidates+=("${ROS1_HYBRID_SLAM_IMAGE}")
  fi
  docker_image_candidates+=(
    "collab_qrc-ros1_hybrid_slam"
    "ros1_hybrid_slam-ros1_hybrid_slam"
    "collab_qrc_ros1_hybrid_slam-ros1_hybrid_slam"
  )

  local available=false
  local buildable=false
  local runtime_ready=false
  local runtime_artifact_exists=false
  local package_xml_exists=false
  local launch_file_exists=false
  local ros1_noetic_available=false
  local catkin_make_available=false
  local catkin_tools_available=false
  local catkin_available=false
  local rospack_available=false
  local docker_cli_available=false
  local docker_daemon_available=false
  local docker_compose_available=false
  local compose_file_exists=false
  local docker_image_exists=false
  local docker_run_ready=false
  local detected_image=""
  local blocker=""
  local recommended_next_action=""

  [[ -d "${source_dir}" ]] && available=true
  [[ -f "${package_xml}" ]] && package_xml_exists=true
  [[ -f "${launch_file}" ]] && launch_file_exists=true
  [[ -f /opt/ros/noetic/setup.bash ]] && ros1_noetic_available=true
  command -v catkin_make >/dev/null 2>&1 && catkin_make_available=true
  command -v catkin >/dev/null 2>&1 && catkin_tools_available=true
  if [[ "${catkin_make_available}" == true || "${catkin_tools_available}" == true ]]; then
    catkin_available=true
  fi
  command -v rospack >/dev/null 2>&1 && rospack_available=true
  command -v docker >/dev/null 2>&1 && docker_cli_available=true
  [[ -f "${compose_file}" ]] && compose_file_exists=true
  if [[ "${docker_cli_available}" == true ]]; then
    docker info >/dev/null 2>&1 && docker_daemon_available=true || true
    if docker compose version >/dev/null 2>&1 || command -v docker-compose >/dev/null 2>&1; then
      docker_compose_available=true
    fi
    if [[ "${docker_daemon_available}" == true ]]; then
      local image
      for image in "${docker_image_candidates[@]}"; do
        if docker image inspect "${image}" >/dev/null 2>&1; then
          docker_image_exists=true
          detected_image="${image}"
          break
        fi
      done
      if [[ -n "${detected_image}" ]] && docker run --rm --entrypoint /bin/true "${detected_image}" >/dev/null 2>&1; then
        docker_run_ready=true
      fi
    fi
  fi

  if [[ -f "${workspace_path}/devel/setup.bash" || -f "${workspace_path}/install/setup.bash" ]]; then
    runtime_artifact_exists=true
  fi

  if [[ "${available}" != true ]]; then
    blocker="${backend_label} source not found at ${source_dir}; run scripts/setup/fetch_slam_backends.sh"
    recommended_next_action="fetch external backend sources without datasets or generated maps"
  elif [[ "${package_xml_exists}" != true ]]; then
    blocker="${backend_label} package.xml not found at ${package_xml}"
    recommended_next_action="verify external checkout layout or refresh external sources"
  elif [[ "${launch_file_exists}" != true ]]; then
    blocker="${backend_label} launch file not found at ${launch_file}"
    recommended_next_action="verify backend launch entrypoint for this repository revision"
  else
    case "${strategy}" in
      host_noetic_catkin)
        if [[ "${ros1_noetic_available}" != true ]]; then
          blocker="ros1_noetic_not_available"
          recommended_next_action="run on real robot or Jetson with ROS1 Noetic installed"
        elif [[ "${catkin_available}" != true ]]; then
          blocker="catkin_not_available"
          recommended_next_action="install catkin_make or catkin tools in the ROS1 Noetic environment"
        elif [[ "${rospack_available}" != true ]]; then
          blocker="rospack_not_available"
          recommended_next_action="install ros-noetic-rospack and source /opt/ros/noetic/setup.bash"
        else
          buildable=true
          if [[ "${runtime_artifact_exists}" != true ]]; then
            blocker="${backend_key}_host_catkin_runtime_artifact_not_found"
            recommended_next_action="build the ROS1 catkin workspace on the onboard host"
          else
            runtime_ready=true
          fi
        fi
        ;;
      docker_catkin)
        if [[ "${docker_cli_available}" != true ]]; then
          blocker="docker_cli_not_available"
          recommended_next_action="install Docker on the simulation host"
        elif [[ "${compose_file_exists}" != true ]]; then
          blocker="docker_compose_file_not_found:${compose_file}"
          recommended_next_action="restore docker/ros1_hybrid_slam/docker-compose.yml"
        elif [[ "${docker_compose_available}" != true ]]; then
          blocker="docker_compose_not_available"
          recommended_next_action="install docker compose plugin or docker-compose"
        elif [[ "${docker_daemon_available}" != true ]]; then
          blocker="docker_run_blocked_by_environment"
          recommended_next_action="run on host with Docker daemon permission or add user to docker group"
        else
          buildable=true
          if [[ "${docker_image_exists}" != true ]]; then
            blocker="${backend_key}_docker_image_not_found"
            recommended_next_action="run Docker build script on a host with Docker permission"
          elif [[ "${docker_run_ready}" != true ]]; then
            blocker="docker_run_blocked_by_environment"
            recommended_next_action="run on host where Docker containers are permitted"
          elif [[ "${runtime_artifact_exists}" != true ]]; then
            blocker="${backend_key}_docker_catkin_runtime_artifact_not_found"
            recommended_next_action="run scripts/manual backend Docker build/test command"
          else
            runtime_ready=true
          fi
        fi
        ;;
      *)
        blocker="unsupported_strategy_or_deployment_mode:${strategy}:${deployment_mode}"
        recommended_next_action="use --host, --docker, or a supported deployment_mode"
        ;;
    esac
  fi

  cat <<EOF
{
  "backend": "$(backend_check_json_escape "${backend_key}")",
  "backend_label": "$(backend_check_json_escape "${backend_label}")",
  "strategy": "$(backend_check_json_escape "${strategy}")",
  "deployment_mode": "$(backend_check_json_escape "${deployment_mode}")",
  "source_dir": "$(backend_check_json_escape "${source_dir}")",
  "workspace_path": "$(backend_check_json_escape "${workspace_path}")",
  "package_xml": "$(backend_check_json_escape "${package_xml}")",
  "package_xml_exists": ${package_xml_exists},
  "launch_file": "$(backend_check_json_escape "${launch_file}")",
  "launch_file_exists": ${launch_file_exists},
  "dependency_hints": "$(backend_check_json_escape "${dependency_hints}")",
  "available": ${available},
  "buildable": ${buildable},
  "runtime_ready": ${runtime_ready},
  "runtime_artifact_exists": ${runtime_artifact_exists},
  "ros1_noetic_available": ${ros1_noetic_available},
  "catkin_make_available": ${catkin_make_available},
  "catkin_tools_available": ${catkin_tools_available},
  "catkin_available": ${catkin_available},
  "rospack_available": ${rospack_available},
  "docker_cli_available": ${docker_cli_available},
  "docker_daemon_available": ${docker_daemon_available},
  "docker_compose_available": ${docker_compose_available},
  "docker_compose_file": "$(backend_check_json_escape "${compose_file}")",
  "docker_compose_file_exists": ${compose_file_exists},
  "docker_image_exists": ${docker_image_exists},
  "docker_image": "$(backend_check_json_escape "${detected_image}")",
  "docker_run_ready": ${docker_run_ready},
  "blocker": "$(backend_check_json_escape "${blocker}")",
  "recommended_next_action": "$(backend_check_json_escape "${recommended_next_action}")"
}
EOF
}
