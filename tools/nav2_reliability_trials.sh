#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$ROOT_DIR/install/setup.bash"
  set -u
else
  echo "Missing $ROOT_DIR/install/setup.bash. Build and source first." >&2
  exit 1
fi

TRIALS="${TRIALS:-5}"
WORLD="${WORLD:-random_world.sdf}"
RENDER_ENGINE="${RENDER_ENGINE:-ogre2}"
RENDER_ENGINE_GUI="${RENDER_ENGINE_GUI:-ogre2}"
WARMUP_FORWARD_DISTANCE_M="${WARMUP_FORWARD_DISTANCE_M:-10.0}"
WARMUP_CMD_VEL_X="${WARMUP_CMD_VEL_X:-0.35}"
WARMUP_MIN_PROGRESS_M="${WARMUP_MIN_PROGRESS_M:-0.5}"
WARMUP_NO_PROGRESS_TIMEOUT_SEC="${WARMUP_NO_PROGRESS_TIMEOUT_SEC:-12}"
GOAL_TIMEOUT_SEC="${GOAL_TIMEOUT_SEC:-120}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-120}"
WARMUP_TIMEOUT_SEC="${WARMUP_TIMEOUT_SEC:-60}"
POST_WARMUP_SETTLE_SEC="${POST_WARMUP_SETTLE_SEC:-1}"
COLLISION_STATE_CAPTURE_SEC="${COLLISION_STATE_CAPTURE_SEC:-90}"
RVIZ="${RVIZ:-false}"

tmpdir="$(mktemp -d)"
diag_root="$tmpdir/nav2_reliability"
mkdir -p "$diag_root"

cleanup_global() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  echo "Diagnostics root: $diag_root"
}
trap cleanup_global EXIT

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_active_nav2() {
  local timeout_s="$1"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    local out
    out="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    if printf '%s' "$out" | grep -qi "active"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

lookup_tf_xyyaw() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_s="$3"
  python3 - <<'PY' "$target_frame" "$source_frame" "$timeout_s"
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

target = sys.argv[1]
source = sys.argv[2]
timeout_s = float(sys.argv[3])

rclpy.init()
node = rclpy.create_node('tf_lookup_once')
buf = Buffer()
listener = TransformListener(buf, node, spin_thread=False)
deadline = time.time() + timeout_s
ok = False

try:
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.2))
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            print(f"{tx} {ty} {yaw} {qx} {qy} {qz} {qw}")
            ok = True
            break
        except TransformException:
            continue
finally:
    node.destroy_node()
    rclpy.shutdown()

if not ok:
    raise SystemExit(1)
PY
}

read_pose_component() {
  local topic="$1"
  local outfile="$2"
  timeout 6s ros2 topic echo --once "$topic" >"$outfile" 2>&1 || true
}

read_map_metadata() {
  local outfile="$1"
  timeout 6s ros2 topic echo --once /map_metadata >"$outfile" 2>&1 || true
}

lookup_gz_world_topic() {
  gz topic -l | awk '/\/world\/.*\/dynamic_pose\/info/ {print $1; exit}'
}

get_maya_pose() {
  local world_topic="$1"
  gz topic -e -n 1 -t "$world_topic" | awk '
    $0 ~ /name: "maya"/ {in_block=1}
    in_block && $1 ~ /^position/ {pos=1}
    in_block && pos && $1=="x:" {x=$2}
    in_block && pos && $1=="y:" {y=$2}
    in_block && pos && $1=="z:" {z=$2; pos=0}
    in_block && $1 ~ /^orientation/ {ori=1}
    in_block && ori && $1=="x:" {qx=$2}
    in_block && ori && $1=="y:" {qy=$2}
    in_block && ori && $1=="z:" {qz=$2}
    in_block && ori && $1=="w:" {qw=$2; ori=0; in_block=0}
    END {printf("%s %s %s %s %s %s %s\n", x, y, z, qx, qy, qz, qw)}'
}

get_maya_pose_retry() {
  local world_topic="$1"
  local tries=6
  local out=""
  for _ in $(seq 1 "$tries"); do
    out="$(get_maya_pose "$world_topic" || true)"
    if [[ -n "$out" && "$out" != "      " ]]; then
      printf '%s\n' "$out"
      return 0
    fi
    sleep 0.3
  done
  return 1
}

compute_xy_distance() {
  local x0="$1"
  local y0="$2"
  local x1="$3"
  local y1="$4"
  python3 - <<'PY' "$x0" "$y0" "$x1" "$y1"
import math
import sys
x0 = float(sys.argv[1]); y0 = float(sys.argv[2]); x1 = float(sys.argv[3]); y1 = float(sys.argv[4])
print(math.hypot(x1 - x0, y1 - y0))
PY
}

start_sim() {
  local trial_dir="$1"
  local ros_log_dir="$trial_dir/roslog"
  mkdir -p "$ros_log_dir"
  export ROS_LOG_DIR="$ros_log_dir"

  local -a cmd=(ros2 launch maya_bringup maya.launch.xml "rviz:=${RVIZ}" "gz_args:=${WORLD} -s --render-engine ${RENDER_ENGINE} --render-engine-gui ${RENDER_ENGINE_GUI} -r")
  setsid "${cmd[@]}" >"$trial_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!
}

stop_sim() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -- "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
    unset LAUNCH_PID
  fi
}

capture_tf_pose() {
  local target="$1"
  local source="$2"
  local outfile="$3"
  if out="$(lookup_tf_xyyaw "$target" "$source" 20 2>/dev/null || true)"; then
    if [[ -n "${out:-}" ]]; then
      printf '%s\n' "$out" >"$outfile"
      return 0
    fi
  fi
  return 1
}

for trial in $(seq 1 "$TRIALS"); do
  trial_dir="$diag_root/trial_${trial}"
  mkdir -p "$trial_dir"
  echo "=== Trial $trial/$TRIALS ==="

  start_sim "$trial_dir"

  wait_for_topic /clock "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /clock" >&2; stop_sim; continue; }
  wait_for_topic /scan "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /scan" >&2; stop_sim; continue; }
  wait_for_topic /odom "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /odom" >&2; stop_sim; continue; }
  wait_for_topic /odometry/filtered "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /odometry/filtered" >&2; stop_sim; continue; }
  wait_for_topic /map "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: timed out waiting /map" >&2; stop_sim; continue; }
  wait_for_active_nav2 "$STARTUP_TIMEOUT_SEC" || { echo "Trial $trial: Nav2 bt_navigator not active" >&2; stop_sim; continue; }

  world_topic="$(lookup_gz_world_topic || true)"
  if [[ -z "$world_topic" ]]; then
    echo "Trial $trial: could not find Gazebo dynamic pose topic" >&2
    stop_sim
    continue
  fi
  printf '%s\n' "$world_topic" >"$trial_dir/gz_world_topic.txt"

  # Capture initial pose before warmup and then drive forward ~10m (manual workflow mimic).
  read_map_metadata "$trial_dir/map_metadata_before.txt"
  read_pose_component /clock "$trial_dir/clock_pre_warmup.txt"
  read_pose_component /odom "$trial_dir/odom_pre_warmup.txt"
  read_pose_component /odometry/filtered "$trial_dir/odom_filtered_pre_warmup.txt"
  capture_tf_pose odom base_footprint "$trial_dir/tf_odom_base_pre_warmup.txt" || true
  get_maya_pose_retry "$world_topic" >"$trial_dir/gz_pose_pre_warmup.txt" || true

  echo "Trial $trial: warming SLAM map + driving forward ${WARMUP_FORWARD_DISTANCE_M}m..."
  if [[ ! -s "$trial_dir/gz_pose_pre_warmup.txt" ]]; then
    echo "Trial $trial: missing initial Gazebo pose" >&2
    stop_sim
    continue
  fi
  read -r gzsx gzsy _ <"$trial_dir/gz_pose_pre_warmup.txt"

  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${WARMUP_CMD_VEL_X}}, angular: {z: 0.0}}" >/dev/null 2>&1 &
  WARMUP_PUB_PID=$!
  warmup_start_sec=$SECONDS
  warmup_last_progress_sec=$SECONDS
  warmup_last_progress_print_sec=$SECONDS
  warmup_best_dist="0.0"
  spawn_ref_captured=0
  : >"$trial_dir/warmup_progress.log"
  while :; do
    # Capture first available map pose as the return target (near spawn after SLAM initializes).
    if [[ "$spawn_ref_captured" -eq 0 ]]; then
      if capture_tf_pose map base_footprint "$trial_dir/tf_map_base_spawn_ref.txt"; then
        spawn_ref_captured=1
        echo "Trial $trial: captured map-frame return reference pose." | tee -a "$trial_dir/warmup_progress.log" >/dev/null
      fi
    fi

    if get_maya_pose_retry "$world_topic" >"$trial_dir/gz_pose_warmup_current.txt"; then
      read -r cgx cgy _ <"$trial_dir/gz_pose_warmup_current.txt"
      warmup_dist="$(compute_xy_distance "$gzsx" "$gzsy" "$cgx" "$cgy")"
      printf 'seconds=%s gz_distance_m=%s\n' "$((SECONDS - warmup_start_sec))" "$warmup_dist" >>"$trial_dir/warmup_progress.log"
      warmup_improved="$(python3 - <<'PY' "$warmup_dist" "$warmup_best_dist"
import sys
print("1" if float(sys.argv[1]) > float(sys.argv[2]) + 1e-3 else "0")
PY
)"
      if [[ "$warmup_improved" == "1" ]]; then
        warmup_best_dist="$warmup_dist"
        warmup_last_progress_sec=$SECONDS
      fi
      if (( SECONDS - warmup_last_progress_print_sec >= 2 )); then
        echo "Trial $trial: warmup progress ${warmup_dist}m / ${WARMUP_FORWARD_DISTANCE_M}m (best=${warmup_best_dist}m, t=$((SECONDS - warmup_start_sec))s)"
        warmup_last_progress_print_sec=$SECONDS
      fi
      warmup_reached="$(python3 - <<'PY' "$warmup_dist" "$WARMUP_FORWARD_DISTANCE_M"
import sys
print("1" if float(sys.argv[1]) >= float(sys.argv[2]) else "0")
PY
)"
      if [[ "$warmup_reached" == "1" ]]; then
        break
      fi
    fi

    if (( SECONDS - warmup_start_sec >= WARMUP_NO_PROGRESS_TIMEOUT_SEC )); then
      enough_progress="$(python3 - <<'PY' "$warmup_best_dist" "$WARMUP_MIN_PROGRESS_M"
import sys
print("1" if float(sys.argv[1]) >= float(sys.argv[2]) else "0")
PY
)"
      if [[ "$enough_progress" != "1" ]]; then
        echo "Trial $trial: warmup fail-fast (only ${warmup_best_dist}m progress after ${WARMUP_NO_PROGRESS_TIMEOUT_SEC}s)." | tee -a "$trial_dir/warmup_progress.log" >/dev/null
        break
      fi
    fi

    if (( SECONDS - warmup_start_sec >= WARMUP_TIMEOUT_SEC )); then
      echo "Trial $trial: warmup timed out before reaching target distance." | tee -a "$trial_dir/warmup_progress.log" >/dev/null
      break
    fi
    sleep 0.5
  done

  kill -INT "$WARMUP_PUB_PID" >/dev/null 2>&1 || true
  wait "$WARMUP_PUB_PID" >/dev/null 2>&1 || true
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  sleep "$POST_WARMUP_SETTLE_SEC"
  read_map_metadata "$trial_dir/map_metadata_after_warmup.txt"
  echo "Trial $trial: warmup done (best Gazebo progress=${warmup_best_dist}m)."

  # Start pose for the navigation trial is after warmup; goal is return-to-reference pose captured near spawn.
  read_pose_component /clock "$trial_dir/clock_start.txt"
  read_pose_component /odom "$trial_dir/odom_start.txt"
  read_pose_component /odometry/filtered "$trial_dir/odom_filtered_start.txt"
  capture_tf_pose map base_footprint "$trial_dir/tf_map_base_start.txt" || true
  capture_tf_pose odom base_footprint "$trial_dir/tf_odom_base_start.txt" || true
  get_maya_pose_retry "$world_topic" >"$trial_dir/gz_pose_start.txt" || true

  if [[ ! -s "$trial_dir/tf_map_base_spawn_ref.txt" ]]; then
    echo "Trial $trial: could not capture a map-frame return reference during warmup" >&2
    stop_sim
    continue
  fi
  read -r gx gy gyaw gqx gqy gqz gqw <"$trial_dir/tf_map_base_spawn_ref.txt"
  printf '%s\n' "{\"pose\": {\"header\": {\"frame_id\": \"map\"}, \"pose\": {\"position\": {\"x\": ${gx}, \"y\": ${gy}, \"z\": 0.0}, \"orientation\": {\"x\": ${gqx:-0.0}, \"y\": ${gqy:-0.0}, \"z\": ${gqz}, \"w\": ${gqw}}}}}" >"$trial_dir/goal.json"

  collision_type="$(ros2 topic type /collision_monitor_state 2>/dev/null || true)"
  printf '%s\n' "${collision_type:-}" >"$trial_dir/collision_monitor_state_type.txt"
  if [[ -n "${collision_type:-}" ]]; then
    timeout "${COLLISION_STATE_CAPTURE_SEC}s" ros2 topic echo /collision_monitor_state >"$trial_dir/collision_monitor_state.txt" 2>&1 &
    COLLISION_ECHO_PID=$!
  else
    COLLISION_ECHO_PID=""
  fi

  goal_yaml="{pose: {header: {frame_id: map}, pose: {position: {x: ${gx}, y: ${gy}, z: 0.0}, orientation: {x: ${gqx:-0.0}, y: ${gqy:-0.0}, z: ${gqz}, w: ${gqw}}}}}"
  echo "Trial $trial: sending NavigateToPose goal (return to recorded start pose)."
  if timeout "${GOAL_TIMEOUT_SEC}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$goal_yaml" >"$trial_dir/nav_goal.txt" 2>&1; then
    true
  else
    echo "Trial $trial: nav goal command timeout/failure (see nav_goal.txt)" >&2
    printf 'timeout_or_failure\n' >"$trial_dir/nav_goal_timeout.flag"
  fi

  if [[ -n "${COLLISION_ECHO_PID:-}" ]]; then
    wait "$COLLISION_ECHO_PID" >/dev/null 2>&1 || true
  fi

  read_pose_component /clock "$trial_dir/clock_end.txt"
  read_pose_component /odom "$trial_dir/odom_end.txt"
  read_pose_component /odometry/filtered "$trial_dir/odom_filtered_end.txt"
  capture_tf_pose map base_footprint "$trial_dir/tf_map_base_end.txt" || true
  capture_tf_pose odom base_footprint "$trial_dir/tf_odom_base_end.txt" || true
  get_maya_pose_retry "$world_topic" >"$trial_dir/gz_pose_end.txt" || true
  read_map_metadata "$trial_dir/map_metadata_end.txt"

  timeout 6s ros2 topic info /cmd_vel --verbose >"$trial_dir/cmd_vel_info.txt" 2>&1 || true
  timeout 6s ros2 topic info /collision_monitor_state --verbose >"$trial_dir/collision_monitor_state_info.txt" 2>&1 || true

  echo "Trial $trial: complete, shutting down simulation."
  stop_sim
done

python3 - <<'PY' "$diag_root" "$TRIALS" "$WARMUP_FORWARD_DISTANCE_M"
import json
import math
import re
import statistics as stats
import sys
from pathlib import Path

diag_root = Path(sys.argv[1])
trials_expected = int(sys.argv[2])
warmup_forward_distance_m = float(sys.argv[3])


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="ignore")


def parse_clock(path: Path):
    txt = read_text(path)
    ms = re.search(r"sec:\s*(\d+)", txt)
    mn = re.search(r"nanosec:\s*(\d+)", txt)
    if not ms or not mn:
        return None
    return int(ms.group(1)) + int(mn.group(1)) * 1e-9


def parse_header_stamp(path: Path):
    txt = read_text(path)
    m = re.search(r"header:\s*\n\s*stamp:\s*\n\s*sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)", txt)
    if not m:
        return None
    return int(m.group(1)) + int(m.group(2)) * 1e-9


def parse_tf_pose(path: Path):
    txt = read_text(path).strip()
    if not txt:
        return None
    parts = txt.split()
    if len(parts) < 3:
        return None
    vals = [float(p) for p in parts]
    out = {"x": vals[0], "y": vals[1], "yaw": vals[2]}
    if len(vals) >= 7:
        out["qx"], out["qy"], out["qz"], out["qw"] = vals[3:7]
    return out


def parse_gz_pose(path: Path):
    txt = read_text(path).strip()
    if not txt:
        return None
    parts = txt.split()
    if len(parts) < 7:
        return None
    x, y, z, qx, qy, qz, qw = [float(p) for p in parts[:7]]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return {"x": x, "y": y, "z": z, "yaw": yaw, "qx": qx, "qy": qy, "qz": qz, "qw": qw}


def parse_odom_xy(path: Path):
    txt = read_text(path)
    mx = re.search(r"position:\s*\n\s*x:\s*([\-0-9.eE]+)", txt)
    my = re.search(r"position:\s*\n\s*x:\s*[\-0-9.eE]+\s*\n\s*y:\s*([\-0-9.eE]+)", txt)
    if not mx or not my:
        return None
    return {"x": float(mx.group(1)), "y": float(my.group(1))}


def parse_goal_json(path: Path):
    txt = read_text(path)
    if not txt:
        return None
    try:
        data = json.loads(txt)
        pose = data["pose"]["pose"]
        return {
            "x": float(pose["position"]["x"]),
            "y": float(pose["position"]["y"]),
            "qz": float(pose["orientation"]["z"]),
            "qw": float(pose["orientation"]["w"]),
        }
    except Exception:
        return None


def parse_nav_goal_result(path: Path):
    txt = read_text(path)
    status_m = re.search(r"Goal finished with status:\s*([A-Z_]+)", txt)
    error_m = re.search(r"error_code:\s*([0-9]+)", txt)
    return {
        "status": status_m.group(1) if status_m else "UNKNOWN",
        "error_code": int(error_m.group(1)) if error_m else None,
        "raw_available": bool(txt.strip()),
    }


def parse_map_metadata(path: Path):
    txt = read_text(path)
    if not txt:
        return None
    w = re.search(r"^width:\s*(\d+)", txt, flags=re.M)
    h = re.search(r"^height:\s*(\d+)", txt, flags=re.M)
    res = re.search(r"^resolution:\s*([0-9.eE+-]+)", txt, flags=re.M)
    return {
        "width": int(w.group(1)) if w else None,
        "height": int(h.group(1)) if h else None,
        "resolution": float(res.group(1)) if res else None,
    }


def parse_collision_states(path: Path):
    txt = read_text(path)
    if not txt.strip():
        return {"message_count": 0, "unique_message_count": 0, "changed": None}
    blocks = [b.strip() for b in txt.split("---") if b.strip()]
    if not blocks:
        return {"message_count": 0, "unique_message_count": 0, "changed": None}
    uniq = len(set(blocks))
    return {"message_count": len(blocks), "unique_message_count": uniq, "changed": uniq > 1}


def dxy(a, b):
    return {"dx": b["x"] - a["x"], "dy": b["y"] - a["y"]}


def mag(v):
    return math.hypot(v["dx"], v["dy"])


def dist_xy(a, b):
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


report = {
    "trials_expected": trials_expected,
    "warmup_forward_distance_commanded_m": warmup_forward_distance_m,
    "trials": [],
    "aggregate": {},
}

for trial_dir in sorted(diag_root.glob("trial_*")):
    trial_name = trial_dir.name
    start_map = parse_tf_pose(trial_dir / "tf_map_base_start.txt")
    end_map = parse_tf_pose(trial_dir / "tf_map_base_end.txt")
    start_odom_tf = parse_tf_pose(trial_dir / "tf_odom_base_start.txt")
    end_odom_tf = parse_tf_pose(trial_dir / "tf_odom_base_end.txt")
    start_gz = parse_gz_pose(trial_dir / "gz_pose_start.txt")
    end_gz = parse_gz_pose(trial_dir / "gz_pose_end.txt")
    odom_start = parse_odom_xy(trial_dir / "odom_start.txt")
    odom_end = parse_odom_xy(trial_dir / "odom_end.txt")
    filt_start = parse_odom_xy(trial_dir / "odom_filtered_start.txt")
    filt_end = parse_odom_xy(trial_dir / "odom_filtered_end.txt")
    goal = parse_goal_json(trial_dir / "goal.json")
    nav = parse_nav_goal_result(trial_dir / "nav_goal.txt")
    if nav["status"] == "UNKNOWN" and (trial_dir / "nav_goal_timeout.flag").exists():
        nav["status"] = "TIMEOUT"
    collision = parse_collision_states(trial_dir / "collision_monitor_state.txt")
    map_meta_before = parse_map_metadata(trial_dir / "map_metadata_before.txt")
    map_meta_after = parse_map_metadata(trial_dir / "map_metadata_after_warmup.txt")
    map_meta_end = parse_map_metadata(trial_dir / "map_metadata_end.txt")

    clock_start = parse_clock(trial_dir / "clock_start.txt")
    clock_end = parse_clock(trial_dir / "clock_end.txt")
    odom_start_stamp = parse_header_stamp(trial_dir / "odom_start.txt")
    odom_end_stamp = parse_header_stamp(trial_dir / "odom_end.txt")
    filt_start_stamp = parse_header_stamp(trial_dir / "odom_filtered_start.txt")
    filt_end_stamp = parse_header_stamp(trial_dir / "odometry_filtered_end.txt") or parse_header_stamp(trial_dir / "odom_filtered_end.txt")

    trial = {
        "trial": trial_name,
        "nav_result": nav,
        "clock": {
            "start": clock_start,
            "end": clock_end,
            "duration_sec": (clock_end - clock_start) if clock_start is not None and clock_end is not None else None,
        },
        "timestamps": {
            "odom_start_header": odom_start_stamp,
            "odom_end_header": odom_end_stamp,
            "odom_filtered_start_header": filt_start_stamp,
            "odom_filtered_end_header": filt_end_stamp,
            "odom_start_age_vs_clock": (clock_start - odom_start_stamp) if None not in (clock_start, odom_start_stamp) else None,
            "odom_end_age_vs_clock": (clock_end - odom_end_stamp) if None not in (clock_end, odom_end_stamp) else None,
            "odom_filtered_start_age_vs_clock": (clock_start - filt_start_stamp) if None not in (clock_start, filt_start_stamp) else None,
            "odom_filtered_end_age_vs_clock": (clock_end - filt_end_stamp) if None not in (clock_end, filt_end_stamp) else None,
        },
        "poses": {
            "start_map_tf": start_map,
            "end_map_tf": end_map,
            "start_odom_tf": start_odom_tf,
            "end_odom_tf": end_odom_tf,
            "start_gazebo": start_gz,
            "end_gazebo": end_gz,
            "start_odom_msg": odom_start,
            "end_odom_msg": odom_end,
            "start_odom_filtered_msg": filt_start,
            "end_odom_filtered_msg": filt_end,
            "goal_map": goal,
        },
        "map_metadata": {
            "before": map_meta_before,
            "after_warmup": map_meta_after,
            "end": map_meta_end,
        },
        "collision_monitor_state": collision,
        "metrics": {},
    }

    # Movement metrics
    if start_map and end_map:
        mvec = dxy(start_map, end_map)
        trial["metrics"]["map_tf_delta"] = {**mvec, "distance_m": mag(mvec)}
    if start_odom_tf and end_odom_tf:
        ovec = dxy(start_odom_tf, end_odom_tf)
        trial["metrics"]["odom_tf_delta"] = {**ovec, "distance_m": mag(ovec)}
    if start_gz and end_gz:
        gvec = dxy(start_gz, end_gz)
        trial["metrics"]["gazebo_delta"] = {**gvec, "distance_m": mag(gvec)}
    if odom_start and odom_end:
        omsg = dxy(odom_start, odom_end)
        trial["metrics"]["odom_msg_delta"] = {**omsg, "distance_m": mag(omsg)}
    if filt_start and filt_end:
        fmsg = dxy(filt_start, filt_end)
        trial["metrics"]["odom_filtered_msg_delta"] = {**fmsg, "distance_m": mag(fmsg)}
    if goal and end_map:
        trial["metrics"]["goal_error_from_end_map_m"] = dist_xy({"x": goal["x"], "y": goal["y"]}, end_map)
        trial["metrics"]["goal_distance_from_start_map_m"] = dist_xy(start_map, {"x": goal["x"], "y": goal["y"]}) if start_map else None
    if goal and start_map:
        trial["metrics"]["goal_distance_from_start_map_m"] = dist_xy(start_map, {"x": goal["x"], "y": goal["y"]})

    # Cross-system delta consistency (Gazebo vs map TF displacement)
    g = trial["metrics"].get("gazebo_delta")
    m = trial["metrics"].get("map_tf_delta")
    if g and m:
        trial["metrics"]["gazebo_vs_map_displacement"] = {
            "distance_diff_m": abs(g["distance_m"] - m["distance_m"]),
            "distance_ratio": (m["distance_m"] / g["distance_m"]) if abs(g["distance_m"]) > 1e-9 else None,
            "note": "Map TF and Gazebo vectors are different frames; compare magnitudes, not dx/dy components.",
        }

    report["trials"].append(trial)


completed = [t for t in report["trials"] if t["nav_result"]["raw_available"]]
succ = [t for t in completed if t["nav_result"]["status"] == "SUCCEEDED"]
map_goal_errors = [t["metrics"].get("goal_error_from_end_map_m") for t in completed if t["metrics"].get("goal_error_from_end_map_m") is not None]
map_moves = [t["metrics"].get("map_tf_delta", {}).get("distance_m") for t in completed if t["metrics"].get("map_tf_delta", {}).get("distance_m") is not None]
gz_moves = [t["metrics"].get("gazebo_delta", {}).get("distance_m") for t in completed if t["metrics"].get("gazebo_delta", {}).get("distance_m") is not None]
disp_diffs = [t["metrics"].get("gazebo_vs_map_displacement", {}).get("distance_diff_m") for t in completed if t["metrics"].get("gazebo_vs_map_displacement", {}).get("distance_diff_m") is not None]
durations = [t["clock"]["duration_sec"] for t in completed if t["clock"]["duration_sec"] is not None]
collision_changes = [t["collision_monitor_state"]["changed"] for t in completed if t["collision_monitor_state"]["changed"] is not None]

report["aggregate"] = {
    "completed_trials": len(completed),
    "success_count": len(succ),
    "success_rate": (len(succ) / len(completed)) if completed else None,
    "aborted_count": sum(1 for t in completed if t["nav_result"]["status"] == "ABORTED"),
    "timeout_count": sum(1 for t in completed if t["nav_result"]["status"] == "TIMEOUT"),
    "statuses": [t["nav_result"]["status"] for t in completed],
    "error_codes": [t["nav_result"]["error_code"] for t in completed],
    "duration_sec_mean": stats.mean(durations) if durations else None,
    "duration_sec_median": stats.median(durations) if durations else None,
    "goal_error_end_map_mean_m": stats.mean(map_goal_errors) if map_goal_errors else None,
    "goal_error_end_map_median_m": stats.median(map_goal_errors) if map_goal_errors else None,
    "map_displacement_mean_m": stats.mean(map_moves) if map_moves else None,
    "gazebo_displacement_mean_m": stats.mean(gz_moves) if gz_moves else None,
    "gazebo_vs_map_disp_diff_mean_m": stats.mean(disp_diffs) if disp_diffs else None,
    "collision_monitor_state_changed_true_count": sum(1 for x in collision_changes if x is True),
    "collision_monitor_state_samples_available_count": len(collision_changes),
}

summary_path = diag_root / "summary.json"
summary_path.write_text(json.dumps(report, indent=2))

print("\n=== Nav2 Reliability Trials Summary ===")
agg = report["aggregate"]
print(f"Trials completed: {agg['completed_trials']} / {trials_expected}")
print(f"Successes: {agg['success_count']}  Aborts: {agg['aborted_count']}  Success rate: {agg['success_rate']}")
print(f"Mean duration (sim clock): {agg['duration_sec_mean']}")
print(f"Mean end-goal error (map TF): {agg['goal_error_end_map_mean_m']}")
print(f"Mean map displacement: {agg['map_displacement_mean_m']}")
print(f"Mean Gazebo displacement: {agg['gazebo_displacement_mean_m']}")
print(f"Mean |Gazebo-map displacement| diff: {agg['gazebo_vs_map_disp_diff_mean_m']}")
print(f"Collision monitor state changed (available samples): {agg['collision_monitor_state_changed_true_count']} / {agg['collision_monitor_state_samples_available_count']}")

print("\nPer-trial:")
for t in report["trials"]:
    status = t["nav_result"]["status"]
    err = t["nav_result"]["error_code"]
    dur = t["clock"]["duration_sec"]
    map_move = (t["metrics"].get("map_tf_delta") or {}).get("distance_m")
    gz_move = (t["metrics"].get("gazebo_delta") or {}).get("distance_m")
    goal_err = t["metrics"].get("goal_error_from_end_map_m")
    cm = t["collision_monitor_state"].get("changed")
    print(f"- {t['trial']}: status={status} error_code={err} dur={dur} map_move={map_move} gz_move={gz_move} goal_err={goal_err} collision_state_changed={cm}")

print(f"\nDetailed report: {summary_path}")
PY
