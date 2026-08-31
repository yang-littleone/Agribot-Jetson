#!/usr/bin/env bash
# Replays exactly the 30 formal 20 m field trials used by the paper.
# The source USB bags are only read. Each .zstd is expanded into one temporary
# local directory and removed immediately after that run.
set -euo pipefail

workspace="${1:-/home/wheeltec/agribot/agribot_ws}"
usb_bag_root="${2:-/media/wheeltec/KINGSTON/agribot_field_data/field_trial_bags}"
output_root="${3:-$workspace/docs/performance_benchmark/jetson_30_formal_trials_20260731}"
manifest="$workspace/docs/performance_benchmark/formal_30_field_trial_bags_20260731.csv"
warmup_frames="${PERF_WARMUP_FRAMES:-30}"
ros_domain_id="${PERF_ROS_DOMAIN_ID:-47}"

if [[ ! -r "$manifest" || ! -d "$usb_bag_root" ]]; then
  echo "错误：找不到正式30包清单或U盘包目录。" >&2
  exit 2
fi
if [[ "$(tail -n +2 "$manifest" | wc -l)" -ne 30 ]]; then
  echo "错误：正式包清单必须正好有30次试验。" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$ros_domain_id"

mkdir -p "$output_root/raw_runs"
cp "$manifest" "$output_root/formal_30_field_trial_bags.csv"
cp "$workspace/src/centerline_extraction/config/field_row_follow.yaml" \
  "$output_root/benchmark_config_snapshot.yaml"
python3 "$workspace/docs/performance_benchmark/verify_formal_30_field_trial_bags.py" \
  --manifest "$manifest" --bag-root "$usb_bag_root" \
  --output "$output_root/formal_30_bag_integrity.csv"

{
  echo "generated_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "ros_domain_id=$ROS_DOMAIN_ID"
  uname -a
  nvpmodel -q 2>&1 || true
  jetson_clocks --show 2>&1 || true
  df -h "$workspace" "$usb_bag_root"
} > "$output_root/system_info.txt"

tegrastats_pid=""
detector_pid=""
collector_pid=""
temporary_root=""
cleanup_current() {
  for pid in "$detector_pid" "$collector_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  detector_pid=""; collector_pid=""
  if [[ -n "$temporary_root" && -d "$temporary_root" ]]; then
    rm -rf -- "$temporary_root"
  fi
  temporary_root=""
}
cleanup_all() {
  cleanup_current
  if [[ -n "$tegrastats_pid" ]] && kill -0 "$tegrastats_pid" 2>/dev/null; then
    kill -INT "$tegrastats_pid" 2>/dev/null || true
    wait "$tegrastats_pid" 2>/dev/null || true
  fi
}
trap cleanup_all EXIT INT TERM

echo "batch_started_at=$(date --iso-8601=seconds)" > "$output_root/batch_timing.txt"
tegrastats --interval 500 > "$output_root/tegrastats_full_batch.log" 2>&1 &
tegrastats_pid=$!
printf 'group_id,site,speed_mps,controller,trial_result_dir,bag_dir,status,started_at,finished_at,playback_exit\n' \
  > "$output_root/batch_status.csv"

run_index=0
while IFS=, read -r group_id site speed_mps controller trial_result_dir bag_dir; do
  [[ "$group_id" == "group_id" ]] && continue
  run_index=$((run_index + 1))
  run_dir="$output_root/raw_runs/${run_index}_${bag_dir}"
  if [[ -s "$run_dir/run_summary.json" ]]; then
    echo "[$run_index/30] 已完成，跳过：$bag_dir"
    printf '%s,%s,%s,%s,%s,%s,skipped_existing,,,\n' \
      "$group_id" "$site" "$speed_mps" "$controller" "$trial_result_dir" "$bag_dir" \
      >> "$output_root/batch_status.csv"
    continue
  fi
  mkdir -p "$run_dir"
  source_bag="$usb_bag_root/$bag_dir"
  source_db="$(find "$source_bag" -maxdepth 1 -type f \( -name '*.db3.zstd' -o -name '*.db3' \) -print -quit)"
  if [[ -z "$source_db" ]]; then
    echo "缺少数据库：$source_bag" >&2
    exit 1
  fi
  temporary_root="$(mktemp -d -t centerline_formal30_XXXXXX)"
  temporary_bag="$temporary_root/bag"
  mkdir -p "$temporary_bag"
  if [[ "$source_db" == *.zstd ]]; then
    zstd -d -q -c "$source_db" > "$temporary_bag/${bag_dir}_0.db3"
  else
    ln -s "$source_db" "$temporary_bag/$(basename "$source_db")"
  fi
  ros2 bag reindex "$temporary_bag" > "$run_dir/reindex.log" 2>&1

  # rosbag2 may retain its player process after printing "closing". Limit each
  # replay to its recorded duration plus a startup/drain allowance, rather than
  # allowing a stale player to consume the global 600 s emergency limit.
  duration_ns="$(awk '/^[[:space:]]*nanoseconds:/ {print $2; exit}' \
    "$source_bag/metadata.yaml")"
  if [[ ! "$duration_ns" =~ ^[0-9]+$ ]]; then
    echo "无法读取包时长：$source_bag/metadata.yaml" >&2
    exit 1
  fi
  playback_limit_s=$((duration_ns / 1000000000 + 20))

  start_at="$(date --iso-8601=seconds)"
  echo "[$run_index/30] 回放：$bag_dir（$site, ${speed_mps}m/s, $controller）"
  "$workspace/install/centerline_extraction/lib/centerline_extraction/corn_row_detector_projection" \
    --ros-args --params-file "$workspace/src/centerline_extraction/config/field_row_follow.yaml" \
    -p use_sim_time:=true -p enable_performance_metrics:=true \
    > "$run_dir/detector.log" 2>&1 &
  detector_pid=$!
  python3 "$workspace/src/centerline_extraction/scripts/benchmark_centerline_runtime.py" \
    --output-dir "$run_dir" --run-id "$bag_dir" --warmup-frames "$warmup_frames" \
    > "$run_dir/collector.log" 2>&1 &
  collector_pid=$!
  sleep 3
  set +e
  timeout --signal=INT --kill-after=8s "${playback_limit_s}s" ros2 bag play "$temporary_bag" --clock 100 \
    --rate 1.0 --disable-keyboard-controls --topics /livox/lidar /tf /tf_static \
    /odometry/filtered /navigation_mode > "$run_dir/rosbag_play.log" 2>&1
  playback_exit=$?
  set -e
  sleep 2
  cleanup_current
  finish_at="$(date --iso-8601=seconds)"
  if [[ -s "$run_dir/run_summary.json" ]]; then
    status="completed"
  else
    status="missing_summary"
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$group_id" "$site" "$speed_mps" "$controller" "$trial_result_dir" "$bag_dir" \
    "$status" "$start_at" "$finish_at" "$playback_exit" >> "$output_root/batch_status.csv"
  [[ "$status" == completed ]] || { echo "未生成统计结果：$bag_dir" >&2; exit 1; }
done < "$manifest"

echo "batch_finished_at=$(date --iso-8601=seconds)" >> "$output_root/batch_timing.txt"
python3 "$workspace/docs/performance_benchmark/analyze_jetson_formal_30_field_trials.py" \
  --input-root "$output_root/raw_runs" --manifest "$manifest" \
  --tegrastats-log "$output_root/tegrastats_full_batch.log" --output-dir "$output_root"
sha256sum "$manifest" "$output_root/formal_30_bag_integrity.csv" \
  "$workspace/src/centerline_extraction/src/corn_row_detector_projection.cpp" \
  "$workspace/src/centerline_extraction/include/centerline_extraction/corn_row_detector_projection.hpp" \
  "$workspace/src/centerline_extraction/scripts/benchmark_centerline_runtime.py" \
  "$workspace/src/centerline_extraction/config/field_row_follow.yaml" \
  > "$output_root/benchmark_manifest.sha256"
echo "完成：$output_root"
