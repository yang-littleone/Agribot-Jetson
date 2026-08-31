#!/usr/bin/env bash
set -euo pipefail

workspace="${1:-/home/wheeltec/agribot/agribot_ws}"
input_db="${2:-$workspace/field_ground_truth/pointcloud_annotation_cache/20260729_115928_p01_normal_qaware_v018_r1/20260729_115928_p01_normal_qaware_v018_r1_0.db3}"
output_root="${3:-$workspace/docs/performance_benchmark/jetson_20260731}"
repeat_count="${PERF_REPEAT_COUNT:-3}"
segment_wall_s="${PERF_SEGMENT_SECONDS:-65}"
warmup_frames="${PERF_WARMUP_FRAMES:-30}"

if [[ ! -f "$input_db" ]]; then
  echo "错误: 找不到真实点云数据库: $input_db" >&2
  exit 1
fi
if [[ ! "$repeat_count" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误: PERF_REPEAT_COUNT必须为正整数。" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u

mkdir -p "$output_root/raw_runs"
cp "$workspace/src/centerline_extraction/config/field_row_follow.yaml" \
  "$output_root/benchmark_config_snapshot.yaml"
temporary_root="$(mktemp -d -t centerline_perf_XXXXXX)"
temporary_bag="$temporary_root/real_field_bag"
mkdir -p "$temporary_bag"
ln -s "$input_db" "$temporary_bag/$(basename "$input_db")"
ros2 bag reindex "$temporary_bag"

detector_pid=""
collector_pid=""
cleanup() {
  if [[ -n "$detector_pid" ]] && kill -0 "$detector_pid" 2>/dev/null; then
    kill -INT "$detector_pid" 2>/dev/null || true
  fi
  if [[ -n "$collector_pid" ]] && kill -0 "$collector_pid" 2>/dev/null; then
    kill -INT "$collector_pid" 2>/dev/null || true
  fi
  rm -rf -- "$temporary_root"
}
trap cleanup EXIT INT TERM

{
  echo "generated_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "architecture=$(uname -m)"
  uname -a
  lsb_release -a 2>&1 || true
  echo "ROS_DISTRO=${ROS_DISTRO:-}"
  python3 --version
  g++ --version | head -n 1
  echo "logical_cpu_count=$(nproc)"
  free -h
  echo "nvpmodel:"
  nvpmodel -q 2>&1 || true
  echo "jetson_clocks:"
  jetson_clocks --show 2>&1 || true
  echo "tegrastats snapshot:"
  timeout 2s tegrastats --interval 1000 2>&1 || true
  echo "detector compile flags:"
  sed -n '1,30p' "$workspace/build/centerline_extraction/CMakeFiles/corn_row_detector_projection.dir/flags.make" 2>/dev/null || true
  echo "cmake build type:"
  rg '^CMAKE_BUILD_TYPE:' "$workspace/build/centerline_extraction/CMakeCache.txt" 2>/dev/null || true
} > "$output_root/system_info.txt"

export ROS_DOMAIN_ID="${PERF_ROS_DOMAIN_ID:-47}"

for run_number in $(seq 1 "$repeat_count"); do
  run_id="run_${run_number}"
  run_dir="$output_root/raw_runs/$run_id"
  mkdir -p "$run_dir"
  echo "开始 $run_id/$repeat_count"

  "$workspace/install/centerline_extraction/lib/centerline_extraction/corn_row_detector_projection" \
    --ros-args \
    --params-file "$workspace/src/centerline_extraction/config/field_row_follow.yaml" \
    -p use_sim_time:=true \
    -p enable_performance_metrics:=true \
    > "$run_dir/detector.log" 2>&1 &
  detector_pid=$!

  python3 "$workspace/src/centerline_extraction/scripts/benchmark_centerline_runtime.py" \
    --output-dir "$run_dir" \
    --run-id "$run_id" \
    --warmup-frames "$warmup_frames" \
    > "$run_dir/collector.log" 2>&1 &
  collector_pid=$!

  sleep 3
  set +e
  timeout --signal=INT --kill-after=8s "${segment_wall_s}s" \
    ros2 bag play "$temporary_bag" \
      --clock 100 \
      --rate 1.0 \
      --disable-keyboard-controls \
      --topics \
        /livox/lidar \
        /tf \
        /tf_static \
        /odometry/filtered \
        /navigation_mode \
    > "$run_dir/rosbag_play.log" 2>&1
  play_status=$?
  set -e
  echo "rosbag_play_exit=$play_status" > "$run_dir/playback_status.txt"
  sleep 2

  if kill -0 "$detector_pid" 2>/dev/null; then
    kill -INT "$detector_pid"
    wait "$detector_pid" || true
  fi
  detector_pid=""
  sleep 1
  if kill -0 "$collector_pid" 2>/dev/null; then
    kill -INT "$collector_pid"
    wait "$collector_pid" || true
  fi
  collector_pid=""

  if [[ ! -s "$run_dir/run_summary.json" ]]; then
    echo "错误: $run_id没有生成run_summary.json" >&2
    exit 1
  fi
  echo "完成 $run_id"
  sleep 2
done

python3 "$workspace/docs/performance_benchmark/analyze_jetson_performance.py" \
  --input-root "$output_root/raw_runs" \
  --output-dir "$output_root" \
  --bag-label "20260729_115928_p01_normal_qaware_v018_r1" \
  --segment-duration-s "$segment_wall_s"

sha256sum \
  "$workspace/src/centerline_extraction/src/corn_row_detector_projection.cpp" \
  "$workspace/src/centerline_extraction/include/centerline_extraction/corn_row_detector_projection.hpp" \
  "$workspace/src/centerline_extraction/config/field_row_follow.yaml" \
  "$output_root/benchmark_config_snapshot.yaml" \
  "$workspace/src/centerline_extraction/scripts/benchmark_centerline_runtime.py" \
  "$workspace/docs/performance_benchmark/analyze_jetson_performance.py" \
  "$input_db" \
  > "$output_root/benchmark_manifest.sha256"

echo "测试完成: $output_root"
