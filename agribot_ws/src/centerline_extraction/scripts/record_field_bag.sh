#!/usr/bin/env bash
set -u
set -o pipefail

usage() {
  echo "用法: ros2 run centerline_extraction record_field_bag.sh TRIAL_ID"
  echo "示例: ros2 run centerline_extraction record_field_bag.sh normal_qaware_v018_r1"
  echo
  echo "U盘可写且剩余空间充足时，优先保存到U盘的 agribot_field_data/field_trial_bags/。"
  echo "未插U盘、空间不足或U盘录制异常时，保存/续录到工作空间的 field_trial_bags/。"
  echo "按 Ctrl+C 正常结束录制、写入rosbag元数据并自动验收bag、CSV和参数快照。"
}

if [[ $# -ne 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  if [[ $# -eq 1 ]]; then
    exit 0
  fi
  exit 2
fi

trial_id="$1"
if [[ ! "$trial_id" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "错误: TRIAL_ID 只能包含字母、数字、下划线和连字符。" >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误: 找不到 ros2，请先 source 工作空间的 install/setup.bash。" >&2
  exit 1
fi

find_workspace_root() {
  local package_prefix=""
  local prefix_parent=""
  local candidate=""

  package_prefix="$(ros2 pkg prefix centerline_extraction 2>/dev/null || true)"
  if [[ -n "$package_prefix" ]]; then
    prefix_parent="$(dirname "$package_prefix")"
    if [[ "$(basename "$prefix_parent")" == "install" ]]; then
      candidate="$(dirname "$prefix_parent")"
    elif [[ "$(basename "$package_prefix")" == "install" ]]; then
      candidate="$(dirname "$package_prefix")"
    fi
    if [[ -n "$candidate" &&
          -f "$candidate/src/centerline_extraction/package.xml" ]]; then
      echo "$candidate"
      return 0
    fi
  fi

  candidate="$PWD"
  while [[ "$candidate" != "/" ]]; do
    if [[ -f "$candidate/src/centerline_extraction/package.xml" ]]; then
      echo "$candidate"
      return 0
    fi
    candidate="$(dirname "$candidate")"
  done
  return 1
}

workspace_root="$(find_workspace_root || true)"
if [[ -z "$workspace_root" ]]; then
  echo "错误: 无法定位 agribot_ws，请在工作空间内执行并先 source install/setup.bash。" >&2
  exit 1
fi

workspace_bag_root="$workspace_root/field_trial_bags"
minimum_free_gib="${FIELD_STORAGE_MIN_FREE_GIB:-1}"
if [[ ! "$minimum_free_gib" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "警告: FIELD_STORAGE_MIN_FREE_GIB=$minimum_free_gib 无效，使用1 GiB。"
  minimum_free_gib="1"
fi
minimum_free_kib="$(
  awk -v gib="$minimum_free_gib" \
    'BEGIN { printf "%.0f", gib * 1024 * 1024 }'
)"

available_kib_for() {
  df -Pk "$1" 2>/dev/null | awk 'NR == 2 {print $4}'
}

is_mounted_writable() {
  [[ -d "$1" ]] && mountpoint -q "$1" && [[ -w "$1" ]]
}

find_usb_mount() {
  local configured_mount="${FIELD_USB_MOUNT:-}"
  local media_root="/media/${USER:-wheeltec}"
  local candidate=""
  local candidate_kib=""
  local best_mount=""
  local best_kib=0

  case "${FIELD_STORAGE_FORCE_WORKSPACE:-}" in
    1|true|TRUE|yes|YES)
      return 1
      ;;
  esac

  if [[ -n "$configured_mount" ]]; then
    if is_mounted_writable "$configured_mount"; then
      candidate_kib="$(available_kib_for "$configured_mount")"
      if [[ "$candidate_kib" =~ ^[0-9]+$ ]] &&
         (( candidate_kib >= minimum_free_kib )); then
        echo "$configured_mount"
        return 0
      fi
    fi
    return 1
  fi

  if [[ ! -d "$media_root" ]]; then
    return 1
  fi
  while IFS= read -r -d '' candidate; do
    if ! is_mounted_writable "$candidate"; then
      continue
    fi
    candidate_kib="$(available_kib_for "$candidate")"
    if [[ "$candidate_kib" =~ ^[0-9]+$ ]] &&
       (( candidate_kib >= minimum_free_kib )) &&
       (( candidate_kib > best_kib )); then
      best_mount="$candidate"
      best_kib="$candidate_kib"
    fi
  done < <(find "$media_root" -mindepth 1 -maxdepth 1 -type d -print0)

  if [[ -n "$best_mount" ]]; then
    echo "$best_mount"
    return 0
  fi
  return 1
}

usb_mount="$(find_usb_mount || true)"
storage_backend="workspace"
bag_root="$workspace_bag_root"
if [[ -n "$usb_mount" ]]; then
  storage_backend="usb"
  bag_root="$usb_mount/agribot_field_data/field_trial_bags"
fi

if ! mkdir -p "$bag_root"; then
  if [[ "$storage_backend" != "usb" ]]; then
    echo "错误: 无法创建工作空间记录目录: $bag_root" >&2
    exit 1
  fi
  echo "警告: 无法创建U盘记录目录，改用工作空间。"
  storage_backend="workspace"
  bag_root="$workspace_bag_root"
  usb_mount=""
  if ! mkdir -p "$bag_root"; then
    echo "错误: 无法创建工作空间记录目录: $bag_root" >&2
    exit 1
  fi
fi

available_kib="$(available_kib_for "$bag_root")"
if [[ "$available_kib" =~ ^[0-9]+$ ]]; then
  available_gib=$((available_kib / 1024 / 1024))
  if (( available_gib < 1 )); then
    echo "错误: 当前记录磁盘仅剩不足1 GiB，无法安全录制原始点云。" >&2
    exit 1
  elif (( available_kib < minimum_free_kib )); then
    echo "警告: 工作空间所在磁盘仅剩约 ${available_gib} GiB，请控制试验时长。"
  fi
fi

stamp="$(date '+%Y%m%d_%H%M%S')"
bag_dir="$bag_root/${stamp}_${trial_id}"
if [[ -e "$bag_dir" ]]; then
  echo "错误: 输出目录已经存在: $bag_dir" >&2
  exit 1
fi

required_topics=(
  /livox/lidar
  /odometry/filtered
)
existing_topics="$(ros2 topic list 2>/dev/null || true)"
for topic in "${required_topics[@]}"; do
  if ! grep -Fqx -- "$topic" <<<"$existing_topics"; then
    echo "警告: 当前尚未发现 $topic；请确认基础系统已启动。"
  fi
done

topics=(
  /livox/lidar
  /livox/imu
  /imu/data_h30
  /imu/selected
  /wheel/odom
  /wheel/odom_validated
  /Odometry
  /lio/odom
  /odometry/fused_internal
  /odometry/filtered
  /tf
  /tf_static
  /corn_row_center_line
  /corn_row_center_line_viz
  /under_canopy_left_boundary
  /under_canopy_right_boundary
  /corridor_width
  /corridor_safety_margin
  /corridor_error_budget
  /corridor_confidence
  /centerline_detection_diagnostics
  /cmd_vel
  /control_quality_factor
  /controller_lateral_error
  /controller_heading_error
  /headland_detected
  /headland_turn_path
  /reacquire_reference_path
  /navigation_safety_state
  /navigation_mode
  /indoor_test_stage
  /indoor_test_finished
  /parameter_events
  /rosout
)

echo "试验编号: $trial_id"
echo "存储位置: $storage_backend"
echo "rosbag目录: $bag_dir"
echo "正在等待并记录已存在及稍后出现的话题；按 Ctrl+C 结束。"

record_bag() {
  local output_dir="$1"
  ros2 bag record \
    --output "$output_dir" \
    --max-bag-size 2147483648 \
    --compression-mode file \
    --compression-format zstd \
    --compression-threads 1 \
    --compression-queue-size 2 \
    "${topics[@]}"
}

interrupted_by_user=0
handle_interrupt() {
  interrupted_by_user=1
}
trap handle_interrupt INT

write_manifest() {
  local output_dir="$1"
  local backend="$2"
  local status="$3"
  local continued_from="${4:-}"
  if [[ ! -d "$output_dir" ]]; then
    return
  fi
  {
    echo "trial_id=$trial_id"
    echo "workspace_root=$workspace_root"
    echo "storage_backend=$backend"
    echo "usb_mount=${usb_mount:-}"
    echo "bag_dir=$output_dir"
    echo "continued_from=$continued_from"
    echo "finished_at=$(date --iso-8601=seconds)"
    echo "record_exit_status=$status"
    echo "topics=${topics[*]}"
  } > "$output_dir/field_recording_manifest.txt" 2>/dev/null || {
    echo "警告: 无法在 $output_dir 写入录制清单。" >&2
  }
}

record_bag "$bag_dir"
record_status=$?
write_manifest "$bag_dir" "$storage_backend" "$record_status"

final_bag_dir="$bag_dir"
if [[ "$storage_backend" == "usb" &&
      "$record_status" -ne 0 &&
      "$record_status" -ne 130 &&
      "$record_status" -ne 143 ]]; then
  echo "警告: U盘录制异常退出（状态码 $record_status），立即改到工作空间续录。"
  mkdir -p "$workspace_bag_root"
  continuation_stamp="$(date '+%Y%m%d_%H%M%S')"
  continuation_dir="$workspace_bag_root/${continuation_stamp}_${trial_id}_continued_after_usb"
  if [[ -e "$continuation_dir" ]]; then
    continuation_dir="${continuation_dir}_$$"
  fi
  echo "续录目录: $continuation_dir"
  record_bag "$continuation_dir"
  continuation_status=$?
  write_manifest \
    "$continuation_dir" "workspace_fallback" \
    "$continuation_status" "$bag_dir"
  final_bag_dir="$continuation_dir"
  record_status="$continuation_status"
fi

if [[ -d "$final_bag_dir" ]]; then
  echo "录制结束: $final_bag_dir"
  trap - INT
  validator="$workspace_root/src/centerline_extraction/scripts/validate_field_trial.py"
  validation_status=1
  if [[ -f "$validator" ]]; then
    validation_args=(
      --bag "$final_bag_dir"
      --trial-id "$trial_id"
      --workspace-root "$workspace_root"
    )
    if [[ -n "$usb_mount" ]]; then
      validation_args+=(--usb-mount "$usb_mount")
    fi
    python3 "$validator" "${validation_args[@]}"
    validation_status=$?
    {
      echo "validation_exit_status=$validation_status"
      echo "validation_report=$final_bag_dir/field_validation_report.json"
    } >> "$final_bag_dir/field_recording_manifest.txt" 2>/dev/null || true
  else
    echo "警告: 找不到自动验收程序: $validator" >&2
    echo "人工检查命令: ros2 bag info \"$final_bag_dir\""
  fi
fi

if [[ "${validation_status:-1}" -ne 0 ]]; then
  exit "$validation_status"
fi
if [[ "$interrupted_by_user" -eq 1 &&
      ( "$record_status" -eq 130 || "$record_status" -eq 143 ) ]]; then
  exit 0
fi
exit "$record_status"
