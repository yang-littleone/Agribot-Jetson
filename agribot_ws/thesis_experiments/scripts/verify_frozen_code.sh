#!/usr/bin/env bash
set -euo pipefail

expected_commit="fe136784aae94e3fb8c53ce0d73d915cf7742495"
expected_tag="small-paper-complete-20260831"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "$script_dir/../.." && pwd)"
repository_root="$(git -C "$workspace_root" rev-parse --show-toplevel)"
source_path="agribot_ws/src"
approved_recording_changes=(
  ":(exclude)$source_path/centerline_extraction/scripts/record_field_bag.sh"
  ":(exclude)$source_path/centerline_extraction/scripts/validate_field_trial.py"
)
base_config="$workspace_root/src/centerline_extraction/config/field_row_follow.yaml"

tag_commit="$(git -C "$repository_root" rev-parse "${expected_tag}^{}")"
head_commit="$(git -C "$repository_root" rev-parse HEAD)"

if [[ "$tag_commit" != "$expected_commit" ]]; then
  echo "错误: 标签 $expected_tag 已偏离冻结提交。" >&2
  echo "预期: $expected_commit" >&2
  echo "实际: $tag_commit" >&2
  exit 1
fi

if ! git -C "$repository_root" diff --quiet "$expected_commit" -- \
    "$source_path" "${approved_recording_changes[@]}"; then
  echo "错误: 主体算法、配置或启动代码与小论文冻结版本不同。" >&2
  git -C "$repository_root" diff --stat "$expected_commit" -- \
    "$source_path" "${approved_recording_changes[@]}"
  exit 1
fi

if [[ ! -f "$base_config" ]]; then
  echo "错误: 找不到基础参数文件: $base_config" >&2
  exit 1
fi

echo "主体算法、配置和原启动代码冻结检查通过。"
echo "当前HEAD: $head_commit"
echo "冻结提交: $expected_commit"
echo "冻结标签: $expected_tag"
echo "参数文件: $base_config"
echo "参数SHA256: $(sha256sum "$base_config" | cut -d' ' -f1)"
echo "说明: 仅允许大论文录包脚本和自动验收脚本相对标签增强。"
echo "说明: build、install、log和未跟踪试验数据不参与主体算法判定。"
