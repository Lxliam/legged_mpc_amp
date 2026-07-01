#!/usr/bin/env sh
workspace_dir="$(cd "$(dirname "$0")/../../.." && pwd)"
tmp_dir="$workspace_dir/tmp/legged_control"
mkdir -p "$tmp_dir"
rosrun xacro xacro "$1" robot_type:="$2" > "$tmp_dir/$2.urdf"
