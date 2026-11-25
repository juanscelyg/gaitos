#!/bin/bash
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
# A MuJoCo clip of gaitos for the documentation (snapshot frames -> make_gif.py).
#
#   record_mujoco.sh <robot> <out.gif> <t0> <t1> "<view options>" [--at t "cmd"]...
#   e.g. record_mujoco.sh go2 docs/media/go2_mujoco_walk.gif 0 12 "--distance 1.6" \
#          --at 0.5 "mode stand_up" --at 3 "vel 0.5 0 0"
# MODEL=<variant> adds gaitos_mujoco params/<robot>_<variant>_mujoco.yaml (MODEL=dex3: the R1
# with Dex3-1 hands). Needs the workspace sourced. ROS_DOMAIN_ID: one of its own (the tool is a ROS node).
set -e
robot=$1; out=$2; t0=$3; t1=$4; view=$5; shift 5
here=$(cd "$(dirname "$0")" && pwd)
tmp=$(mktemp -d)
share() { ros2 pkg prefix "$1" >/dev/null && echo "$(ros2 pkg prefix "$1")/share/$1"; }
# shellcheck disable=SC2086
ros2 run gaitos_mujoco snapshot "$@" --width 960 --height 540 $view --frames "$t0" "$t1" 10 "$tmp/f" \
  --ros-args --params-file "$(share gaitos)/params/gaitos.yaml" \
  --params-file "$(share gaitos_description)/robots/${robot}_description/config/${robot}.yaml" \
  --params-file "$(share gaitos_mujoco)/params/${robot}_mujoco.yaml" \
  ${MODEL:+--params-file "$(share gaitos_mujoco)/params/${robot}_${MODEL}_mujoco.yaml"} -p real_time_factor:=100.0 \
  | grep -v "^saved" || true
python3 "$here/make_gif.py" "$out" "$tmp"/f_*.png --fps 10 ${GIF_OPTS:-}
rm -rf "$tmp"
