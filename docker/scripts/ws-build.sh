#!/bin/bash -eu
set -e
set -u

source /opt/ros/${ROS_DISTRO}/setup.bash
source "${HOME}/.bashrc"

# ワークスペースディレクトリの設定
WORKSPACE=${HOME}/catkin_ws

# ワークスペースのビルド
cd ${WORKSPACE}
catkin_make
