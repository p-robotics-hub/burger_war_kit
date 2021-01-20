#!/bin/bash
set -e
set -u

# ワークスペースディレクトリの設定
WORKSPACE=${HOME}/catkin_ws

# ワークスペースのビルド
cd ${WORKSPACE}
catkin_make
