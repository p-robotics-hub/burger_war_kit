#!/bin/bash -eu
set -e
set -u

WORKSPACE=${HOME}/catkin_ws
BASE_KIT_REPOSITORY='https://github.com/p-robotics-hub/burger_war_kit.git'

# 既にワークスペースが存在する場合は、ユーザーに確認して削除
if [ -e "${WORKSPACE}/src/CMakeLists.txt" ]; then
  read -p "${WORKSPACE}は既に存在します。削除しますか？(yes/no)" yesno
  case "$yesno" in
    yes ) rm -vf "${WORKSPACE}/.catkin_workspace"
          rm -vf "${WORKSPACE}/src/CMakeLists.txt"
          rm -vrf "${WORKSPACE}/build"
          rm -vrf "${WORKSPACE}/devel"
          echo "古いワークスペースを削除しました"
          ;;
      * ) echo "ワークスペースの作成を中断しました。"
          exit 1
          ;;
  esac
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
source "${HOME}/.bashrc"

# ワークスペースの作成
[ -d ${WORKSPACE}/src ] || mkdir -p ${WORKSPACE}/src
cd ${WORKSPACE}/src
catkin_init_workspace
cd ${WORKSPACE}/
catkin_make

# burger_wat_kitが存在しない場合はcloneする
if [ ! -e "${WORKSPACE}/src/burger_war_kit" ]; then
  cd ${WORKSPACE}/src
  git clone ${BASE_KIT_REPOSITORY}
fi

# ビルド実施
cd ${WORKSPACE}
catkin_make

source ~/.bashrc
