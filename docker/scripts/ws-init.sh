#!/bin/bash
set -e
set -u

# ワークスペースディレクトリの設定
#------------------------------------------------
WORKSPACE=$HOME/catkin_ws
while getopts w: OPT
do
  case $OPT in
    w  ) # ワークスペースディレクトリの設定"
      WORKSPACE=${OPTARG}
      ;;
    \? ) # 不正オプション時
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))

# ワークスペースの初期化
#------------------------------------------------
cd ${WORKSPACE}/src
catkin_init_workspace

# ビルド実施
#------------------------------------------------
cd ${WORKSPACE}/
catkin_make

source ~/.bashrc
