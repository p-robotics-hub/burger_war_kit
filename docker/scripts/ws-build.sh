#!/bin/bash
set -e
set -u

# オプション・引数解析
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

MAKE_OPTION="$@"

# ワークスペースのビルド
#------------------------------------------------
cd ${WORKSPACE}
set -x
catkin_make ${MAKE_OPTION}
