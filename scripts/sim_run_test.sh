#!/bin/bash
###############################################################################
#-burger-warのシミュレーションを実施する
#-
#+[USAGE]
#+  $0 [-h] シミュレーション起動タイムアウト時間
#+
#-[OPTIONS]
#-  -h            このヘルプを表示
#-
###############################################################################
set -e
set -u
CMD_NAME=$(basename $0)
SCRIPT_DIR=$(cd "$(dirname $0)"; pwd)
SIM_JUDGE_PID=
SIM_START_PID=

# 設定
#------------------------------------------------
# ログ出力先ファイル
SIM_JUDGE_LOG="${HOME}/sim_with_judge_nogui.log"
SIM_START_LOG="${HOME}/start_test.log"

# 中断処理
#------------------------------------------------
cancel_exit() {
  echo "TEST CANCEL..."
  [ -z "#{SIM_JUDGE_PID}" ] || kill -TERM ${SIM_JUDGE_PID}
  [ -z "#{SIM_START_PID}" ] || kill -TERM ${SIM_START_PID}
  exit 1
}
trap cancel_exit 1 2 3 4 9 15

# 関数定義
#------------------------------------------------
usage_exit() {
  # ファイル冒頭のコメントからUSAGEを出力
  sed '/^[^#]/q' "$0"             \
  | sed -n '/^#+/s/^#+//p'        \
  | sed "s/\$0/${CMD_NAME}/g"     1>&2
  exit 1
}
help_exit() {
  # ファイル冒頭のコメントからヘルプを出力
  sed '/^[^#]/q' "$0"             \
  | sed -n '/^#[-+]/s/^#[-+]//p'  \
  | sed "s/\$0/${CMD_NAME}/g"     1>&2
  exit 0
}

# オプション・引数解析
#------------------------------------------------
BUILD_OPTION=
IMAGE_VERSION=latest
while getopts h OPT
do
  case $OPT in
    h  ) # ヘルプの表示
      help_exit
      ;;
    \? ) # 不正オプション時のUSAGE表示
      usage_exit
      ;;
  esac
done
shift $((OPTIND - 1))

# タイムアウト時間
TIMEOUT_SECOND=${1:-15}

# テスト実施
#------------------------------------------------
# シミュレータ起動
(bash ${SCRIPT_DIR}/sim_with_judge_nogui.sh > "${SIM_JUDGE_LOG}") &
SIM_JUDGE_PID=$!

# judgeサーバーが立ち上がるまでの仮待機(時間は要調整)
sleep 10

# シミュレータ起動待ち
while :
do
  sleep 1
  ready_count=$(curl -s http://localhost:5000/warState | jq .ready[] | grep -c 'true' && :)
  echo REDAY_COUNT: $ready_count
  if [ ${ready_count} -eq 2 ]; then
    echo STATE: simulator is running ...
    break
  elif [ ${TIMEOUT_SECOND} -eq 0 ]; then
    echo ERROR: simulator is not running ...
    kill $$
  fi
  TIMEOUT_SECOND=$((TIMEOUT_SECOND - 1)) 
done

# シミュレーション開始
(bash ${SCRIPT_DIR}/start_test.sh  > "${SIM_START_LOG}") &
SIM_START_PID=$!

# シミュレーション終了待ち
TIMEOUT_SECOND=185
while [ ${TIMEOUT_SECOND} -ne 0 ]
do
  sleep 1
  if ! ( curl -s http://localhost:5000/warState | jq .state | grep -c 'running' > /dev/null ) ; then
    # シミュレーション状態が実施中以外になった場合、終了
    #curl -s http://localhost:5000/warState | jq .state,.ready,.scores 
    break
  fi
  TIMEOUT_SECOND=$((TIMEOUT_SECOND - 1)) 
done

# 得点確認
BLUE_POINT=$( curl -s http://localhost:5000/warState | jq .scores.b )
RED_POINT=$( curl -s http://localhost:5000/warState | jq .scores.r )
echo "TEST SCORE (blue vs red): ${BLUE_POINT} vs ${RED_POINT}"

# プロセスを落とす
kill ${SIM_JUDGE_PID}
kill ${SIM_START_PID}

# テストPASS
exit 0
