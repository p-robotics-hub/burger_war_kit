#!/bin/bash
###############################################################################
#-開発用コンテナ上でburger-war-kitディレクトリに移動してコマンドを実行する
#-
#+[USAGE]
#+  $0 [-a EXECオプション] [-h] [-s] [-c] 実行コマンド
#+
#-[OPTIONS]
#-  -a options    'docker exec'に追加で渡す引数を指定（複数回指定可能）
#-  -c            'bash -c'により引数のコマンドを実行する
#-  -s            burger-war-kit/scripts/以下のスクリプトを実行する
#-  -h            このヘルプを表示
#-
#-[ARGUMENTS]
#-  script-name  実行するスクリプト名を指定する
#-
###############################################################################
set -e
set -u
CMD_NAME=$(basename $0)

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
print_error() {
  # 引数のエラーメッセージを出力
  echo -n -e "\e[31m"
  echo -e "$@" | xargs -I{} echo -e {}
  echo -n -e "\e[m"
}

# 設定値読み込み
#------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname $0)"; pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
#------------------------------------------------
EXEC_OPTION=
BASH_OPTION=
BASH_ARGS=
EXEC_SCRIPT=
RUN_DOCKER_CONTAINER_NAME=${KIT_DOCKER_CONTAINER_NAME}
while getopts a:csh OPT
do
  case $OPT in
    a  ) # docker execへの追加オプション引数指定
      EXEC_OPTION="${EXEC_OPTION} ${OPTARG}"
      ;;
    c  ) # bashにコマンドを渡して実行する場合
      BASH_OPTION="-l -c"
      break
      ;;
    s  ) # docker execへの追加オプション引数指定
      EXEC_SCRIPT=1
      BASH_OPTION="-l -c"
      break
      ;;
    h  ) # ヘルプの表示
      help_exit
      ;;
    \? ) # 不正オプション時のUSAGE表示
      usage_exit
      ;;
  esac
done
shift $((OPTIND - 1))

# bashに渡す引数を設定
if [ "$EXEC_SCRIPT" != "" ]; then
  # burger_war_kit/scriptsディレクトリ配下のスクリプトを実行する場合
  EXEC_COMMAND="cd ${CONTAINER_WS_DIR}/src/burger_war_kit && bash -- scripts/$*"
else
  # 任意のコマンドを実行する場合
  EXEC_COMMAND="--"
  [ $# -ne 0 ] && EXEC_COMMAND="$*"
fi

# 起動中の指定コンテナが存在するかチェック
#------------------------------------------------
if docker ps --format '{{.Names}}' | grep -q -E "${RUN_DOCKER_CONTAINER_NAME}" ; then
  # 指定コンテナが存在する場合
  :
else
  # 指定コンテナが存在しない場合
  print_error "起動中の指定のコンテナは存在しません: ${RUN_DOCKER_CONTAINER_NAME}"
  exit 1
fi

# コマンドの実行
#------------------------------------------------
cat <<-EOM
#--------------------------------------------------------------------
# 以下のコンテナでコマンドを実行します
# CONTAINER NAME: ${RUN_DOCKER_CONTAINER_NAME}
# EXEC COMMAND  : bash ${BASH_OPTION} ${EXEC_COMMAND}
#--------------------------------------------------------------------
EOM
set -x
docker exec \
  -it \
  --user $(id -u) \
  ${EXEC_OPTION} \
  ${RUN_DOCKER_CONTAINER_NAME} \
  bash ${BASH_OPTION} "${EXEC_COMMAND}"
