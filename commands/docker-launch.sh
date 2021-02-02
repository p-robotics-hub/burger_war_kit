#!/bin/bash
###############################################################################
#-burger-war-kitのDockerコンテナを起動する
#-
#+[USAGE]
#+  $0 [-a RUNオプション] [-v イメージのバージョン] [-w WSディレクトリ] [-h]
#+
#-[OPTIONS]
#-  -a options    'docker run'に追加で渡す引数を指定（複数回指定可能）
#-  -w dir-path   ホストPCのロボコンワークスペースのパスを指定 (default: $HOME/catkin_ws)
#-  -v version    'docker run'で起動するイメージの'version'を指定 (default: latest)
#-  -h            このヘルプを表示
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

# 設定値読み込み
#------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname $0)"; pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
#------------------------------------------------
RUN_OPTION=
IMAGE_VERSION=latest
while getopts a:v:w:h OPT
do
  case $OPT in
    a  ) # docker runへの追加オプション引数指定
      RUN_OPTION="${RUN_OPTION} ${OPTARG}"
      ;;
    w  ) # ホストのワークスペースの指定
      HOST_WS_DIR="${OPTARG}"
      ;;
    v  ) # Dockerイメージのバージョン指定
      IMAGE_VERSION="${OPTARG}"
      ;;
    h  ) # ヘルプの表示
      help_exit
      ;;
    \? ) # 不正オプション時のUSAGE表示
      usage_exit
  esac
done
shift $((OPTIND - 1))

# 同名のコンテナが存在する場合は停止する
#------------------------------------------------
if docker container ls --format '{{.Names}}' | grep -q -e "^${KIT_DOCKER_CONTAINER_NAME}$" ; then
  echo "起動中の ${KIT_DOCKER_CONTAINER_NAME} コンテナを停止します..."
  docker container stop ${KIT_DOCKER_CONTAINER_NAME} >/dev/null
  echo "起動中の ${KIT_DOCKER_CONTAINER_NAME} コンテナを停止しました"
fi

# 同名のコンテナが存在する場合は削除する
#------------------------------------------------
if docker container ls -a --format '{{.Names}}' | grep -q -e "^${KIT_DOCKER_CONTAINER_NAME}$" ; then
  echo "既存の ${KIT_DOCKER_CONTAINER_NAME} コンテナを削除します..."
  docker rm ${KIT_DOCKER_CONTAINER_NAME} >/dev/null
  echo "既存の ${KIT_DOCKER_CONTAINER_NAME} コンテナを削除しました"
fi

# 新たにコンテナを起動する
#------------------------------------------------
set -x
docker run \
  --name ${KIT_DOCKER_CONTAINER_NAME} \
  -d \
  --privileged \
  --net host \
  --mount type=bind,src=/tmp/.X11-unix/,dst=/tmp/.X11-unix \
  --mount type=bind,src=${HOST_WS_DIR},dst=${CONTAINER_WS_DIR} \
  --device /dev/snd \
  --device /dev/shm \
  -e DISPLAY=${DISPLAY} \
  -e HOST_USER_ID=$(id -u) \
  -e HOST_GROUP_ID=$(id -g) \
  ${RUN_OPTION} \
  ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION} \
  tail -f /dev/null
set +x

cat <<-EOM
#--------------------------------------------------------------------
# 開発用のベースコンテナを起動しました
# USE IMAGE NAME: ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}
# CONTAINER NAME: ${KIT_DOCKER_CONTAINER_NAME}
#--------------------------------------------------------------------
EOM
