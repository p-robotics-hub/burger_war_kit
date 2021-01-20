#!/bin/bash -eu
set -e
set -u

CMD_NAME=`basename $0`
# 設定値読み込み
SCRIPT_DIR=$(cd "$(dirname $0)"; pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
ARGS=
IMAGE_VERSION=latest
while getopts a:w:v: OPT
do
  case $OPT in
    "a" ) ARGS="${ARGS} ${OPTARG}" ;;
    "w" ) HOST_WS="${OPTARG}" ;;
    "v" ) IMAGE_VERSION="${OPTARG}" ;;
      * ) echo "USAGE: ${CMD_NAME} [-b docker-build-args] [-v version]" 1>&2
          exit 1 ;;
  esac
done

echo "#--------------------------------------------------------------------"
echo "# 開発用のベースコンテナを起動します"
echo "# IMAGE NAME: ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}"
echo "# CONTAINER NAME: ${KIT_DOCKER_IMAGE_NAME}"
echo "#--------------------------------------------------------------------"
# 同名のコンテナが存在する場合は、停止＆削除する
docker container stop ${KIT_DOCKER_IMAGE_NAME}
docker rm ${KIT_DOCKER_IMAGE_NAME}

# 新たにコンテナを起動する
docker run -d --privileged --net host \
    --name ${KIT_DOCKER_IMAGE_NAME} \
    -e DISPLAY=${DISPLAY} \
    -e HOST_USER_ID=$(id -u) \
    -e HOST_GROUP_ID=$(id -g) \
    --mount type=bind,src=/tmp/.X11-unix/,dst=/tmp/.X11-unix \
    --mount type=bind,src=${HOST_WS_DIR},dst=${CONTAINER_WS_DIR} \
    --device /dev/snd \
    ${ARGS} \
    ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION} \
    tail -f /dev/null
