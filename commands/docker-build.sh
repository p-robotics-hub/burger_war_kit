#!/bin/bash -eu
# Dockerfileのビルドスクリプト
# [USAGE]
#   docker-build.sh [-a docker buildに渡すオプション引数] [-v Dockerイメージのバージョン]
set -e
set -u

CMD_NAME=`basename $0`

# 設定値読み込み
SCRIPT_DIR=$(cd "$(dirname $0)"; pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
BUILD_OPTION=
IMAGE_VERSION=latest
while getopts a:v: OPT
do
  case $OPT in
    "a" ) BUILD_OPTION="${BUILD_OPTION} ${OPTARG}" ;;
    "v" ) IMAGE_VERSION="${OPTARG}" ;;
      * ) echo "USAGE: ${CMD_NAME} [-b docker-build-args] [-v version]" 1>&2
          exit 1 ;;
  esac
done

# Dockerfileのビルド
docker build \
  ${BUILD_OPTION} \
  --build-arg USERNAME=${DEVELOPER_NAME} \
  -f ${KIT_DOCKER_FILE_PATH} \
  -t ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION} \
  ${DOCKER_ROOT_DIR}

echo "#--------------------------------------------------------------------"
echo "# 以下のイメージを作成しました"
echo "# ロボコンベース環境用: ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}"
echo "#--------------------------------------------------------------------"
