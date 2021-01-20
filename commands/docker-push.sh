#!/bin/bash -eu
set -e
set -u

# 設定値読み込み
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
IMAGE_VERSION=latest
while getopts u:v: OPT
do
  case $OPT in
    "v" ) IMAGE_VERSION="${OPTARG}" ;;
      * ) echo "USAGE: ${CMD_NAME} [-b docker-build-args] [-v version]" 1>&2
          exit 1 ;;
  esac
done

# docker login
bash ${SCRIPT_DIR}/docker-login.sh ${LOGIN_USER}

# add tag
docker tag ${KIT_DOCKER_IMAGE_NAME} ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}

# push images
docker push ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}

echo "#--------------------------------------------------------------------"
echo "# PUSH TO: ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}"
echo "#--------------------------------------------------------------------"
