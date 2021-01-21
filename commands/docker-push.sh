#!/bin/bash
###############################################################################
#-burger-war-kitのイメージをghcr.ioへpushする
#-予めアクセストークンファイル($HOME/.github-token)を用意しておく必要がある
#-
#+[USAGE]
#+  $0 [-v イメージのバージョン] [-h]
#+
#-[OPTIONS]
#-  -v version    'docker tag -t'で指定するイメージのバージョンを指定 (default: latest)
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
IMAGE_VERSION=latest
while getopts v:h OPT
do
  case $OPT in
    v  ) # Dockerイメージのバージョン指定
      IMAGE_VERSION="${OPTARG}"
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

# イメージにタグ付けしてpushする
#------------------------------------------------
# add tag
docker tag ${KIT_DOCKER_IMAGE_NAME} ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}

# push image
docker push ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}

echo "#--------------------------------------------------------------------"
echo "# 以下のリポジトリにイメージをPUSHしました"
echo "# URL: ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}"
echo "#--------------------------------------------------------------------"
