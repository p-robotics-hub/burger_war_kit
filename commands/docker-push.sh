#!/bin/bash
###############################################################################
#-burger-war-kitのイメージをghcr.ioへpushする
#-
#+[USAGE]
#+  $0 [-l] [-v イメージのバージョン] [-h]
#+
#-[OPTIONS]
#-  -l            レジストリにpushせずにローカル環境でのタグ付けのみを行う
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
LOCAL_ONLY=
while getopts lv:h OPT
do
  case $OPT in
    l  ) # レジストリにpushせずにローカル環境でのタグ付けのみを行う
      LOCAL_ONLY=1
      ;;
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
docker tag ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION} ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}

if [ -z "${LOCAL_ONLY}" ]; then
  # push image
  docker push ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}
fi

cat <<-EOM
#--------------------------------------------------------------------
# 以下のリポジトリにイメージをPUSHしました
# URL: ${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}
#--------------------------------------------------------------------
EOM
