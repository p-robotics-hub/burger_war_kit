#!/bin/bash
###############################################################################
#-burger-war-kitのDockerfileをビルドする
#-
#+[USAGE]
#+  $0 [-a BUILDオプション] [-v イメージのバージョン] [-h]
#+
#-[OPTIONS]
#-  -a options    'docker build'に追加で渡す引数を指定（複数回指定可能）
#-  -v version    'docker build -t'で指定するイメージのバージョンを指定 (default: latest)
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
BUILD_OPTION=
IMAGE_VERSION=latest
while getopts a:v:h OPT
do
  case $OPT in
    a  ) # docker buildへの追加オプション引数指定
      BUILD_OPTION="${BUILD_OPTION} ${OPTARG}"
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

# Dockerfileのビルド
#------------------------------------------------
set -x
docker build \
  ${BUILD_OPTION} \
  --build-arg USERNAME=${DEVELOPER_NAME} \
  -f ${KIT_DOCKER_FILE_PATH} \
  -t ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION} \
  ${DOCKER_ROOT_DIR}
set +x

echo "#--------------------------------------------------------------------"
echo "# 以下のイメージを作成しました"
echo "# ロボコンベース環境用: ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}"
echo "#--------------------------------------------------------------------"
