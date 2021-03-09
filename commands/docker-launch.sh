#!/bin/bash
###############################################################################
#-burger-war-kitのDockerコンテナを起動する
#-
#+[USAGE]
#+  $0 [-a RUNオプション] [-f][-r] [-v イメージのバージョン] [-w WSディレクトリ] [-h]
#+
#-[OPTIONS]
#-  -a options    'docker run'に追加で渡す引数を指定（複数回指定可能）
#-  -f            既存のコンテナを削除して、新しいコンテナを作成し起動する
#-  -r            既存のコンテナを前回の設定で再起動する
#-  -R            ghcr.io上にプッシュされているdockerイメージを使用する
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
print_error() {
  # 引数のエラーメッセージを出力
  echo -n -e "\e[31m"
  echo -e "$@" | xargs -I{} echo -e {}
  echo -n -e "\e[m"
}
remove_container() {
  # 既存のコンテナを削除
  if [ "${1}" == "-n" ]; then
    docker rm ${2} >/dev/null
  else
    echo ""
    echo "既存の ${1} コンテナを削除します..."
    docker rm ${1} >/dev/null
    echo "既存の ${1} コンテナを削除しました"
  fi
}
save_contianer() {
  # 既存のコンテナを保存
  container_name=${1}
  echo ""
  echo -e "既存のコンテナをイメージとして保存します"
  read -p "保存するバージョン名を入力して下さい: " backup_version
  if [ -z "${backup_version}" ]; then
    echo "バージョン名が不正です。起動処理を中断します"
    exit 1
  fi
  docker commit ${container_name} ${RUN_DOCKER_IMAGE_NAME}:${backup_version} >/dev/null
  cat <<-EOM_SAVE
	#--------------------------------------------------------------------
	# 既存のコンテナを以下のイメージとして保存しました
	# SAVE IMAGE NAME: ${RUN_DOCKER_IMAGE_NAME}:${backup_version}
	#
	# 保存したイメージからコンテナを起動するには、以下のコマンドを実行して下さい
	# RUN COMMAND    : bash commands/docker-launch.sh -t ${RUN_TARGET} -v ${backup_version}
	#--------------------------------------------------------------------
EOM_SAVE
  docker rm ${container_name} >/dev/null
}
print_run_message() {
  # 起動したコンテナの情報を出力
  container_name=${1}
  image_name=$(docker ps -f "name=${container_name}" --format "{{.Image}}")
  if [ -z "${image_name}" ]; then
    # 起動失敗時
    print_error \
      "#--------------------------------------------------------------------\n" \
      "# コンテナの起動に失敗しました...\n" \
      "#--------------------------------------------------------------------"
    read -p "Dockerのログを確認しますか？(y/n): " yesno
    case ${yesno} in
      y|yes|Y|YES ) # Dockerのログを出力する
        set -x
        docker logs "${container_name}"
        set +x
        ;;
      * ) # 出力しない
        ;;
    esac
    exit 1
  fi
  # 起動成功時
  echo ""
  echo "#--------------------------------------------------------------------"
  echo "# 開発用のコンテナを起動しました"
  echo "# USE IMAGE NAME: ${image_name}"
  echo "# CONTAINER NAME: ${container_name}"
  echo "#--------------------------------------------------------------------"
}

# 設定値読み込み
#------------------------------------------------
SCRIPT_DIR=$(cd "$(dirname $0)"; pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
#------------------------------------------------
RUN_OPTION=
IMAGE_VERSION=latest
RUN_TARGET=kit
RUN_DOCKER_IMAGE_NAME=${KIT_DOCKER_IMAGE_NAME}
RUN_DOCKER_CONTAINER_NAME=${KIT_DOCKER_CONTAINER_NAME}
RESTART_CONTAINER_REQUEST=
FORCE_NEW_CONTAINER_REQUEST=
while getopts a:frRv:w:h OPT
do
  case $OPT in
    a  ) # docker runへの追加オプション引数指定
      RUN_OPTION="${RUN_OPTION} ${OPTARG}"
      ;;
    f  ) # 既存のコンテナを削除して新しいコンテナを起動
      FORCE_NEW_CONTAINER_REQUEST=1
      ;;
    r  ) # 既存のコンテナを再起動
      RESTART_CONTAINER_REQUEST=1
      ;;
    R  ) # ghcr.io上の Dockerイメージを使用する
      KIT_DOCKER_IMAGE_NAME="${REGISTRY_URL}/${KIT_DOCKER_IMAGE_NAME}"
      ;;
    v  ) # Dockerイメージのバージョン指定
      IMAGE_VERSION="${OPTARG}"
      ;;  
    w  ) # ホストのワークスペースの指定
      HOST_WS_DIR="${OPTARG}"
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

RUN_DOCKER_IMAGE_NAME_FULL=${RUN_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}

# 指定のコンテナが存在するかチェック
#------------------------------------------------
if [ -n "${RESTART_CONTAINER_REQUEST}" ]; then
  if docker ps -a --format '{{.Image}}' | grep -q -E "^${RUN_DOCKER_IMAGE_NAME_FULL}$" ; then
    # 指定Dockerイメージから起動したコンテナが存在する場合
    :
  else
    # 指定Dockerイメージから起動したコンテナが存在しない場合
    echo -e "\e[33m指定のイメージ ${RUN_DOCKER_IMAGE_NAME_FULL} から起動したコンテナは存在しません"
    echo -e "${RUN_DOCKER_IMAGE_NAME_FULL} から新しいコンテナを起動します\e[m"
    RESTART_CONTAINER_REQUEST=
  fi
fi

# 指定のDockerイメージが存在するかチェック
#------------------------------------------------
if docker images "${RUN_DOCKER_IMAGE_NAME_FULL}" | awk 'NR>1{print $1":"$2}' \
  | grep -q -E "${RUN_DOCKER_IMAGE_NAME_FULL}"; then
  # 指定イメージが存在する場合
  :
else
  # 指定イメージが存在しない場合
  print_error "指定のイメージ ${RUN_DOCKER_IMAGE_NAME_FULL} は存在しません"
  exit 1
fi

# 起動中の同名のコンテナが存在する場合は停止する
#------------------------------------------------
if docker ps --format '{{.Names}}' | grep -q -e "^${RUN_DOCKER_CONTAINER_NAME}$" ; then
  echo "起動中の ${RUN_DOCKER_CONTAINER_NAME} コンテナを停止します..."
  docker container stop ${RUN_DOCKER_CONTAINER_NAME} >/dev/null
  echo "起動中の ${RUN_DOCKER_CONTAINER_NAME} コンテナを停止しました"
fi

# 同名のコンテナが存在する場合
#------------------------------------------------
if docker ps -a --format '{{.Names}}' | grep -q -e "^${RUN_DOCKER_CONTAINER_NAME}$" ; then
  if [ -n "${RESTART_CONTAINER_REQUEST}" ]; then
    # オプションにより既存コンテナの再起動を指定済みのため確認はスキップ
    :
  elif [ -n "${FORCE_NEW_CONTAINER_REQUEST}" ]; then
    # オプションにより新コンテナの作成を指定済みのため確認はスキップ
    remove_container "${RUN_DOCKER_CONTAINER_NAME}"
  else
    # ユーザーによる起動方法の選択
    echo -e "\e[33m前回起動していた ${RUN_DOCKER_CONTAINER_NAME} コンテナが存在します"
    echo -e "コンテナを起動する方法を以下から選択できます"
    echo -e "---------------------------------------------------------"
    echo -e "  1: 既存のコンテナを再起動する"
    echo -e "  2: 既存のコンテナを保存して新しいコンテナを起動する"
    echo -e "  3: 既存のコンテナを削除して新しいコンテナを起動する"
    echo -e "----------------------------------------------------------"
    read -p "選択肢の番号を入力して下さい(1〜3): " choice_numer
    case ${choice_numer} in
      2 ) # 既存のコンテナを別名で保存
        save_contianer "${RUN_DOCKER_CONTAINER_NAME}"
        ;;
      3 ) # 既存のコンテナを削除
        remove_container "${RUN_DOCKER_CONTAINER_NAME}"
        ;;
      * ) # 既存のコンテナを再起動
        RESTART_CONTAINER_REQUEST=1
        ;;
    esac
    echo -e "\e[m"
  fi
fi


# コンテナを起動する
#------------------------------------------------
if [ -n "${RESTART_CONTAINER_REQUEST}" ]; then
  # 既存のコンテナを再起動
  echo -e "既存のコンテナを前回起動時の設定で起動します\n"
  set -x
  docker start ${RUN_DOCKER_CONTAINER_NAME}
  set +x
else
  # 新しくコンテナを起動
  set -x
  docker run \
    --name ${RUN_DOCKER_CONTAINER_NAME} \
      -d \
    --privileged \
    --net host \
    --mount type=bind,src=/tmp/.X11-unix/,dst=/tmp/.X11-unix \
    --mount type=bind,src=${HOST_WS_DIR},dst=${CONTAINER_WS_DIR} \
    --device /dev/snd \
    -v /dev/shm \
    -e DISPLAY=${DISPLAY} \
    -e HOST_USER_ID=$(id -u) \
    -e HOST_GROUP_ID=$(id -g) \
    ${RUN_OPTION} \
    ${RUN_DOCKER_IMAGE_NAME_FULL} \
    tail -f /dev/null
  set +x
fi

cat <<-EOM
#--------------------------------------------------------------------
# 開発用のベースコンテナを起動しました
# USE IMAGE NAME: ${KIT_DOCKER_IMAGE_NAME}:${IMAGE_VERSION}
# CONTAINER NAME: ${RUN_DOCKER_CONTAINER_NAME}
#--------------------------------------------------------------------
EOM
