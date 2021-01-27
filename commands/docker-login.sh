#!/bin/bash
###############################################################################
#-ghcr.ioにログインする
#-予めユーザ(*1)のアクセストークンを保存したファイル(*2)を用意しておく必要がある
#-(*1) デフォルトでは、$(git config user.email)
#-(*2) デフォルトでは、$HOME/.github-token
#-
#+[USAGE]
#+  $0 [-u リポジトリへのログインユーザ] [-f アクセストークンファイル] [-h]
#+
#-[OPTIONS]
#-  -u user       'docker login -u'でログインするユーザを指定 (default: $(git config user.email))
#-  -f file-path  アクセストークンを保存したファイルパスを指定 (default: $HOME/.github-token)
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
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source "${SCRIPT_DIR}/config.sh"

# オプション・引数解析
#------------------------------------------------
LOGIN_USER=$(git config user.email)
while getopts u:f:h OPT
do
  case $OPT in
    u  ) # Dockerイメージのバージョン指定
      LOGIN_USER="${OPTARG}"
      ;;
    f  ) # パスワードファイルのパスを指定 
      GITHUB_TOKEN_FILE="${OPTARG}"
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

# パスワードファイルがない場合は終了
if [ ! -f "${GITHUB_TOKEN_FILE}" ]; then
  echo "'${GITHUB_TOKEN_FILE}'は存在しません"
  exit 1
fi

# ログイン
#------------------------------------------------
cat "${GITHUB_TOKEN_FILE}" \
  | docker login ${REGISTRY_ROOT} \
      --username ${LOGIN_USER} \
      --password-stdin
