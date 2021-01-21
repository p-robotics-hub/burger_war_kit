#!/bin/bash
###############################################################################
#-ghcr.ioにログインする
#-予めアクセストークンを保存したファイル(*1)を用意しておく必要がある
#-(*1) デフォルトでは、$HOME/.github-token
#-
#+[USAGE]
#+  $0 [-f アクセストークンファイル] [-h]
#+
#-[OPTIONS]
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
while getopts f:h OPT
do
  case $OPT in
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
      --username $(git config user.email) \
      --password-stdin
