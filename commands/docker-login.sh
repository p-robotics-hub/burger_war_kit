#!/bin/bash -eu
set -e
set -u

# 設定値読み込み
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source "${SCRIPT_DIR}/config.sh"
[ -f "${GITHUB_TOKEN_FILE}" ] || ( echo "not found '${GITHUB_TOKEN_FILE}'" && exit 1 )

# docker login
cat "${GITHUB_TOKEN_FILE}" | docker login ${REGISTRY_ROOT} --username $(git config user.email) --password-stdin
