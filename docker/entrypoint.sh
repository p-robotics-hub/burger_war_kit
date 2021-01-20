#!/bin/bash
set -e

# DEVELOPER_NAMEはdocker build時に置き換える
DEVELOPER_NAME=developer

# ホストPCとUSER ID/GROUP IDを合わせる(ファイルアクセスできなくなる為)
sudo usermod -u ${HOST_USER_ID} -o -m ${DEVELOPER_NAME} -d /home/${DEVELOPER_NAME}
sudo groupmod -g ${HOST_GROUP_ID} ${DEVELOPER_NAME}

# setup ros environment
# source "${HOME}/.bashrc"
source "/opt/ros/$ROS_DISTRO/setup.bash"
#exec /bin/bash -c "$*"
exec "$@"
