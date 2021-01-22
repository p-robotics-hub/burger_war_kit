ARG ROS_DISTRO=melodic
FROM osrf/ros:${ROS_DISTRO}-desktop-full


# Dockerのビルドに必要な環境変数の設定
#------------------------------------------------


# Dockerのビルドに必要なツールのインストール
#------------------------------------------------
RUN apt-get update -q && apt-get upgrade -yq && apt-get install -y --no-install-recommends \
        libglib2.0-dev-bin \
        dh-python \
        apt-utils \
        python-pip \
        git \
        lsb-release \
        tzdata \
        gosu \
    && pip install --upgrade pip \
    && rm -rf /var/lib/apt/lists/*


# 依存ツール/ライブラリのインストール
#------------------------------------------------
RUN apt-get update -q && apt-get install -y --no-install-recommends \
        gnome-terminal \
        dbus-x11 \
        at-spi2-core \
        python-tk \
        python-pil.imagetk \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-msgs \
        ros-${ROS_DISTRO}-aruco-ros \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

RUN yes | pip install \
    requests \
    flask


# コンテナ内で使いたいツール等をインストール
#------------------------------------------------
RUN apt-get update -q && apt-get install -y --no-install-recommends \
        vim \
        tmux \
    && rm -rf /var/lib/apt/lists/*


# 開発者ユーザーの追加
#------------------------------------------------
ARG USERNAME=developer
RUN useradd --create-home --home-dir /home/${USERNAME} --shell /bin/bash --user-group --groups adm,sudo,video,audio ${USERNAME} \
    && echo ${USERNAME}:${USERNAME} | chpasswd \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


# ROSの初期化
#------------------------------------------------
RUN test -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init && \
    gosu ${USERNAME} rosdep update


# 補助スクリプトのコピー
#------------------------------------------------
COPY ./scripts /home/${USERNAME}/scripts
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/scripts \
    && chmod -R +x /home/${USERNAME}/scripts


# ワークスペース作成
#------------------------------------------------
ARG ROBOCON_WS=/home/${USERNAME}/catkin_ws


# コンテナ起動時の設定
#------------------------------------------------
# 必要なファイルを展開
COPY ./templates/ /home/${USERNAME}/

# 永続化したい設定を.bashrc/.bash_profileに追加
RUN cat /home/${USERNAME}/export_env \
    | sed "s@{{ROBOCON_WS}}@${ROBOCON_WS}@g" \
    | tee -a /home/${USERNAME}/.bashrc \
          >> /home/${USERNAME}/.bash_profile \
    && rm /home/${USERNAME}/export_env

RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/

# 環境変数の設定
ENV TZ Asia/Tokyo

# 初期ディレクトリの指定
WORKDIR ${ROBOCON_WS}

# 起動時の実行コマンドの指定
COPY ./entrypoint.sh /entrypoint.sh
RUN chown ${USERNAME}:${USERNAME} /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && sed -i'' -e "s/^\(DEVELOPER_NAME\)=.*$/\1=${USERNAME}/g" /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" "-l" ]
