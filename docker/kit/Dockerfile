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
        libcanberra-gtk-module \
        dbus-x11 \
        at-spi2-core \
        python-tk \
        python-pil \
        python-pil.imagetk \
        python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-msgs \
        ros-${ROS_DISTRO}-aruco-ros \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-turtlebot3-simulations \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-amcl \
        ros-${ROS_DISTRO}-map-server \
        ros-${ROS_DISTRO}-move-base \
        ros-${ROS_DISTRO}-urdf \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-compressed-image-transport \
        ros-${ROS_DISTRO}-rqt-image-view \
        ros-${ROS_DISTRO}-gmapping \
        ros-${ROS_DISTRO}-navigation \
        ros-${ROS_DISTRO}-interactive-markers \
        ros-${ROS_DISTRO}-hector-gazebo-plugins \
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
    | sed "s@{{ROS_DISTRO}}@${ROS_DISTRO}@g" \
    | tee -a /home/${USERNAME}/.bashrc \
          >> /home/${USERNAME}/.bash_profile \
    && rm /home/${USERNAME}/export_env

RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/

# 環境変数の設定
ENV TZ Asia/Tokyo

# sudo時に引き継ぐ変数の設定
RUN echo 'Defaults        env_keep += "http_proxy https_proxy ftp_proxy no_proxy HTTP_PROXY HTTPS_PROXY FTP_PROXY NO_PROXY"' >> /etc/sudoers

# 初期ディレクトリの指定
WORKDIR ${ROBOCON_WS}

# 起動時の実行コマンドの指定
COPY ./entrypoint.sh /entrypoint.sh
RUN chown ${USERNAME}:${USERNAME} /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && sed -i'' -e "s/^\(DEVELOPER_NAME\)=.*$/\1=${USERNAME}/g" /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" "-l" ]
