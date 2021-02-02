# burger_war_kit 保守用手順書
burger_war_kitリポジトリでは、[burger_war_dev](https://github.com/p-robotics-hub/burger_war_dev)に最低限必要なツールやライブラリをインストールしたDockerイメージ(burger-war-kitイメージ)を提供します。

本ドキュメントには、burger-war-kitイメージを開発するための情報を記載しています。

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**

- [Dockerイメージの構成](#docker%E3%82%A4%E3%83%A1%E3%83%BC%E3%82%B8%E3%81%AE%E6%A7%8B%E6%88%90)
- [Docker関連のファイル構成](#docker%E9%96%A2%E9%80%A3%E3%81%AE%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E6%A7%8B%E6%88%90)
- [開発の流れ](#%E9%96%8B%E7%99%BA%E3%81%AE%E6%B5%81%E3%82%8C)
- [事前準備](#%E4%BA%8B%E5%89%8D%E6%BA%96%E5%82%99)
  - [Personal access token の作成](#personal-access-token-%E3%81%AE%E4%BD%9C%E6%88%90)
- [コマンド](#%E3%82%B3%E3%83%9E%E3%83%B3%E3%83%89)
  - [burger-war-kitイメージのビルド](#burger-war-kit%E3%82%A4%E3%83%A1%E3%83%BC%E3%82%B8%E3%81%AE%E3%83%93%E3%83%AB%E3%83%89)
  - [burger-war-kitイメージからコンテナを起動](#burger-war-kit%E3%82%A4%E3%83%A1%E3%83%BC%E3%82%B8%E3%81%8B%E3%82%89%E3%82%B3%E3%83%B3%E3%83%86%E3%83%8A%E3%82%92%E8%B5%B7%E5%8B%95)
  - [ghcr.ioへのログイン](#ghcrio%E3%81%B8%E3%81%AE%E3%83%AD%E3%82%B0%E3%82%A4%E3%83%B3)
  - [burger-war-kitイメージをghcr.ioへプッシュ](#burger-war-kit%E3%82%A4%E3%83%A1%E3%83%BC%E3%82%B8%E3%82%92ghcrio%E3%81%B8%E3%83%97%E3%83%83%E3%82%B7%E3%83%A5)
- [スクリプト設定ファイル](#%E3%82%B9%E3%82%AF%E3%83%AA%E3%83%97%E3%83%88%E8%A8%AD%E5%AE%9A%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB)
- [補足](#%E8%A3%9C%E8%B6%B3)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Dockerイメージの構成


<br />

## Docker関連のファイル構成
burger-war-kitイメージの作成に関連するファイルは以下となります。

```
burger_war_kit
|-- commands
|   |-- config.sh               ... 共通設定ファイル
|   |-- docker-build.sh         ... Dockerイメージをビルドするためのスクリプト
|   |-- docker-launch.sh        ... Dockerイメージからコンテナを起動するためのスクリプト
|   |-- docker-login.sh         ... ghcr.ioにログインするためのスクリプト
|   |-- docker-push.sh          ... pushするためのスクリプト
|-- doc
|   |-- FOR_MAINTAINER.md       ... 本ドキュメント
|-- docker
|   |-- entrypoint.sh           ... DockerのENTRYPOINTで指定する実行スクリプト
|   |-- kit
|   |   |-- Dockerfile          ... burger-war-kitイメージを作成するためのDockerfile
|   |-- templates
|   |   |-- export_env          ... コンテナ内のユーザーに必要な設定(`.bash_profile`と`.bashrc`に追記する内容)
```

<br />

## 開発の流れ
基本的な開発の流れは以下になります。

1. Dockerfileの修正
2. burger-war-kitイメージのビルド
3. burger-war-kitコンテナの動作確認
   - 問題があれば1に戻る
4. 修正したソースコード(Dockerfileなど)をGitHubにプッシュ
5. burger-war-kitイメージをバージョン指定してghcr.ioにプッシュ
6. プッシュしたburger-war-kitイメージを使ってburger-war-devで動作確認
   - 問題があれば1に戻る
7. burger-war-kitイメージを最新版(latest)としてghcr.ioにプッシュ

<br />

## 事前準備

### Personal access token の作成
burger-war-kitイメージをghcr.ioにプッシュするためには、各自のGitHubアカウントで`Personal access token`を作成する必要があります。

以下の手順に従って、[こちらのページ](https://github.com/settings/tokens)から作成して下さい。

<br />

#### 1. Developers settings をクリック
![PAT作成手順1](https://user-images.githubusercontent.com/76457573/106542094-7fbad980-6546-11eb-8f1e-29d968c5403f.png)

<br />

#### 2. Personal access tokens の Generate new tokenをクリック
![PAT作成手順2](https://user-images.githubusercontent.com/76457573/106542236-c4467500-6546-11eb-84e8-76071223a224.png)

<br />

#### 3. Select scopes で権限設定
少なくとも以下にチェックを入れて、ページ下部にある[Generate token]をクリックして下さい。

- write:packages
- read:packages
- delete:packages

![PAT作成手順3](https://user-images.githubusercontent.com/76457573/106542385-15566900-6547-11eb-9763-c89951b94f0d.png)

<br />

#### 4. 生成された Personal access token をファイルに保存

以下のコマンドを実行して、`Personal access token`を保存するファイルを作成して下さい。

```bash
touch $HOME/.github-token
chmod 600 $HOME/.github-token
```

生成された`Personal access token`(下の画像の黒塗り部分)をコピーして、`$HOME/.github-token`に保存して下さい。

![PAT作成手順4](https://user-images.githubusercontent.com/76457573/106542405-21dac180-6547-11eb-9db1-125b09e336cf.png)

以上で、`Personal access token`の作成は完了です。

<br />

## コマンド
以下の操作については、専用のスクリプトを用意しています。

- burger-war-kitイメージのビルド
- burger-war-kitイメージからコンテナ起動
- ghcr.ioへのログイン
- burger-war-kitイメージをghcr.ioへプッシュ

以降の手順は、burger_war_kitのルートディレクトリに移動して実行する想定で記載しています。

```bash 
cd $HOME/catkin_ws/src/burger_war_kit
```

<br />

### burger-war-kitイメージのビルド
修正したDockerfileをビルドしてburger-war-kitイメージを作成するには、以下のコマンドを実行して下さい。

```bash
bash commands/docker-build.sh
```

<br />

### burger-war-kitイメージからコンテナを起動
ビルドして作成したイメージからコンテナを起動するには、以下のコマンドを実行して下さい。

```bash
bash commands/docker-launch.sh
```

起動するコンテナ名はデフォルトでは`burger-war-kit`となります。

起動後は、例えば以下のコマンドでコンテナ内で`bash`を起動することができます。

```bash
docker exec -it bash
```

意図した修正が反映されているか、確認して下さい。

<br />

### ghcr.ioへのログイン
ghcr.ioにログインするには、以下のコマンドを実行して下さい。

```bash
bash commands/docker-login.sh
```

このスクリプトには`Personal access token`を保存した`$HOME/.github-token`が必要です。

予め[こちらの手順](#personal-access-token-の作成)を実施して、作成して下さい。

<br />

### burger-war-kitイメージをghcr.ioへプッシュ
ghcr.ioにイメージをプッシュするには、以下のコマンドを実行します。

```bash
bash commands/docker-push.sh                    # バージョン未指定(burger-war-kit:latestになる)
bash commands/docker-push.sh   -v 202101302145  # バージョン指定時(burger-war-kit:202101302145になる)
```

プッシュしたイメージは、以下のページから確認できます。

[https://github.com/orgs/p-robotics-hub/packages/container/package/burger-war-kit]

<br />

#### 既に同じバージョンが存在する場合
既に同じバージョンのイメージがghcr.ioに登録されている場合は、後からプッシュしたものが古いイメージと置き換えられます。

古いイメージはghcr.io上に残りますが、利用したい場合は以下のようにハッシュ値を指定する必要があります。

```
docker pull ghcr.io/p-robotics-hub/burger-war-kit@sha256:9c337a0021be4b8a24cd8b9b3c2d976b876e6fb611bedb17bde1f7aa7b9579f1
```

DockerfileのFROM命令で指定する場合も同様です。

```
FROM ghcr.io/p-robotics-hub/burger-war-kit@sha256:9c337a0021be4b8a24cd8b9b3c2d976b876e6fb611bedb17bde1f7aa7b9579f1
```

<br />

#### ローカルのファイルだけ更新したい場合
もし、ghcr.ioへプッシュせずにローカルにあるイメージのバージョンだけ更新した場合は、`-l`オプションを付けてください。

```bash
bash commands/docker-push.sh -l
```

以下のように
```
REPOSITORY                                      TAG                    IMAGE ID       CREATED              SIZE
burger-war-kit                                  202101302145           a8b2cdb5fbdd   About a minute ago   3.45GB
ghcr.io/p-robotics-hub/burger-war-kit           202101302145           a8b2cdb5fbdd   About a minute ago   3.45GB
```

<br />

## スクリプト設定ファイル
commandsディレクトリ以下のスクリプトの共通変数は`commands/config.sh`に集約しています。
具体的には以下のような変数により設定を変更できます。

```bash
#----------------------------------------------------------
# Repository config values
#----------------------------------------------------------
# Dockerイメージを登録するレジストリのURL
# $REGISTRY_URL/$KIT_DOCKER_IMAGE_NAME[:version] がURLとなる
REGISTRY_ROOT=ghcr.io
REGISTRY_URL=${REGISTRY_ROOT}/p-robotics-hub

# Dockerイメージ名
KIT_DOCKER_IMAGE_NAME=burger-war-kit
KIT_DOCKER_CONTAINER_NAME=${KIT_DOCKER_IMAGE_NAME}

#----------------------------------------------------------
# Local config values
#----------------------------------------------------------
# 開発者ユーザー名 (変更する場合はburger_war_devも見直すこと)
DEVELOPER_NAME=developer

# GitHubのPersonal access tokensを保存したファイルのパス
GITHUB_TOKEN_FILE=${HOME}/.github-token

# ワークスペースのrootディレクトリのパス
HOST_WS_DIR=${HOME}/catkin_ws

# コンテナ上のワークスペースディレクトリ
CONTAINER_WS_DIR=/home/${DEVELOPER_NAME}/catkin_ws

# ワークスペースのsrcディレクトリのパス
BURGER_WAR_KIT_DIR=${HOST_WS_DIR}/src/burger_war_kit

# ビルドするDockerfileパス
DOCKER_ROOT_DIR=${BURGER_WAR_KIT_DIR}/docker
KIT_DOCKER_FILE_PATH=${DOCKER_ROOT_DIR}/kit/Dockerfile
```

ただし、以下の変数については`burger_war_dev`リポジトリにも影響する為、大会期間中は変更はしないで下さい。

- REGISTRY_ROOT
- REGISTRY_URL
- KIT_DOCKER_IMAGE_NAME
- DEVELOPER_NAME

<br />

## 補足
