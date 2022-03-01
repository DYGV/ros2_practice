# 学習メモ
- [演習1](./memo/02_ex1.md)
  - [目的](./memo/02_ex1.md#目的)
  - [実装](./memo/02_ex1.md#実装)
  - [参考](./memo/02_ex1.md#参考)
- [演習2](./memo/03_ex2.md)
  - [目的](./memo/03_ex2.md#目的)
  - [仕様](./memo/03_ex2.md#仕様)
  - [独自メッセージを定義するパッケージの作成](./memo/03_ex2.md#独自メッセージを定義するパッケージの作成)
  - [作成した独自メッセージでPub/Sub通信をするパッケージの作成](./memo/03_ex2.md#作成した独自メッセージでpubsub通信をするパッケージの作成)
  - [パラメータの値の設定](./memo/03_ex2.md#パラメータの値の設定)
  - [参考](./memo/03_ex2.md#参考)
- [演習3](./memo/04_ex3.md)
  - [目的](./memo/04_ex3.md#目的)
  - [実装](./memo/04_ex3.md#実装)
  - [コマンドラインからノードを立ち上げる場合](./memo/04_ex3.md#コマンドラインからノードを立ち上げる場合)
  - [launchファイルからノードを立ち上げる場合](./memo/04_ex3.md#launchファイルからノードを立ち上げる場合)
  - [参考](./memo/04_ex3.md#参考)
- [サービス通信で人の顔検出を行う](./memo/05_srv_cli.md)  
  - [概要](./memo/05_srv_cli.md#概要)
  - [目的](./memo/05_srv_cli.md#目的)
  - [トピック通信との違い](./memo/05_srv_cli.md#トピック通信との違い)
  - [作成する題材](./memo/05_srv_cli.md#作成する題材)
  - [作成する手順](./memo/05_srv_cli.md#作成する手順)
  - [サービス通信で用いる独自メッセージの作成](./memo/05_srv_cli.md#サービス通信で用いる独自メッセージの作成)
  - [サービス通信でのアプリケーションの作成](./memo/05_srv_cli.md#サービス通信でのアプリケーションの作成)
  - [参考](./memo/05_srv_cli.md#参考)
- [GazeboでTurtleBot3の走行と障害物への衝突回避](./memo/06_gazebo_turtlebot3.md)
  - [概要](./memo/06_gazebo_turtlebot3.md#概要)
  - [目的](./memo/06_gazebo_turtlebot3.md#目的)
  - [とりあえず動かしてみる](./memo/06_gazebo_turtlebot3.md#とりあえず動かしてみる)
  - [障害物への衝突回避(1)](./memo/06_gazebo_turtlebot3.md#障害物への衝突回避1)
  - [障害物への衝突回避(2)](./memo/06_gazebo_turtlebot3.md#障害物への衝突回避2)
  - [参考](./memo/06_gazebo_turtlebot3.md#参考)
- [まとめ](./memo/07_summary.md)

# Docker
## 動作確認済のホスト環境
- ホストOS: Ubuntu 20.04
- Dockerバージョン: 20.10.12

## 構築される環境
- Ubuntu 18.04
- ROS eloquent

## 初回起動
### Dockerイメージの作成
```bash
$ DOCKER_BUILDKIT=1 docker build . -t ubuntu1804_ros2
```
### コンテナからXサーバへ接続するための認証情報の作成
```bash
$ # dockerに対してディスレプレイの使用を許可する
$ xhost +local:docker
$ # X11のソケットファイルの環境変数
$ XSOCK=/tmp/.X11-unix
$ # Xサーバと接続するための認証情報(xauthファイル)のパス
$ XAUTH=/tmp/.docker.xauth
$ # xauthファイルを作成する
$ xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
```
### コンテナの作成
```bash
$ docker create --interactive --tty \
--name=container_ros2 \
--env=DISPLAY=$DISPLAY \
--volume=$XSOCK:$XSOCK \
--volume=/dev:/dev \
--device-cgroup-rule='c *:* rw' \
ubuntu1804_ros2 /bin/bash
```

- -- name: 作成するコンテナの名前
- -- interactive: コンテナの標準入力にアタッチ
- -- tty: 疑似ターミナルを割り当てる
- --env: 環境変数の設定
  - 接続するXサーバの指定
- --volume: ホスト側のディレクトリをとコンテナ側でマウントする
  - Xサーバのソケットファイルのディレクトリをマウント
  - ビデオデバイスを使いたいので、/dev/をマウント
- --device-cgroup-rule: デバイスリストのルール追加
  - プロセスをグループ化、リソースの利用を制限・監視するLinuxカーネルの機能[cgroup](https://man7.org/linux/man-pages/man7/cgroups.7.html)に、マウントしたデバイスのうちキャラクタデバイスの読み書きを可能にするルールを追加する。


### コンテナの起動と接続
Xサーバ関係のファイルを/tmp/に置いているので、PCの再起動後などは、`コンテナからXサーバへ接続するための認証情報の作成`を行ってからコンテナを起動すること。

```bash
$ docker start container_ros2
$ docker exec -it container_ros2 /bin/bash
```

コンテナに入ると起動したアプリケーションのGUIがホスト側で表示できるようになる。

![GUIアプリケーションを起動した様子](https://user-images.githubusercontent.com/8480644/154786230-f24fbaa8-9aa2-46aa-b94c-8b15d7bc34fb.png)

## 参考
- [Docker-docs-ja](http://docs.docker.jp/v19.03/engine/reference/commandline/run.html)
- [ubuntu:20.04 dockerコンテナ内でホスト側にGUIを出力（xeyesを出力）する検討メモ](https://qiita.com/seigot/items/83bc1bb32c04ca36c97f)
- [Docker Ubuntu18.04でtzdataをinstallするときにtimezoneの選択をしないでinstallする](https://qiita.com/yagince/items/deba267f789604643bab)
- [cgroupのDevice Whitelist Controllerについて](https://kamatama41.hatenablog.com/entry/2021/08/10/044822)
- [cgroups(7) - Linux manual page](https://man7.org/linux/man-pages/man7/cgroups.7.html)


