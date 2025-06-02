提供launch
++++++++++

提供worldと、紐付いたlaunchファイルについて説明します。

.. list-table::
   :header-rows: 1

   * - launchファイル名
     - 環境
     - 用途
   * - | hsrb_apartment_world.launch
       | (hsrb_apartment_no_objects_world.launch)
     - | RoboCup\@Home 2016のnavigationタスクの環境を再現しました。
       | 部屋が４つあります。
       | 開閉可能なヒンジドア（ノブと連動）、開閉可能なスライディングドア、家具が配置されています。
       | また一部の家具は、引き出しやドアが開閉可能となっています。
     - * RoboCup\@Home 2016のnavigationタスク
         | 軽量なものを使いたいときはno_objectsをお使いください。
         | ただし、no_objectsはドアもありません。
   * - | hsrb_empty_world.launch
     - | 何も物が置いていない環境です。
     - * ストレスなくHSRを動作させる
         | 軽量なため、実時間に近いRealTimeFactorで動作可能です。
   * - | hsrb_hcr2013_world.launch
     - | マーカーを貼ったオブジェクト（小物、棚）とベットが置いてあるシンプルな環境です。
     - * マーカー認識確認による物取り
   * - | hsrb_megaweb2015_world.launch
       | (hsrb_megaweb2015_no_objects_world.launch)
     - | 中央に広いテーブル、周囲に幅の長い棚などが置いてある環境です。
     - * テーブルや棚を幅広く利用するタスク
         | 軽量なものを使いたいときはno_objectsをお使いください。
   * - | hsrb_mock_home_world.launch
     - | 部屋が３つある環境です。
       | ドア、机、棚等が配置されています。
       | 開閉可能なスライディングドアと家具の一部にマーカーが貼ってあります。
     - * マーカー認識によるドア開け


提供arg
++++++++

設定可能な引数について説明します。

.. list-table::
   :header-rows: 1

   * - パラメータ
     - 役割
     - デフォルト
   * - fast_physics
     - 物理シミュレーションを高速化する(※物理演算が粗くなるため、物体の挙動が不安定になることがあります)
     - false
   * - gui
     - 起動時にViewerを立ち上げる
     - true
   * - rviz
     - 起動時にrvizを立ち上げる
     - true
   * - gazebo_visualization
     - レーザーを表示する
     - false
   * - use_manipulation
     - manipulation機能を使う
     - true
   * - use_navigation
     - navigation機能を使う
     - true
   * - use_perception
     - 認識機能を使う
     - true
   * - use_task
     - タスク機能を使用する（例：Ｗebツールで物拾い）
     - true
   * - use_teleop
     - teleop機能を使う
     - true
   * - use_web
     - Webツールを使う
     - true
   * - use_laser_odom
     - レーザーオドメトリを使う（false：wheelオドメトリ）
     - true
   * - paused
     - gazeboを一時停止状態で起動する
     - true
   * - robot_name
     - 使用するロボットモデル（環境変数 ``ROBOT_NAME`` から取得）
     - hsrb
   * - personal_name
     - ロボットの個別の名前
     - ''（空文字列）

Internal
++++++++

.. ifconfig:: internal

              PKGDOC.rstでargとlaunchの説明を外部公開しているので、
              argとlaunchを追加する際はPKGDOC.rstも更新してください。
