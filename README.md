# ROS_Caffe_prediction_sample
ROS により Subscribe した画像を Caffe で判別を行い、時間計測するサンプル

## 使い方
- ROS、OpenCV が必要です。
- ビルド
    - トップディレクトリで catkin_make します。

            $ catkin_make

- サンプル実行
    - launch/execute.launch を利用して一括実行できます。

            $ source devel/setup.bash
            $ roslaunch launch/execute.launch

    - 起動すると、適当な画像が表示された OpenCV Window が表示されます。
        - s ボタンを押すと、画像が判別スクリプトに Publish されます。
        - 判別スクリプトは画像を Subscribe して、判別を実行。判別時間を計測して ROS_INFO として書き出します。


## 使い方 ( Python 判別プログラム切り替え )
- launch/execute.launch のコメントアウトを切り替えることにより、対象の Python スクリプトを変更します。
    - predictor_simple.py
        - Subscriber の callback で直接 Caffe による判別を実行。
        - 判別遅い。
    - predictor_simple_inner_caffe.py
        - Subscriber の callback で直接 Caffe による判別を実行。
        - callback 内で Caffe を import し直している。判別速い。
    - predictor_thread.py
        - callback から Queue 経由で判別処理 thread に画像を渡して判別実行。
        - 判別遅い。
    - predictor_thread_inner_caffe.py
        - callback から Queue 経由で判別処理 thread に画像を渡して判別実行。
        - 判別処理 thread 内で Caffe を import し直している。判別速い。


