My libraries
===

FlyCaptureカメラ制御，画像処理で自分がよく使う処理をまとめたもの．
名前募集中．

ほとんどのファイルがファイル名通りの機能を持つ．
一つ一つはあまり長くない．
  以下，一部についてのコメント
  
## 点群保存
[PCL](http://pointclouds.org/ "Point Cloud Library)から，点群を.PLYで保存する部分だけ切り取った．
コンパイル高速化のため．

## カメラ制御
FlyCaptureAPIを使ったカメラ制御を使いやすくした．
使い方はsamples/のファイルを参照されたい．
camera_controller.hpp  -> CのAPIを使用
camera_controller2.hpp -> C++のAPIを使用

## パターン生成
projection_image.hppでできること
*ランダムドットパターン生成
*ランダムストライプパターン生成
*cv::Matの画像をウィンドウ枠なしに表示

## 画像の平行化
ステレオカメラをrectifyする．
ステレオカメラキャリブレーションのときに計算したマップを使う．
使い方はsamples/のファイルを参照されたい．（作成中．お待ち下さい．）
