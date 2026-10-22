# 机の上を片付けるロボット（ブロック押し）

Physical AI 応用編1 の最終課題です。
Gazebo 上の CRANE+ V2 に、机の上のブロックを横に押してどける動きを、模倣学習（ACT）で覚えさせました。

- 動画：YouTube「PAI最終課題_nariponsan」
- 実行環境：講義の Docker イメージ（ROS 2 Humble + Ignition Gazebo）、学習は CPU のみ

## ベースにしたもの

講義（第5回）で使った次の仕組みをそのまま使いました。

- Gazebo の CRANE+ V2 環境（`crane_plus_gazebo` の `crane_plus_with_table.launch.py`）
- `pai_training` の変換（`pixi run convert`）、学習（`pixi run train`）、推論スクリプト（`scripts/infer.py`）

講義の例は「お辞儀」のようなもので、物に触らない動きでした。今回は**物（ブロック）に触って動かすタスク**に変えています。

## 自分で追加・変更したところ

### 1. 手本をプログラムで自動生成（`scripts/push_demo.py`）

テレオペでブロックを押す手本を10回そろえるのは難しかったので、関節角の経由点を決めて自動で再生するスクリプトを作りました。

- 講義のテレオペと同じトピック・同じ関節の並び（joint1, joint2, joint4, joint3）で指令を送るので、記録した rosbag はそのまま `pixi run convert` に使える。変換スクリプトは講義のテレオペのデータが前提。並びが違うと関節が入れ替わり、AIが別の関節を動かしてしまうため。
- 毎回ブロックを元の位置に戻してから動かす（`ign service ... /set_pose`）。前の回のブロックが残ると違う位置から始まってしまうため。手で戻すと位置がずれるうえ、回数が増えると手間でもあるため。
- 毎回、関節角を少し（±0.05 rad）ずらして、少しずつ違う手本にしています。同じ動きだけだと、少しずれただけで続きを出せなくなると考えたため。テレオペで自然に出るくらいのばらつきを入れた。
- 記録は「動き出し」から「腕を上げたところ」までにした。最初と最後が同じ姿勢だと、カメラのない（関節角だけを見る）AIは、これから動くのか止まるのかを区別できないため。
- 手先の高さは、TF で手先の位置を測りながら、テーブルすれすれになる角度に調整しました（`--pose-test`）。`--pose-test` で指令した角度と実際の関節角を比べると、実際の方が 0.2〜0.3 rad 浅く止まっていて（腕の重さなどの影響と考えています）、計算どおりの高さにならなかったため。高すぎると空振りし、低すぎるとテーブルに当たる

```
python3 scripts/push_demo.py                            # 動きの確認だけ
python3 scripts/push_demo.py --record --episodes 10     # 10回分を rosbag に記録
python3 scripts/push_demo.py --noise 0                  # ずらしなし（動画撮影用）
```

### 2. 速度の上限をつけた推論スクリプト（`scripts/infer_safe.py`）

講義の `infer.py` をもとに、以下の2点を追加しました。

- 1回（0.05秒）あたりに指令を動かせる量に上限をつけた（初期値 0.5 rad/秒）
- 決めた秒数（初期値 40秒）で自動的に止まる

```
cd ~/pai_training
pixi run python scripts/infer_safe.py --model-path <学習済みモデルのフォルダ>
```

### 3. 評価スクリプト（`scripts/eval_push.sh`）

「直立に戻す → ブロックを置く → AIに押させる → ブロックの位置を記録」を N 回くり返します。

```
# infer_safe.py と eval_push.sh を /workspace に置いた状態で
bash /workspace/eval_push.sh 5
```

## 試行錯誤の流れ

最初の版（v1）は、ブロックをロボットの後ろ側へ押す手本で、講義の `infer.py` のまま1000回学習させました。動かしてみると、AI の振る速さが手本の約3倍になっていてブロックを弾き飛ばし、ブロックの中心からテーブルの端まで約11cmしかなかったこともあって、床に落としてしまいました（落ちたあとのブロックの高さは 0.025 m。テーブルの上なら 1.015 m。`results/cube_after_infer_v1.txt`）。
そこで v2 では、押す向きをテーブルの広い前側に変え、`infer_safe.py` で速度を 0.5 rad/秒までに抑え、学習も3000回に増やしました（loss は v1 の最後が約1.78、v2 の最後が約0.36）。その結果、5回中4回押せて、床に落ちることはなくなりました。

## 結果（v2、5回）

ブロックの最初の位置は (x, y) = (0.000, -0.100) [m] です。
「テーブルの上に残り、3cm 以上動いた」を成功としました。

| 回 | x | y | 動いた距離 | 判定 |
|---|---|---|---|---|
| 1 | 0.040 | -0.045 | 6.8 cm | 成功 |
| 2 | 0.156 | -0.099 | 15.6 cm | 成功 |
| 3 | 0.029 | -0.062 | 4.8 cm | 成功 |
| 4 | 0.100 | -0.092 | 10.0 cm | 成功 |
| 5 | 0.006 | -0.085 | 1.6 cm | 失敗 |

成功率は 80%（4/5）で、5回とも床には落ちませんでした。動いた距離は、評価スクリプトが Gazebo から取得したブロックの位置と、最初に置いた位置との直線距離です。生データは `results/eval_result_v2.txt` にあります。

## 分かったこと・残った課題

- 押す距離が 1.6〜15.6cm とばらつきました。AI は関節角しか見ていないので、ブロックにどう当たったかが分からず、押し方を調整できません。カメラ画像を入力に加えるのが次の課題です。
- 指令は 0.05 秒ごと（20Hz）に出す設定ですが、40秒間の記録（`results/infer_log_v2.txt`）では指令が206回しか出ておらず、約5Hz でした。CPU だけで AI の計算をしているためと考えています。
- 40秒で止める設定にしたところ、速度制限でゆっくり動くぶん、腕を上げきる前に止まることがありました。秒数はもう少し長くした方がよさそうです。

## ファイル構成

```
scripts/
  push_demo.py     手本の自動再生・記録
  infer_safe.py    速度制限つき推論
  eval_push.sh     くり返し評価
configs/           pai_training の configs/ に置いて使う設定
  push_features.py
  conversion_config.yaml
  train_config.yaml
results/
  eval_result_v2.txt      v2 の5回の評価（押したあとのブロックの位置）
  cube_after_infer_v1.txt v1 で動かしたあとのブロックの位置
  infer_log_v2.txt        v2 を初めて動かしたときの推論ログ
```

## 再現の手順（概要）

1. 講義の手順どおりに Docker と `pai_training` を準備し、Gazebo を起動する
   `ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py`
2. 手本を記録する：`python3 scripts/push_demo.py --record --episodes 10`
3. `configs/` の3ファイルを `~/pai_training/configs/` に置き、変換と学習を行う
   `pixi run convert` → `WANDB_MODE=offline pixi run train`
4. `infer_safe.py` を `~/pai_training/scripts/` に置いて、推論または評価を行う
