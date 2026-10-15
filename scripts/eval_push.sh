#!/bin/bash
# 学習したAIで「ブロック押し」を N 回くり返し、毎回のブロックの位置を記録する評価用スクリプト。
# 使い方（~/pai_training で）: bash /workspace/eval_push.sh [回数] [モデルのフォルダ]
N=${1:-5}
MODEL=${2:-/home/ubuntu/checkpoints/train/crane_plus_push_act}
OUT=/workspace/eval_result.txt
cd ~/pai_training || exit 1
cp /workspace/infer_safe.py scripts/

home() {  # ロボットを初期姿勢（直立・手を開く）に戻す
  ros2 topic pub --once /crane_plus_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [crane_plus_joint1, crane_plus_joint2, crane_plus_joint4, crane_plus_joint3], points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4}}]}" > /dev/null
  ros2 topic pub --once /crane_plus_gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [crane_plus_joint_hand], points: [{positions: [-0.5236], time_from_start: {sec: 2}}]}" > /dev/null
  sleep 6
}

reset_cube() {
  ign service -s /world/default/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
    --timeout 2000 --req 'name: "wood_cube_5cm", position: {x: 0.0, y: -0.1, z: 1.05}, orientation: {w: 1.0}' > /dev/null
  sleep 2
}

echo "trial x y z yaw" > $OUT
for i in $(seq 1 $N); do
  echo "=== 試行 $i / $N ==="
  home
  reset_cube
  pixi run python scripts/infer_safe.py --model-path $MODEL > /workspace/eval_infer_$i.txt 2>&1
  POSE=$(ign model -m wood_cube_5cm -p | grep -A2 "Pose \[" | tail -2 | tr -d '[]' | tr '\n' ' ')
  # POSE = "x y z roll pitch yaw"
  echo "$i $(echo $POSE | awk '{print $1, $2, $3, $6}')" >> $OUT
  echo "試行 $i のブロック位置: $POSE"
done
home
echo "評価が終わりました。結果: $OUT"
cat $OUT
