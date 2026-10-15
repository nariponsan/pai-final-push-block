#!/usr/bin/env python3
"""
講義の推論スクリプト（scripts/infer.py）に「速度制限」を加えた版。

学習したAI（ACT）は、関節の今の角度を見て次の目標角度を出す。
シミュレータのロボットは指令にすばやく追従するため、AIの出した動きが
手本（デモ）より何倍も速くなり、ブロックを弾き飛ばしてしまうことがあった。
そこで、1回（0.05秒）あたりに指令を動かせる量に上限を付けて、手本に近い速さで動かす。

使い方（~/pai_training で）:
  pixi run python scripts/infer_safe.py --model-path <学習済みモデル> [--max-vel 0.5]
"""
import argparse
import time

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.common.policies.act.modeling_act import ACTPolicy

ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'
DT = 0.05  # 20Hz（講義と同じ）


class SafeInferenceNode(Node):
    def __init__(self, model_path, max_vel, duration):
        super().__init__('crane_plus_act_infer_safe')
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        print('AIを読み込んでいます...', flush=True)
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.policy.to(torch.device('cpu'))
        self.max_step = max_vel * DT          # 1回あたりに動かせる最大角度 [rad]
        self.duration = duration              # この秒数たったら自動で止まる（0なら止まらない）
        self.latest_joint_state = None
        self.command = None                   # 直前に送った指令（速度制限の基準）
        self.start_time = None
        self.done = False
        self.create_timer(DT, self.publish_action)
        print(f'準備完了。速度の上限 {max_vel:.2f} rad/秒 で動かします。', flush=True)

    def joint_state_callback(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        arm_pos = [name_to_pos.get(j, 0.0) for j in ARM_JOINT_NAMES]
        gripper_pos = name_to_pos.get(GRIPPER_JOINT_NAME, 0.0)
        self.latest_joint_state = np.array(arm_pos + [gripper_pos], dtype=np.float32)

    def publish_action(self):
        if self.latest_joint_state is None or self.done:
            return
        if self.start_time is None:
            self.start_time = time.time()
            self.command = self.latest_joint_state.copy()   # 今の姿勢から動き始める
        if self.duration > 0 and time.time() - self.start_time > self.duration:
            print(f'{self.duration:.0f}秒たったので終了します。', flush=True)
            self.done = True
            return

        state = torch.tensor(self.latest_joint_state, dtype=torch.float32).unsqueeze(0)
        obs = {'observation.state': state, 'observation.environment_state': state}
        with torch.no_grad():
            target = self.policy.select_action(obs).cpu().numpy().flatten()  # AIが出した目標 (5,)

        # ここが追加部分: 目標に向かって、1回あたり max_step までしか進めない
        step = np.clip(target - self.command, -self.max_step, self.max_step)
        self.command = self.command + step

        self.send(ARM_JOINT_NAMES, self.command[:4], self.arm_pub)
        self.send([GRIPPER_JOINT_NAME], self.command[4:5], self.gripper_pub)
        print('target:', np.round(target[:4], 3), 'command:', np.round(self.command[:4], 3), flush=True)

    @staticmethod
    def send(names, positions, pub):
        jt = JointTrajectory()
        jt.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        jt.points.append(pt)
        pub.publish(jt)


def main():
    p = argparse.ArgumentParser(description='速度制限つき ACT 推論ノード（CRANE+ V2）')
    p.add_argument('--model-path', required=True, help='学習済みモデルのフォルダ')
    p.add_argument('--max-vel', type=float, default=0.5, help='関節の速度の上限 [rad/秒]')
    p.add_argument('--duration', type=float, default=40.0, help='この秒数で自動終了（0なら無制限）')
    args = p.parse_args()

    rclpy.init()
    node = SafeInferenceNode(args.model_path, args.max_vel, args.duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n中断しました。')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
