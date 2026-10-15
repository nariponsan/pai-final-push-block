#!/usr/bin/env python3
"""
CRANE+ V2（Gazebo）で「ブロックを横に押してどける」デモ動作を自動で再生し、
必要なら rosbag に記録するスクリプト。

- 講義のテレオペ（physicalai_teleop）と同じトピック・同じ関節の並び順で指令を送るので、
  記録したデータはそのまま pai_training の変換（pixi run convert）に使える。
- 毎回わずかに角度をばらつかせて（--noise）、AIが少し違う状況にも対応できるようにする。

使い方（Gazebo を起動した状態で）:
  動きの確認だけ（記録しない）:  python3 push_demo.py
  10回分を自動で記録:            python3 push_demo.py --record --episodes 10
"""
import argparse
import math
import os
import random
import signal
import subprocess
import time

import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 講義のテレオペ・推論と同じ並び順（joint4 と joint3 の順番に注意）
ARM_JOINT_NAMES = ['crane_plus_joint1', 'crane_plus_joint2',
                   'crane_plus_joint4', 'crane_plus_joint3']
GRIPPER_JOINT_NAME = 'crane_plus_joint_hand'
GRIPPER_OPEN = -0.5236
GRIPPER_CLOSE = 0.1745

ARM_TOPIC = '/crane_plus_arm_controller/joint_trajectory'
GRIPPER_TOPIC = '/crane_plus_gripper_controller/joint_trajectory'
RECORD_TOPICS = ['/joint_states', ARM_TOPIC, GRIPPER_TOPIC]

HOME = [0.0, 0.0, 0.0, 0.0]  # [j1, j2, j3, j4]（ここでは分かりやすい順で書く）


# 手先を下ろした姿勢の基準値 [j2, j3, j4]（--lower で上書きできる）
LOWER = [0.10, -1.92, -1.52]  # 2026-10-14 測定値から調整（重力で約0.2rad浅くなる分を上乗せ）
# ブロックをテーブルの広い側（ロボットの前方 +x）へ押すため、後ろ側から前側へ振る。
# （テーブルは x=-0.11〜0.69 なので、後ろ向きに押すと端から落ちやすい）
YAW_START, YAW_END = -2.40, -0.60


def make_waypoints(noise):
    """経由点のリスト: (移動にかける秒数, [j1, j2, j3, j4], グリッパ)"""
    def n():
        return random.uniform(-noise, noise)

    start_yaw = YAW_START + n()   # ブロックの後ろ側へ根元を回す
    end_yaw = YAW_END + n()       # ブロックを前方へ押し出す向きに振る
    j2 = LOWER[0] + n()           # 肩
    j3 = LOWER[1] + n()           # ひじ
    j4 = LOWER[2] + n()           # 手首
    return [
        (1.5, HOME, GRIPPER_CLOSE),                      # 1. 手を閉じて「へら」にする
        (2.0, [start_yaw, 0.0, 0.0, 0.0], GRIPPER_CLOSE),  # 2. ブロックの後ろ側を向く
        (3.0, [start_yaw, j2, j3, j4], GRIPPER_CLOSE),     # 3. 手先をテーブル近くまで下ろす
        (4.0, [end_yaw, j2, j3, j4], GRIPPER_CLOSE),       # 4. 前方へ振ってブロックを押す
        (2.0, [end_yaw, 0.0, 0.0, 0.0], GRIPPER_CLOSE),    # 5. 腕を上げる
    ]
    # 注: 初期姿勢に戻る動きは記録しない（始まりと終わりが同じ姿勢だと、
    #     AIが「動き出す」のか「止まる」のか区別できなくなるため）


class DemoPlayer(Node):
    def __init__(self):
        super().__init__('push_demo_player')
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        # 手先の位置を測るための TF と、実際の関節角
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joints = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def report(self):
        """手先の位置（ロボットの根元から見た座標）と実際の関節角を表示する"""
        try:
            t = self.tf_buffer.lookup_transform('world', 'crane_plus_link_hand', Time())
            p = t.transform.translation
            print(f'  手先の位置 [m]: 前後 x={p.x:+.3f}, 左右 y={p.y:+.3f}, 高さ z={p.z:+.3f}')
            print(f'  （ブロックの中心は おおよそ x=+0.000, y=-0.100, 高さ 0〜0.05）')
        except Exception as e:  # noqa: BLE001
            print(f'  手先の位置を取得できませんでした: {e}')
        names = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
        vals = ', '.join(f'{n[-6:]}={self.joints.get(n, float("nan")):+.2f}' for n in names)
        print(f'  実際の関節角 [rad]: {vals}')

    def send(self, arm, gripper):
        j1, j2, j3, j4 = arm
        jt = JointTrajectory()
        jt.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(j1), float(j2), float(j4), float(j3)]  # 講義と同じ並びに並べ替え
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        jt.points.append(pt)
        self.arm_pub.publish(jt)

        jg = JointTrajectory()
        jg.joint_names = [GRIPPER_JOINT_NAME]
        pg = JointTrajectoryPoint()
        pg.positions = [float(gripper)]
        pg.time_from_start.nanosec = int(0.1 * 1e9)
        jg.points.append(pg)
        self.gripper_pub.publish(jg)

    def hold(self, arm, gripper, seconds, hz=20):
        for _ in range(int(seconds * hz)):
            self.send(arm, gripper)
            time.sleep(1.0 / hz)

    def play(self, waypoints, slow=1.0, hz=20, start=None):
        arm, grip = start if start else (HOME, GRIPPER_OPEN)
        for duration, target_arm, target_grip in waypoints:
            steps = max(1, int(duration * slow * hz))
            for i in range(1, steps + 1):
                s = 0.5 - 0.5 * math.cos(math.pi * i / steps)  # なめらかに加速・減速
                cur = [a + (b - a) * s for a, b in zip(arm, target_arm)]
                cur_g = grip + (target_grip - grip) * s
                self.send(cur, cur_g)
                time.sleep(1.0 / hz)
            arm, grip = target_arm, target_grip
        self.hold(arm, grip, 0.5, hz)  # 最後に少しだけ静止


def reset_cube(name, x, y, z):
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1.0}}'
    r = subprocess.run(['ign', 'service', '-s', '/world/default/set_pose',
                        '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '2000', '--req', req],
                       capture_output=True, text=True)
    ok = 'true' in r.stdout
    print(f'ブロックを元の位置に戻す: {"成功" if ok else "失敗"}'
          + ('' if ok else f'（返事: {r.stdout.strip()} {r.stderr.strip()}）'))


def main():
    p = argparse.ArgumentParser(description='ブロック押し出しデモの自動再生・記録')
    p.add_argument('--record', action='store_true', help='rosbag に記録する')
    p.add_argument('--episodes', type=int, default=1, help='くり返す回数')
    p.add_argument('--out', default=os.path.expanduser('~/dataset/rosbags/push'),
                   help='rosbag の保存先フォルダ')
    p.add_argument('--start-index', type=int, default=0, help='ep番号の開始値')
    p.add_argument('--noise', type=float, default=0.05, help='毎回の角度のばらつき [rad]')
    p.add_argument('--slow', type=float, default=1.5, help='動きをゆっくりにする倍率')
    p.add_argument('--cube', default='wood_cube_5cm')
    p.add_argument('--cube-pose', type=float, nargs=3, default=[0.0, -0.1, 1.05],
                   metavar=('X', 'Y', 'Z'))
    p.add_argument('--lower', type=float, nargs=3, metavar=('J2', 'J3', 'J4'),
                   help='手先を下ろした姿勢の関節角 [rad] を上書きする')
    p.add_argument('--yaw', type=float, nargs=2, metavar=('START', 'END'),
                   help='根元の回転の開始角と終了角 [rad] を上書きする')
    p.add_argument('--pose-test', action='store_true',
                   help='手先を下ろした姿勢まで動かして10秒止まる（位置の確認用）')
    args = p.parse_args()

    global LOWER, YAW_START, YAW_END
    if args.lower:
        LOWER = list(args.lower)
    if args.yaw:
        YAW_START, YAW_END = args.yaw

    rclpy.init()
    node = DemoPlayer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)  # 受信用
    spin_thread.start()
    time.sleep(1.0)  # 送信の準備待ち
    try:
        if args.pose_test:
            reset_cube(args.cube, *args.cube_pose)
            target = [YAW_START] + LOWER
            print(f'確認用の姿勢 [j1, j2, j3, j4] = {target} に動かします...')
            node.play([(1.5, HOME, GRIPPER_CLOSE), (2.0, [YAW_START, 0, 0, 0], GRIPPER_CLOSE),
                       (3.0, target, GRIPPER_CLOSE)], slow=args.slow)
            print('止まるのを待っています（10秒）...')
            node.hold(target, GRIPPER_CLOSE, 10.0)
            print('\n===== 測定結果 =====')
            node.report()
            print('====================\n直立に戻します...')
            node.play([(2.0, [YAW_START, 0, 0, 0], GRIPPER_CLOSE), (2.0, HOME, GRIPPER_OPEN)],
                      slow=args.slow, start=(target, GRIPPER_CLOSE))
            print('完了しました。')
            return
        for k in range(args.episodes):
            ep = args.start_index + k
            print(f'\n=== エピソード {ep} ===')
            print('ブロックを元の位置に戻し、初期姿勢で待機します...')
            reset_cube(args.cube, *args.cube_pose)
            node.hold(HOME, GRIPPER_OPEN, 3.0)

            rec = None
            if args.record:
                os.makedirs(args.out, exist_ok=True)
                path = os.path.join(args.out, f'ep{ep}')
                if os.path.exists(path):
                    print(f'[スキップ] {path} はすでにあります')
                    continue
                rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', path] + RECORD_TOPICS,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f'記録開始: {path}')
                time.sleep(2.5)  # 記録の準備待ち（指令は送らない＝止まっている時間を記録に残さない）

            print('デモ動作を再生中...')
            wps = make_waypoints(args.noise)
            node.play(wps, slow=args.slow)

            if rec is not None:
                rec.send_signal(signal.SIGINT)
                rec.wait(timeout=15)
                print('記録終了')

            print('初期姿勢に戻ります（記録しない）...')
            last_arm, last_grip = wps[-1][1], wps[-1][2]
            node.play([(2.5, HOME, GRIPPER_OPEN)], slow=args.slow, start=(last_arm, last_grip))
        print('\nすべて完了しました。')
    except KeyboardInterrupt:
        print('\n中断しました。')
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
