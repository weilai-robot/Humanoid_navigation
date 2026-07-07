#!/usr/bin/env python3
"""
nav_state_manager.py — 导航状态管理节点

功能：
  1. 启动时自动按顺序切换机器人进入行走模式：
       idle → zero → stand → walk_leg
  2. 监听 Nav2 NavigateToPose action 反馈，导航完成/取消/失败后：
       先发 stand_mode（切到 PD 稳定），等 stand_to_zero_s 后清零 cmd_vel
  3. 监听 /cmd_vel 持续缺失，作为额外的安全兜底

重要：停止顺序必须是 stand_mode FIRST，然后再清零 cmd_vel
      反过来（cmd_vel=0 → stand_mode）会让 RL 在零速下短暂失稳导致摔倒

订阅：
  /navigate_to_pose/_action/status  (action_msgs/GoalStatusArray)
  /cmd_vel                          (geometry_msgs/Twist)

发布：
  /zero_mode   (std_msgs/Float32)
  /stand_mode  (std_msgs/Float32)
  /walk_mode   (std_msgs/Float32)
  /cmd_vel     (geometry_msgs/Twist)  — 仅用于安全停止时清零

参数：
  auto_start        (bool,  default: true)   启动时自动切换到 walk 模式
  startup_delay_s   (float, default: 1.0)    启动后等待 AimRT 就绪的延迟
  zero_hold_s       (float, default: 2.5)    zero → stand 的等待时间
  stand_hold_s      (float, default: 2.5)    stand → walk 的等待时间
  stop_to_stand_s   (float, default: 0.5)    收到停止信号后发 stand_mode 的延迟
  stand_to_zero_s   (float, default: 2.0)    stand_mode 发出后等待确认稳定的时长（AimRT 内部 hold 1s，此处留余量）
  cmd_vel_timeout_s (float, default: 1.5)    /cmd_vel 停止多久触发安全降级
"""

import threading
import time

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# Nav2 goal 终态集合
_TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}


class NavStateManager(Node):
    def __init__(self):
        super().__init__('nav_state_manager')

        # ── 参数 ──
        self.declare_parameter('auto_start',        True)
        self.declare_parameter('startup_delay_s',   1.0)
        self.declare_parameter('zero_hold_s',       2.5)
        self.declare_parameter('stand_hold_s',      2.5)
        self.declare_parameter('stop_to_stand_s',   0.5)
        self.declare_parameter('stand_to_zero_s',   2.0)
        self.declare_parameter('cmd_vel_timeout_s', 1.5)

        self._auto_start        = self.get_parameter('auto_start').value
        self._startup_delay     = self.get_parameter('startup_delay_s').value
        self._zero_hold         = self.get_parameter('zero_hold_s').value
        self._stand_hold        = self.get_parameter('stand_hold_s').value
        self._stop_to_stand     = self.get_parameter('stop_to_stand_s').value
        self._stand_to_zero     = self.get_parameter('stand_to_zero_s').value
        self._cmd_vel_timeout   = self.get_parameter('cmd_vel_timeout_s').value

        # ── 内部状态 ──
        self._robot_in_walk     = False
        self._stop_triggered    = False
        self._last_cmd_vel_time = None
        self._nav_active        = False
        self._lock              = threading.Lock()

        # ── 发布器 ──
        self._mode_pubs = {
            'zero':  self.create_publisher(Float32, '/zero_mode',  1),
            'stand': self.create_publisher(Float32, '/stand_mode', 1),
            'walk':  self.create_publisher(Float32, '/walk_mode',  1),
        }
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # ── 订阅 Nav2 action 状态 ──
        self._nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav_status_cb,
            10,
        )

        # ── 订阅 cmd_vel 做超时兜底 ──
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_cb,
            10,
        )

        # ── 超时检查定时器（1Hz）──
        self._timeout_timer = self.create_timer(1.0, self._check_cmd_vel_timeout)

        # ── 自动启动序列 ──
        if self._auto_start:
            t = threading.Thread(target=self._startup_sequence, daemon=True)
            t.start()

        self.get_logger().info(
            '[NavStateManager] 启动\n'
            f'  auto_start={self._auto_start}, startup_delay={self._startup_delay}s\n'
            f'  zero_hold={self._zero_hold}s, stand_hold={self._stand_hold}s\n'
            f'  stop->stand={self._stop_to_stand}s, stand_confirm={self._stand_to_zero}s\n'
            f'  cmd_vel_timeout={self._cmd_vel_timeout}s'
        )

    # ────────────────────────────────────────────────────────
    #  启动序列：zero → stand → walk
    # ────────────────────────────────────────────────────────
    def _startup_sequence(self):
        self.get_logger().info(
            f'[NavStateManager] 等待 {self._startup_delay}s 让 AimRT 就绪...'
        )
        time.sleep(self._startup_delay)

        self.get_logger().info('[NavStateManager] → zero_mode')
        self._pub_mode('zero')
        time.sleep(self._zero_hold)

        self.get_logger().info('[NavStateManager] → stand_mode')
        self._pub_mode('stand')
        time.sleep(self._stand_hold)

        self.get_logger().info('[NavStateManager] → walk_mode（机器人进入 RL 行走，等待 Nav2 /cmd_vel）')
        self._pub_mode('walk')

        with self._lock:
            self._robot_in_walk = True
            self._stop_triggered = False

    # ────────────────────────────────────────────────────────
    #  Nav2 goal 状态回调
    # ────────────────────────────────────────────────────────
    def _nav_status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        has_active = any(
            s.status == GoalStatus.STATUS_EXECUTING
            for s in msg.status_list
        )
        latest = msg.status_list[-1]
        just_finished = latest.status in _TERMINAL_STATUSES

        with self._lock:
            prev_active = self._nav_active
            self._nav_active = has_active

        if just_finished and prev_active and not has_active:
            status_name = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_CANCELED:  'CANCELED',
                GoalStatus.STATUS_ABORTED:   'ABORTED',
            }.get(latest.status, str(latest.status))
            self.get_logger().info(
                f'[NavStateManager] Nav2 goal 结束 ({status_name})，触发安全停止'
            )
            self._trigger_stop(delay=self._stop_to_stand)

    # ────────────────────────────────────────────────────────
    #  cmd_vel 回调：记录上次非零活跃时间
    # ────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        is_nonzero = (
            abs(msg.linear.x)  > 0.01 or
            abs(msg.linear.y)  > 0.01 or
            abs(msg.angular.z) > 0.01
        )
        if is_nonzero:
            self._last_cmd_vel_time = time.monotonic()

    # ────────────────────────────────────────────────────────
    #  超时兜底：Nav2 有活跃 goal 但 cmd_vel 长时间停止
    # ────────────────────────────────────────────────────────
    def _check_cmd_vel_timeout(self):
        with self._lock:
            if not self._robot_in_walk or self._stop_triggered:
                return
            if not self._nav_active:
                return
            if self._last_cmd_vel_time is None:
                return

        elapsed = time.monotonic() - self._last_cmd_vel_time
        if elapsed > self._cmd_vel_timeout:
            self.get_logger().warn(
                f'[NavStateManager] cmd_vel 超时 {elapsed:.1f}s，触发安全停止（兜底）'
            )
            self._trigger_stop(delay=0.0)

    # ────────────────────────────────────────────────────────
    #  核心停止流程（防重入）
    # ────────────────────────────────────────────────────────
    def _trigger_stop(self, delay: float = 0.0):
        with self._lock:
            if self._stop_triggered:
                return
            self._stop_triggered = True

        t = threading.Thread(
            target=self._stop_sequence,
            args=(delay,),
            daemon=True,
        )
        t.start()

    def _stop_sequence(self, delay: float):
        """
        正确的停止顺序：
          1. 等待 delay（让机器人把当前步态走完）
          2. 发 stand_mode → AimRT 内部自动完成 walk_stop 过渡保护（1s hold）
             control_module.cc: walk_ready_to_stand_=true, walk_stop_hold_sec_=1.0
          3. 不需要主动发 cmd_vel=0，Nav2 停止发布后自然归零即可
             主动发 cmd_vel=0 反而有风险：若在 stand_mode 生效前到达，RL 零速不稳会摔倒

        walk_stop_hold_sec_ 默认 1.0s，可在 rl_x1.yaml 里用 walk_stop_hold_sec 覆盖
        """
        if delay > 0:
            time.sleep(delay)

        self.get_logger().info('[NavStateManager] → stand_mode（AimRT 内部自动完成 walk_stop 过渡）')
        self._pub_mode('stand')

        # 等待 AimRT 内部 walk_stop_hold_sec(1s) 完成后再标记结束
        # 留 0.5s 余量
        time.sleep(max(self._stand_to_zero, 1.5))

        with self._lock:
            self._robot_in_walk = False

        self.get_logger().info(
            '[NavStateManager] 停止完成，机器人处于 stand 模式'
        )

    # ────────────────────────────────────────────────────────
    #  工具方法
    # ────────────────────────────────────────────────────────
    def _pub_mode(self, mode: str):
        msg = Float32()
        msg.data = 1.0
        self._mode_pubs[mode].publish(msg)

    def _pub_zero_cmd_vel(self):
        self._cmd_vel_pub.publish(Twist())  # 全零


def main(args=None):
    rclpy.init(args=args)
    node = NavStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
