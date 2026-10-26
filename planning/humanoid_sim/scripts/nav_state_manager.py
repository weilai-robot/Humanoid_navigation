#!/usr/bin/env python3
"""
nav_state_manager.py — 双足导航安全看门狗（纯看门狗，v0.3 改造）

定位（方案 docs/nav/改进TODO/P0-nav-state-看门狗改造方案.md）：
  只做异常兜底，单向 walk→stand，绝不 auto-resume（回 walk 需要人的意图，
  重跑 send_nav_goal_real.sh 即可）。模式切换归人/脚本（switch_x1_mode.sh），
  看门狗不抢启动控制权——auto_start 启动序列已删（栈起来自动走比禁用更危险）。

两条触发线：
  ① action 终态：/navigate_to_pose/_action/status 到 SUCCEEDED/CANCELED/ABORTED
     （且机器人在 walk）→ stop_to_stand_s 后发 /stand_mode
  ② 心跳沉默兜底：walk + nav_active 且心跳话题完全无消息超过
     cmd_vel_timeout_s → 立即发 /stand_mode。覆盖 controller 崩溃 /
     odom_bridge 中继死 / 链路断、action 永远无终态的场景——这是发目标
     脚本罩不住的唯一刹车。心跳源 = 下游 /cmd_vel_limiter（odom_bridge
     之后、AimRT 实际消费的流——controller 或 odom_bridge 任一死亡此
     话题都断流；odom_bridge 死时 controller 照发 /cmd_vel 的"假健康"
     只有盯下游才能识破）；本会话从未见过 limiter 消息（relay 未启用/
     未接线）才回退 /cmd_vel。零值消息也算心跳：MPPI 等人期持续发零速
     是正常态（P1 堵路等待语义），只有话题沉默才是链路死。阈值须 > BT
     Wait 时长。

模式事件（事件推断只看"topic 有消息"、忽略 data 数值——/walk_mode 真机
发 0.0、/stand_mode 发 1.0、handoff 又写 0.0，三处口径不一，按 data 判必翻车）：
  /walk_mode 事件 → _robot_in_walk=True + 完整 re-arm（_stop_triggered=False、
    _last_cmd_vel_time=None）。不补 re-arm 看门狗是一次性的：第二个 goal
    到点不发 stand、兜底线永久失效（审核 F1）。
  /stand_mode、/zero_mode 事件 → _robot_in_walk=False（人工切离 walk）
  非零 cmd_vel → 兜底推断 walk + re-arm + 补 _nav_active=True（DDS 发现
    晚丢 /walk_mode 事件的替代入口；nav_active 补位盖"看门狗晚于 goal
    启动/错过唯一 EXECUTING 帧"的盲区——否则两条触发线对执行中的 goal
    双解除，到点不停）。自订阅回声无害：本节点只发 /stand_mode，回声
    只会把 _robot_in_walk 反复置 False，幂等无环。

停止顺序（勿动）：只发 stand_mode、主动不发 cmd_vel=0——stand 先于归零，
反序会让 RL 零速失稳摔倒（KB biped-bridge: stand_before_zero 必须 true）。
AimRT 内部 walk_stop_hold_sec_(1.0) 完成过渡保护。

订阅：/navigate_to_pose/_action/status (action_msgs/GoalStatusArray)
      /cmd_vel (geometry_msgs/Twist)
      /cmd_vel_limiter (geometry_msgs/Twist)——线②心跳判据源
      /walk_mode、/stand_mode、/zero_mode (std_msgs/Float32)
发布：/stand_mode (std_msgs/Float32)——5Hz×2s 持续发，单发在 DDS
      发现未完成时常丢（对齐 switch_x1_mode.sh 既有约定）

参数：
  auto_start         (bool,  default: false) 已废弃（纯看门狗无启动序列），
                                          传 true 只打 WARN，无效果
  stop_to_stand_s    (float, default: 0.5)  终态触发后发 stand_mode 的延迟
  stand_to_zero_s    (float, default: 2.0)  stand_mode 发出后等待稳定的时长
  cmd_vel_timeout_s  (float, default: 3.0)  心跳话题（/cmd_vel_limiter，
                                          回退 /cmd_vel）沉默多久触发兜底
  no_cmd_vel_warn_s  (float, default: 6.0)  输入自检阈值：nav_active 却从未
                                          收到 cmd_vel 的告警线（须 >5s：
                                          正常启动窗口 ≈3.5s，防误报）
"""

import threading
import time
from functools import partial

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
        self.declare_parameter('auto_start',        False)
        self.declare_parameter('stop_to_stand_s',   0.5)
        self.declare_parameter('stand_to_zero_s',   2.0)
        self.declare_parameter('cmd_vel_timeout_s', 3.0)
        self.declare_parameter('no_cmd_vel_warn_s', 6.0)

        self._auto_start      = self.get_parameter('auto_start').value
        self._stop_to_stand   = self.get_parameter('stop_to_stand_s').value
        self._stand_to_zero   = self.get_parameter('stand_to_zero_s').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_s').value
        self._warn_no_cmd_vel = self.get_parameter('no_cmd_vel_warn_s').value

        if self._auto_start:
            self.get_logger().warn(
                '[NavStateManager] auto_start=true 已废弃（纯看门狗无启动序列，'
                '模式切换归人/脚本），本参数无效果'
            )

        # ── 内部状态 ──
        self._robot_in_walk      = False
        self._stop_triggered     = False
        self._stop_in_flight     = False  # 停止序列进行中（re-arm 闸，防并发序列）
        self._last_cmd_vel_time  = None
        self._last_limiter_time  = None   # 下游 /cmd_vel_limiter 心跳（线②判据源）
        self._no_limiter_warned  = False  # "cmd_vel 活但 limiter 从未出现"告警锁存
        self._nav_active         = False
        self._nav_active_since   = None   # nav_active 上升沿时刻（输入自检用）
        self._no_cmd_vel_warned  = False  # 每个 nav_active 周期只 WARN 一次
        self._lock               = threading.Lock()
        self._last_mode_log      = {}     # 模式事件日志节流（5Hz×2s 回声）

        # ── 发布器（只发 stand——纯看门狗不切 walk/zero，不发 cmd_vel）──
        self._mode_pubs = {
            'stand': self.create_publisher(Float32, '/stand_mode', 1),
        }

        # ── 订阅 Nav2 action 状态（线①）──
        self._nav_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav_status_cb,
            10,
        )

        # ── 订阅 cmd_vel 做沉默兜底（线②心跳）──
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_cb,
            10,
        )

        # ── 订阅 /cmd_vel_limiter（线②心跳判据源：odom_bridge 之后的
        #    链路段，controller 或 odom_bridge 任一死都断流）──
        self._limiter_sub = self.create_subscription(
            Twist,
            '/cmd_vel_limiter',
            self._cmd_vel_limiter_cb,
            10,
        )

        # ── 订阅模式事件：/walk_mode re-arm、stand/zero 离开 walk ──
        self._mode_subs = {
            'walk':  self.create_subscription(
                Float32, '/walk_mode',  partial(self._on_mode_event, 'walk'),  10),
            'stand': self.create_subscription(
                Float32, '/stand_mode', partial(self._on_mode_event, 'stand'), 10),
            'zero':  self.create_subscription(
                Float32, '/zero_mode',  partial(self._on_mode_event, 'zero'),  10),
        }

        # ── 超时检查定时器（1Hz）──
        self._timeout_timer = self.create_timer(1.0, self._check_cmd_vel_timeout)

        self.get_logger().info(
            '[NavStateManager] 纯看门狗启动（单向 walk→stand，不做模式启动）\n'
            f'  终态→stand 延迟={self._stop_to_stand}s, stand 确认={self._stand_to_zero}s\n'
            f'  心跳沉默超时={self._cmd_vel_timeout}s（源=/cmd_vel_limiter，'
            '零值=心跳，沉默才算死）\n'
            f'  输入自检阈值={self._warn_no_cmd_vel}s'
        )

    # ────────────────────────────────────────────────────────
    #  模式事件：/walk_mode → 完整 re-arm；stand/zero → 离开 walk
    # ────────────────────────────────────────────────────────
    def _on_mode_event(self, mode: str, msg: Float32):
        # 忽略 msg.data（口径不一，见文件头）；只看"topic 有消息"
        now = time.monotonic()

        if mode == 'walk':
            # 外部切 walk = 新任务开始：完整 re-arm（审核 F1）
            with self._lock:
                self._robot_in_walk = True
                # 同 _cmd_vel_cb：序列进行中不重置（防双序列并发）；序列
                # 收尾的 in_walk=False 会与本处置 True 竞争，后者按真实
                # 状态收敛（已接受边界）
                if not self._stop_in_flight:
                    self._stop_triggered = False
                # None 而非 now()：防旧时间戳在新 goal "已 EXECUTING 但
                # FollowPath 未 tick"窗口秒触发假停机；代价是"controller
                # 死在首帧前"只由输入自检 WARN 覆盖（已接受边界）
                self._last_cmd_vel_time = None
                self._last_limiter_time = None  # 同上（limiter 是线②判据源）
                self._no_limiter_warned = False
        else:
            # 人工 stand/zero：机器人已离开 walk，两条触发线都不该再发
            with self._lock:
                self._robot_in_walk = False

        # 日志节流：外部切换是 5Hz×2s 持续发，一次切换约 10 条事件
        if now - self._last_mode_log.get(mode, 0.0) > 3.0:
            self._last_mode_log[mode] = now
            if mode == 'walk':
                self.get_logger().info(
                    '[NavStateManager] /walk_mode 事件 → in_walk + re-arm（新任务）'
                )
            else:
                self.get_logger().info(
                    f'[NavStateManager] /{mode}_mode 事件 → 机器人已离开 walk'
                )

    # ────────────────────────────────────────────────────────
    #  Nav2 goal 状态回调（线①：终态 → stand）
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
            # 上升沿：重置输入自检计时（每个 nav_active 周期只 WARN 一次）
            if has_active and not prev_active:
                self._nav_active_since = time.monotonic()
                self._no_cmd_vel_warned = False
                self._no_limiter_warned = False
            in_walk = self._robot_in_walk

        if just_finished and prev_active and not has_active:
            status_name = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_CANCELED:  'CANCELED',
                GoalStatus.STATUS_ABORTED:   'ABORTED',
            }.get(latest.status, str(latest.status))
            if in_walk:
                self.get_logger().info(
                    f'[NavStateManager] Nav2 goal 结束 ({status_name})，触发安全停止'
                )
                self._trigger_stop(delay=self._stop_to_stand)
            else:
                # in_walk 闸门（审核附记 E）：人工 zero/stand 中断导航后 goal
                # abort，不再把机器人从 zero（阻尼）拉起站立——stand 对
                # zero 非空操作，不是幂等
                self.get_logger().info(
                    f'[NavStateManager] Nav2 goal 结束 ({status_name})，'
                    '但机器人不在 walk（已 stand/zero），跳过 stand'
                )

    # ────────────────────────────────────────────────────────
    #  cmd_vel 回调（非零兜底推断 + 回退心跳记录；判据源见 limiter）
    # ────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        now = time.monotonic()
        is_nonzero = (
            abs(msg.linear.x)  > 0.01 or
            abs(msg.linear.y)  > 0.01 or
            abs(msg.angular.z) > 0.01
        )
        if is_nonzero:
            # 非零 → 兜底推断 walk + re-arm（DDS 发现晚丢 /walk_mode 事件
            # 时的替代入口）。与 _stop_sequence 收尾的置 False 竞争无害：
            # stand 中的机器人收 cmd_vel 也不会走，False 才是真实状态。
            with self._lock:
                self._robot_in_walk = True
                # 停止序列进行中不许 re-arm stop_triggered：odom_bridge 死
                # 场景 controller 会继续发非零 cmd_vel（它不知道下游死了），
                # 不闸会 1Hz 连开并发停止线程、/stand_mode 双倍 5Hz 轰炸；
                # 代价是新任务前 ≤4s 线② 仍被闸（人重启任务本就 >4s）
                if not self._stop_in_flight:
                    self._stop_triggered = False
                # 补 _nav_active（审核盲区：看门狗晚于 goal 启动/错过唯一
                # EXECUTING 帧 → 线① prev_active=False 跳过、线② nav_active
                # =False 永久解除，到点不停）。非零 cmd_vel ⇒ controller
                # 在算 ⇒ 导航在执行，此处置位方向安全
                if not self._nav_active:
                    self._nav_active = True
                    self._nav_active_since = now
                    self._no_cmd_vel_warned = False
                    self._no_limiter_warned = False

        # 心跳判据：所有消息都记时间（含零值——MPPI 等人期持续发零速
        # 是正常态，只有话题沉默才算链路死）
        self._last_cmd_vel_time = now

    def _cmd_vel_limiter_cb(self, msg: Twist):
        """下游心跳：/cmd_vel_limiter（odom_bridge 中继之后、AimRT 实际
        消费的流），线② 判据源——controller 或 odom_bridge 任一死亡，此
        话题都断流。值不看（等待期零值=正常心跳）。与 _last_cmd_vel_time
        一样只在 executor 线程写，无锁访问安全。
        """
        self._last_limiter_time = time.monotonic()

    # ────────────────────────────────────────────────────────
    #  超时兜底（线②）+ 输入自检（1Hz）
    # ────────────────────────────────────────────────────────
    def _check_cmd_vel_timeout(self):
        now = time.monotonic()
        with self._lock:
            if not self._robot_in_walk or self._stop_triggered:
                return
            if not self._nav_active:
                return
            # 心跳源选下游 /cmd_vel_limiter（controller 或 odom_bridge 任一
            # 死都断流——odom_bridge 死时 controller 照发 /cmd_vel 的"假
            # 健康"只有盯下游才能识破，否则 AimRT 保持末帧盲走 20-35s 才
            # 被进度检查器兜住）；limiter 本会话从未出现过（relay 未启用/
            # 未接线）才回退 /cmd_vel
            if self._last_limiter_time is not None:
                last = self._last_limiter_time
                limiter_seen = True
            else:
                last = self._last_cmd_vel_time
                limiter_seen = False
            active_since = self._nav_active_since
            warned = self._no_cmd_vel_warned
            warned_limiter = self._no_limiter_warned

        if last is None:
            # 输入侧自检（审核 N4）：goal 长期 EXECUTING 却从未收到任何
            # 心跳消息 → 输入链路可能断了（env/话题配错最阴的失效
            # 形态："永远不触发"）。正常启动窗口（切 walk→发 goal→首帧）
            # ≈3.5s，阈值须 >5s（审核 G3）防误报
            if (active_since is not None and not warned
                    and now - active_since > self._warn_no_cmd_vel):
                with self._lock:
                    self._no_cmd_vel_warned = True
                self.get_logger().warn(
                    f'[NavStateManager] 输入自检：nav_active 已 '
                    f'{now - active_since:.1f}s 但从未收到 /cmd_vel 或 '
                    '/cmd_vel_limiter 消息——检查 controller_server 与话题接线'
                )
            return

        elapsed = now - last
        if elapsed > self._cmd_vel_timeout:
            source = '/cmd_vel_limiter' if limiter_seen else '/cmd_vel'
            self.get_logger().warn(
                f'[NavStateManager] {source} 沉默 {elapsed:.1f}s，'
                '触发安全停止（兜底）'
            )
            self._trigger_stop(delay=0.0)
        elif (not limiter_seen and not warned_limiter
                and active_since is not None
                and now - active_since > self._warn_no_cmd_vel):
            # 残余覆盖（odom_bridge 死在首帧前 / relay 未接线，线②判据还
            # 在回退源上）：cmd_vel 活着但 limiter 从未出现 → 中继疑似死，
            # 机器人可能在盲走——WARN 暴露，人工处置
            with self._lock:
                self._no_limiter_warned = True
            self.get_logger().warn(
                '[NavStateManager] 输入自检：/cmd_vel 流动但从未收到 '
                '/cmd_vel_limiter——odom_bridge 中继疑似死亡/未接线'
                '（机器人可能盲走），检查 odom_bridge'
            )

    # ────────────────────────────────────────────────────────
    #  核心停止流程（防重入）
    # ────────────────────────────────────────────────────────
    def _trigger_stop(self, delay: float = 0.0):
        with self._lock:
            if self._stop_triggered:
                return
            self._stop_triggered = True
            self._stop_in_flight = True

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
          2. 发 stand_mode（5Hz×2s 持续发）→ AimRT 内部自动完成 walk_stop
             过渡保护（1s hold）
             control_module.cc: walk_ready_to_stand_=true, walk_stop_hold_sec_=1.0
          3. 不需要主动发 cmd_vel=0，Nav2 停止发布后自然归零即可
             主动发 cmd_vel=0 反而有风险：若在 stand_mode 生效前到达，RL 零速不稳会摔倒

        walk_stop_hold_sec_ 默认 1.0s，可在 rl_x1.yaml 里用 walk_stop_hold_sec 覆盖
        """
        if delay > 0:
            time.sleep(delay)

        self.get_logger().info(
            '[NavStateManager] → stand_mode 5Hz×2s（AimRT 内部自动完成 walk_stop 过渡）'
        )
        self._pub_mode('stand')

        # 等待 AimRT 内部 walk_stop_hold_sec(1s) 完成后再标记结束
        # 留 0.5s 余量
        time.sleep(max(self._stand_to_zero, 1.5))

        with self._lock:
            self._robot_in_walk = False
            self._stop_in_flight = False

        self.get_logger().info(
            '[NavStateManager] 停止完成，机器人处于 stand 模式'
        )

    # ────────────────────────────────────────────────────────
    #  工具方法
    # ────────────────────────────────────────────────────────
    def _pub_mode(self, mode: str):
        """模式消息 5Hz×2s 持续发：DDS 发现未完成时单发常丢
        （对齐 switch_x1_mode.sh 约定）。只在 _stop_sequence 工作线程里
        调用，循环 sleep 不阻塞 spin。
        """
        msg = Float32()
        msg.data = 1.0
        pub = self._mode_pubs[mode]
        for _ in range(10):  # 5Hz × 2s
            pub.publish(msg)
            time.sleep(0.2)


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
