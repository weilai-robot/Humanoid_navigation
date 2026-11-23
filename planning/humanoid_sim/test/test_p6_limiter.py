#!/usr/bin/env python3
"""P6 单测（审核 §3 断言清单 7 条）——不依赖 rclpy 运行环境。

用 sys.modules 桩替换 rclpy / 消息库后导入 odom_bridge，直接实例化 OdomBridge
（真实 __init__ + 回调 + 沉默定时器逻辑），用可拨动的假时钟驱动：

  ① 任意序列收敛速率 ≤ 夹板×dt（加速向/减速向/wz 两把减速夹板分别断言）；
  ② 首帧直通、dt≤0 不动（原语义保持）；
  ③ 沉默+末目标≈0：快车道 T 内归零，发布只出现在值变化 tick；
  ④ 沉默+末目标非零：T 前零发布，T 后距离预算内归零（0.3 m/s → 22.5cm）；
  ⑤ ramp 期来新帧：从衰减态续积分、无跳变；
  ⑥ /clock 前跳注入（强制大 dt）：单步位移 ≤ 夹板×dt_cap；
  ⑦ §3.4 复现判定：bridge dt=0.083~0.084 + 收包 0.052 → Δv=0.125「超限观感」
     对自身 dt 成立 → 判 sim 时钟伪影非 bug。

运行：python3 test/test_p6_limiter.py
"""
import math
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', 'scripts')


# ---------------------------------------------------------------- 桩模块 ----
class _Vec:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()


class _TwistStamped:  # geometry_msgs 里仅类型名被引用
    def __init__(self):
        self.twist = _Twist()
        self.header = types.SimpleNamespace(stamp=None, frame_id='')


class _Odometry:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.child_frame_id = ''
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=_Vec(),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ), covariance=[0.0] * 36)
        self.twist = types.SimpleNamespace(twist=_Twist(), covariance=[0.0] * 36)


class _TransformStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.child_frame_id = ''
        self.transform = types.SimpleNamespace(
            translation=_Vec(),
            rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))


class _Pub:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns


class _Clock:
    """可拨动的假 ROS clock（真实节点经 use_sim_time 走 /clock）。"""

    def __init__(self):
        self.now_sec = 0.0

    def now(self):
        return _Time(int(round(self.now_sec * 1e9)))


# launch 级参数覆盖：make_bridge(**kw) 前设置，declare_parameter 时消费
OVERRIDES = {}


class _NodeBase:
    """rclpy.node.Node 最小桩：参数表 + 发布/订阅/定时器登记 + 假时钟。"""

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._clock = _Clock()
        self._timers = []      # (period, cb)
        self._subs = []        # (topic, cb)
        self._pubs = {}
        self._logs = []

    def declare_parameter(self, name, value):
        self._params[name] = OVERRIDES.pop(name, value)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        return self._pubs.setdefault(topic, _Pub(topic))

    def create_subscription(self, msg_type, topic, cb, qos):
        self._subs.append((topic, cb))
        return None

    def create_timer(self, period, cb):
        self._timers.append((period, cb))
        return None

    def get_logger(self):
        return self

    def info(self, m):
        self._logs.append(str(m))

    def warn(self, m):
        self._logs.append(str(m))

    def get_clock(self):
        return self._clock


def _install_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _NodeBase
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = lambda **k: types.SimpleNamespace(**k)
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    qos_mod.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2)
    rclpy.node = node_mod
    rclpy.qos = qos_mod

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = _Odometry
    nav_msgs.msg = nav_msgs_msg

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_msg.Twist = _Twist
    geometry_msgs_msg.TwistStamped = _TwistStamped
    geometry_msgs_msg.TransformStamped = _TransformStamped
    geometry_msgs.msg = geometry_msgs_msg

    tf2_ros = types.ModuleType('tf2_ros')

    class _TB:
        def __init__(self, node):
            self.node = node

        def sendTransform(self, t):
            pass

    tf2_ros.TransformBroadcaster = _TB

    for name, mod in [('rclpy', rclpy), ('rclpy.node', node_mod),
                      ('rclpy.qos', qos_mod), ('nav_msgs', nav_msgs),
                      ('nav_msgs.msg', nav_msgs_msg),
                      ('geometry_msgs', geometry_msgs),
                      ('geometry_msgs.msg', geometry_msgs_msg),
                      ('tf2_ros', tf2_ros)]:
        sys.modules[name] = mod


_install_stubs()
sys.path.insert(0, SCRIPTS)
import odom_bridge  # noqa: E402

SILENCE_MARGIN = odom_bridge.SILENCE_MARGIN
SILENCE_DT_CAP = odom_bridge.SILENCE_DT_CAP
EPS = 1e-9


def make_bridge(**overrides):
    """构造真实 OdomBridge（默认 sim 参数 max_decel=0.6 / T=0.5）。"""
    OVERRIDES.clear()
    OVERRIDES.update(overrides)
    return odom_bridge.OdomBridge()


def drive_cb(node, t, vx=0.0, vy=0.0, wz=0.0):
    """拨时钟到 t 并投一帧 /cmd_vel，返回限幅器输出。"""
    node._clock.now_sec = t
    msg = _Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
    node._cmd_vel_relay_cb(msg)
    return node._last_published


def tick(node, t):
    """拨时钟到 t 并触发沉默定时器一次（等价 spin 处理一次 timer 事件）。"""
    node._clock.now_sec = t
    node._timers[0][1]()


def pubs_since(pub, n0):
    return [(m.linear.x, m.linear.y, m.angular.z) for m in pub.msgs[n0:]]


# ================================================================ 断言 ①②⑦ ==
class TestLimiterPure(unittest.TestCase):
    L = odom_bridge.VelocityRateLimiter

    def test_1_asymmetric_clamps(self):
        """① 三把夹板各自成立；缺省回落对称（旧行为等价）。"""
        lim = self.L(1.5, 0.5, 2.0, max_decel=0.6, max_decel_wz=None)
        self.assertAlmostEqual(lim.max_decel, 0.6)
        self.assertAlmostEqual(lim.max_decel_wz, 2.0)  # None → =max_az
        # 加速向：0→1.0 @0.05 raw 20 → 夹 1.5
        lim.limit(0.0, 0.0, 0.0, 0.0)
        v = lim.limit(1.0, 0.0, 0.0, 0.05)
        self.assertAlmostEqual(v[0], 0.075, places=9)
        # 减速向：0.075→0 raw -1.5 → 夹 -0.6（对称 1.5 时该步不夹）
        v = lim.limit(0.0, 0.0, 0.0, 0.10)
        self.assertAlmostEqual(v[0], 0.045, places=9)
        # wz 加速向夹 2.0
        lim2 = self.L(1.5, 0.5, 2.0, max_decel=0.6)
        lim2.limit(0.0, 0.0, 0.4, 0.0)
        v = lim2.limit(0.0, 0.0, 4.0, 0.05)
        self.assertAlmostEqual(v[2], 0.5, places=9)
        # wz 衰减夹板 -2.0（而非 -0.6）
        v = lim2.limit(0.0, 0.0, 0.0, 0.10)
        self.assertAlmostEqual(v[2], 0.4, places=9)
        # 缺省全对称 = 旧行为
        old = self.L()
        self.assertAlmostEqual(old.max_decel, old.max_ax)
        self.assertAlmostEqual(old.max_decel_wz, old.max_az)

    def test_1_sustained_rate_bound(self):
        """① 任意序列下单步速率 ≤ 夹板×dt（含随机目标序列）。"""
        lim = self.L(1.5, 0.5, 2.0, max_decel=0.6, max_decel_wz=1.2)
        lim.limit(0.0, 0.0, 0.0, 0.0)
        t = 0.0
        tgts = [(1.2, 0.3, 0.8), (0.0, 0.0, 0.0), (-0.25, 0.0, -0.6),
                (0.5, 0.1, 1.5), (0.0, 0.0, 0.4)] * 20
        for vx, vy, wz in tgts:
            dt = 0.05
            t += dt
            px, py, pz = lim._last_vx, lim._last_vy, lim._last_wz
            vx2, vy2, wz2 = lim.limit(vx, vy, wz, t)
            # 方向感知断言：加速向夹 max_ax/max_az、减速向夹 max_decel/max_decel_wz
            if vx2 >= px:
                self.assertLessEqual(vx2 - px, 1.5 * dt + EPS)
            else:
                self.assertLessEqual(px - vx2, 0.6 * dt + EPS)
            self.assertLessEqual(abs(vy2 - py), 0.5 * dt + EPS)  # vy 对称
            if wz2 >= pz:
                self.assertLessEqual(wz2 - pz, 2.0 * dt + EPS)
            else:
                self.assertLessEqual(pz - wz2, 1.2 * dt + EPS)
        # 向零收敛单调不过冲
        lim2 = self.L(1.5, 0.5, 2.0, max_decel=0.6)
        lim2.limit(0.0, 0.0, 0.0, 0.0)
        lim2.limit(0.3, 0.0, 0.0, 0.05)
        prev = 0.3
        t = 0.1
        for _ in range(20):
            v = lim2.limit(0.0, 0.0, 0.0, t)[0]
            self.assertLessEqual(v, prev + EPS)
            self.assertGreaterEqual(v, -EPS)  # 不冲到负
            prev = v
            t += 0.05
        self.assertAlmostEqual(prev, 0.0, places=9)

    def test_2_first_frame_and_dt_guard(self):
        """② 首帧直通；dt≤0 返回上一帧结果不动。"""
        lim = self.L(1.5, 0.5, 2.0, max_decel=0.6)
        v = lim.limit(0.9, 0.4, 3.0, 5.0)  # 首帧直通（wz 也透传）
        self.assertEqual(v, (0.9, 0.4, 3.0))
        self.assertEqual(lim._tgt, (0.9, 0.4, 3.0))
        v = lim.limit(0.0, 0.0, 0.0, 5.0)  # dt=0 → 不动
        self.assertEqual(v, (0.9, 0.4, 3.0))
        self.assertAlmostEqual(lim._last_time, 5.0)
        v = lim.limit(0.0, 0.0, 0.0, 4.9)  # dt<0 → 同守卫
        self.assertEqual(v, (0.9, 0.4, 3.0))

    def test_7_overlimit_is_sim_clock_artifact(self):
        """⑦ §3.4 复现：Δv=0.125 在收包间隙 52ms 下观感 2.40 超限；
        对 bridge 自身 dt≥83ms 则夹板成立 → sim 时钟伪影非 bug。"""
        lim = self.L(1.5, 0.5, 2.0, max_decel=1.5)  # 对称 1.5（P6a 前口径）
        lim.limit(0.294, 0.0, 0.0, 0.0)
        # bag 收包间隙口径（52ms）下表观 ax = -2.40，名义夹板外
        self.assertGreater(abs((0.169 - 0.294) / 0.052), 1.5)
        # bridge 自身 dt 口径（83ms，sim /clock 前跳实证 ≥31ms 偏差）下夹板成立
        v = lim.limit(0.169, 0.0, 0.0, 0.0833)
        self.assertAlmostEqual(v[0], 0.169, delta=6e-4)  # (0.294-1.5*0.0833)=0.16905
        # 若 dt 真为 52ms，同目标会被夹到 0.216 而非 0.169 —— 两 dt 源 diverge
        lim2 = self.L(1.5, 0.5, 2.0, max_decel=1.5)
        lim2.limit(0.294, 0.0, 0.0, 0.0)
        v2 = lim2.limit(0.169, 0.0, 0.0, 0.052)
        self.assertAlmostEqual(v2[0], 0.294 - 1.5 * 0.052, places=9)
        self.assertGreater(v2[0], 0.169)  # 夹板使衰减慢于 bag 观感


# ==================================================== 断言 ③④⑤⑥（端到端）==
class TestSilenceTakeover(unittest.TestCase):
    def _bridge(self):
        return make_bridge(max_decel=0.6, max_decel_wz=0.0,
                           cmd_silence_timeout_s=0.5)

    def test_3_fast_lane_zero_target(self):
        """③ 沉默+末目标≈0（取消形状）：快车道不等 T，0.5s 积分内归零；
        补 2 帧零后停发；发布只出现在值变化 tick。"""
        n = self._bridge()
        pub = n._pubs['/cmd_vel_limiter']
        drive_cb(n, 1.00, 0.3)          # 巡航帧（首帧直通）
        drive_cb(n, 1.05, 0.0)          # 取消零帧 → 限幅只吃到 -0.6 → 0.27
        self.assertAlmostEqual(n._last_published[0], 0.27, places=9)
        self.assertEqual(n._rate_limiter._tgt, (0.0, 0.0, 0.0))
        n0 = len(pub.msgs)

        tick(n, 1.10)                   # margin 内：零发布
        self.assertEqual(pubs_since(pub, n0), [])
        tick(n, 1.15)                   # 首个定时器步：dt=0.1（上限）
        ramp = pubs_since(pub, n0)
        self.assertAlmostEqual(ramp[0][0], 0.21, places=9)

        # 逐 tick 衰减至零；全 tick 值变化才发布
        t = 1.20
        prev = 0.21
        while prev > 0.01:
            tick(n, t)
            r = pubs_since(pub, n0)
            self.assertTrue(r and r[-1][0] != prev, '值未变不应发布')
            prev = r[-1][0]
            t += 0.05
        # 收敛 tick（循环末步）补 2 帧零（0.00 本身计入这 2 帧）；随后静默 tick 停发
        n1 = len(pub.msgs)
        tick(n, t)
        tick(n, t + 0.05)
        tick(n, t + 0.10)
        self.assertEqual(pubs_since(pub, n1), [])
        tail = pubs_since(pub, n0)
        self.assertEqual(tail[-2:], [(0.0, 0.0, 0.0)] * 2)
        # 总时长上界：margin 0.1 + 衰减 0.5（0.27 起点 < 0.27/0.6=0.45）
        self.assertLessEqual(t + 0.05 - 1.05, 0.1 + 0.27 / 0.6 + EPS)

    def test_4_crash_timeout_lane(self):
        """④ 沉默+末目标非零（崩溃形状）：T 前零发布；T 后 ramp 距离预算内归零。"""
        n = self._bridge()
        pub = n._pubs['/cmd_vel_limiter']
        drive_cb(n, 10.00, 0.3)         # 巡航全速帧后上游崩溃（首帧直通）
        n0 = len(pub.msgs)
        for t in (10.15, 10.30, 10.45):
            tick(n, t)
            self.assertEqual(pubs_since(pub, n0), [], 'T 之前必须保沉默')
        tick(n, 10.50)                  # T 到 → 强制归零 ramp，dt 封顶 0.1
        ramp = pubs_since(pub, n0)
        self.assertAlmostEqual(ramp[0][0], 0.24, places=9)
        self.assertEqual(n._rate_limiter._tgt, (0.0, 0.0, 0.0))
        # ramp 距离（梯形上界）+ 盲走距离 ≤ 0.25m 预算：0.5×0.3 + 0.3²/(2×0.6) = 0.225
        t = 10.55
        dist = 0.24 * 0.05
        prev = 0.24
        while prev > 0.01:
            tick(n, t)
            r = pubs_since(pub, n0)
            cur = r[-1][0] if r else prev
            dist += (prev + cur) / 2 * 0.05
            prev = cur
            t += 0.05
        tick(n, t)
        tail = pubs_since(pub, n0)
        self.assertEqual(tail[-2:], [(0.0, 0.0, 0.0)] * 2)
        blind = 0.3 * 0.5
        self.assertLessEqual(blind + dist + 0.3 * 0.05, 0.25 + 1e-3,
                             '盲走 + ramp 距离超出 0.25m 预算')
        self.assertAlmostEqual(prev, 0.0, places=9)

    def test_5_ramp_resume_no_jump(self):
        """⑤ ramp 期来新帧：从衰减态续积分、无跳变（加速夹板接管）；
        新目标非零再沉默 → 回超时车道（设计语义）。"""
        n = self._bridge()
        pub = n._pubs['/cmd_vel_limiter']
        drive_cb(n, 10.00, 0.3)
        n0 = len(pub.msgs)
        tick(n, 10.50)                  # 崩溃 ramp 起点 0.24
        tick(n, 10.55)                  # 0.21
        tick(n, 10.60)                  # 0.18
        self.assertAlmostEqual(n._last_published[0], 0.18, places=9)
        # 新帧 0.25 @ 10.63：cb 路径从衰减态续积分（gap dt=0.03）
        out = drive_cb(n, 10.63, 0.25)
        self.assertAlmostEqual(out[0], 0.18 + min(1.5, (0.25 - 0.18) / 0.03) * 0.03,
                               places=9)
        # 无跳变：单步 ≤ 加速夹板×dt
        self.assertLessEqual(out[0] - 0.18, 1.5 * 0.03 + EPS)
        self.assertLess(out[0], 0.25)   # 未直接跳到目标
        # 再沉默：末目标非零 → 回超时车道（0.5s 保沉默后 ramp）
        n0b = len(pub.msgs)
        for t in (10.70, 10.90, 11.10):
            tick(n, t)
            self.assertEqual(pubs_since(pub, n0b), [])
        tick(n, 11.13 + 0.02)
        self.assertTrue(pubs_since(pub, n0b))

    def test_6_clock_jump_capped(self):
        """⑥ /clock 前跳注入：单步衰减 ≤ 夹板×dt_cap；虚假超时最坏=温柔自停。"""
        n = self._bridge()
        pub = n._pubs['/cmd_vel_limiter']
        drive_cb(n, 20.00, 0.3)
        drive_cb(n, 20.05, 0.0)         # 末目标≈0 → 快车道
        n0 = len(pub.msgs)
        n._clock.now_sec = 28.0         # /clock 前跳 7.95s
        tick(n, 28.0)
        r = pubs_since(pub, n0)
        self.assertAlmostEqual(r[0][0], 0.27 - 0.6 * SILENCE_DT_CAP, places=9)
        self.assertLessEqual(0.27 - r[0][0], 0.6 * SILENCE_DT_CAP + EPS)
        # 非零末目标 + 前跳跨过 T：温柔自停（一次 ramp），不发跳变
        n2 = self._bridge()
        pub2 = n2._pubs['/cmd_vel_limiter']
        drive_cb(n2, 20.00, 0.3)        # 崩溃形状
        n20 = len(pub2.msgs)
        n2._clock.now_sec = 40.0        # 前跳 20s，跨过 T
        tick(n2, 40.0)
        r2 = pubs_since(pub2, n20)
        self.assertAlmostEqual(r2[0][0], 0.3 - 0.6 * SILENCE_DT_CAP, places=9)
        self.assertLessEqual(0.3 - r2[0][0], 0.6 * SILENCE_DT_CAP + EPS)

    def test_invert_position_ramp_chassis_frame(self):
        """invert 在入口（限幅前）→ 沉默 ramp 全程底盘口径、出口不再反号。"""
        n = make_bridge(max_decel=0.6, cmd_silence_timeout_s=0.5,
                        invert_cmd_vel_yz=True)
        pub = n._pubs['/cmd_vel_limiter']
        drive_cb(n, 1.00, 0.3, 0.2, 0.4)   # nav2 口径 → 入口取负到底盘
        self.assertAlmostEqual(n._last_published[1], -0.2, places=9)
        self.assertAlmostEqual(n._last_published[2], -0.4, places=9)
        n0 = len(pub.msgs)
        tick(n, 1.15)
        self.assertEqual(pubs_since(pub, n0), [])
        tick(n, 1.45)                   # 0.45s < T=0.5 → 超时车道保沉默
        self.assertEqual(pubs_since(pub, n0), [])
        tick(n, 1.50)                   # T 到 → ramp，dt 封顶 0.1，vy/wz 保持底盘负号
        r = pubs_since(pub, n0)
        self.assertAlmostEqual(r[0][0], 0.3 - 0.6 * SILENCE_DT_CAP, places=9)
        self.assertAlmostEqual(r[0][1], -0.2 + 0.5 * SILENCE_DT_CAP, places=9)
        self.assertLess(r[0][1], 0.0)   # 未被出口二次反号
        self.assertAlmostEqual(r[0][2], -0.4 + 2.0 * SILENCE_DT_CAP, places=9)
        self.assertLess(r[0][2], 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
