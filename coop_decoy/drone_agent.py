"""红枫2026 赛题2 coop_decoy —— 无人机智能体主入口。

结构说明：
    本类继承竞赛平台提供的 Agent 基类，按平台生命周期工作：
        __init__  -> 构造（每个可控实体一次）
        reset     -> 每局开始前重置
        sensor    -> 每帧感知回调（视觉检测 + 通信接收）
        decide    -> 每个决策周期输出飞行动作（~10 Hz）

    四大功能模块全部从各自包的 main.py 相对导入，接口固定：
        VisionDetect.detect(obs)            -> 目标列表
        CommHandle.receive()/broadcast()    -> 队友消息收发
        SchedulerBrain.decide(t, m)         -> 调度指令 cmd
        MotionCtrl.hover()/get_action(cmd)  -> 飞行动作

阶段说明：
    阶段1（当前）：MULTI_DRONE = False，仅 0 号无人机执行
        "感知 -> 调度 -> 控制 -> 广播" 完整闭环，其余无人机悬停。
    阶段2（预留）：MULTI_DRONE = True 时启用多机协同逻辑，
        扩展位置见 decide() 中的 TODO 标记。
"""
from __future__ import annotations

import logging
import os as _os
import sys
import traceback
from typing import Any, List, Optional

# ── 日志（必须在所有可能失败的 import 之前初始化）──
_log_dir = _os.path.dirname(_os.path.abspath(__file__))
_log_path = _os.path.join(_log_dir, "agent_run.log")

logger = logging.getLogger("coop_decoy.DroneAgent")
logger.setLevel(logging.DEBUG)
if not logger.handlers:
    _fmt = logging.Formatter(
        "[DroneAgent] %(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    _fh = logging.FileHandler(_log_path, mode="a", encoding="utf-8")
    _fh.setFormatter(_fmt)
    logger.addHandler(_fh)
    _sh = logging.StreamHandler(sys.stderr)
    _sh.setFormatter(_fmt)
    logger.addHandler(_sh)

# ── 业务模块导入（每个单独 try/except，失败时写日志不炸）──
from .agent import Agent

# SDK 命令（report_target 等）
try:
    from competition.sdk.core.commands import report_target
except ImportError:
    try:
        from sdk.core.commands import report_target
    except ImportError:
        report_target = None  # type: ignore

_MultiSimClient = None
try:
    from .search_track.multi_client import MultiSimClient as _MSC
    _MultiSimClient = _MSC
except Exception:
    logger.exception("FAILED to import MultiSimClient")

_VisionDetect = None
try:
    from .search_track.vision_detect.main import VisionDetect as _VD
    _VisionDetect = _VD
except Exception:
    logger.exception("FAILED to import VisionDetect")

_MotionCtrl = None
try:
    from .search_track.motion_control.main import MotionCtrl as _MC
    _MotionCtrl = _MC
except Exception:
    logger.exception("FAILED to import MotionCtrl")

_CommHandle = None
try:
    from .search_track.communication.main import CommHandle as _CH
    _CommHandle = _CH
except Exception:
    logger.exception("FAILED to import CommHandle")

_SchedulerBrain = None
try:
    from .search_track.cluster_scheduler.main import SchedulerBrain as _SB
    _SchedulerBrain = _SB
except Exception:
    logger.exception("FAILED to import SchedulerBrain")

_DecoyClassifier = None
try:
    from .search_track.vision_detect.decoy_classifier import DecoyClassifier as _DC
    _DecoyClassifier = _DC
except Exception:
    logger.exception("FAILED to import DecoyClassifier")
    logger.addHandler(_sh)
logger.info("==== session start ====")


class DroneAgent(Agent):
    """coop_decoy 赛题的无人机智能体。

    职责划分：
        - 本类只做"编排"：串起 感知 -> 调度 -> 控制 -> 通信 的流水线；
        - 具体算法全部下沉到四大模块内部，本类不实现任何算法细节。
    """

    def __init__(self, my_uid: str):
        """构造智能体：保存本机 UID 并实例化四大模块。

        Args:
            my_uid: 本机唯一标识（由平台 runner 注入）。
        """
        super().__init__(my_uid)
        self.my_uid = my_uid

        # ---- 阶段开关 ----
        # False：阶段1，单机闭环（仅 0 号机决策，其余悬停）；
        # True ：阶段2，多机协同（预留，见 decide() 中扩展位置）。
        self.MULTI_DRONE = False

        # ---- 底层工具 ----
        self.sim_client = None
        if _MultiSimClient is not None:
            try:
                self.sim_client = _MultiSimClient()
                logger.debug("[uid=%s] MultiSimClient created OK", my_uid)
            except Exception:
                logger.exception("[uid=%s] MultiSimClient creation FAILED", my_uid)
        else:
            logger.warning("[uid=%s] MultiSimClient not available (import failed)", my_uid)

        self.vision = None
        if _VisionDetect is not None:
            try:
                self.vision = _VisionDetect()
                logger.debug("[uid=%s] VisionDetect created OK", my_uid)
            except Exception:
                logger.exception("[uid=%s] VisionDetect creation FAILED", my_uid)
        else:
            logger.warning("[uid=%s] VisionDetect not available", my_uid)

        self.motion = None
        if _MotionCtrl is not None:
            try:
                self.motion = _MotionCtrl(str(my_uid))
                logger.debug("[uid=%s] MotionCtrl created OK", my_uid)
            except Exception:
                logger.exception("[uid=%s] MotionCtrl creation FAILED", my_uid)
        else:
            logger.warning("[uid=%s] MotionCtrl not available", my_uid)

        self.comm = None
        if _CommHandle is not None:
            try:
                self.comm = _CommHandle(my_uid)
                logger.debug("[uid=%s] CommHandle created OK", my_uid)
            except Exception:
                logger.exception("[uid=%s] CommHandle creation FAILED", my_uid)
        else:
            logger.warning("[uid=%s] CommHandle not available", my_uid)

        self.scheduler = None
        if _SchedulerBrain is not None:
            try:
                self.scheduler = _SchedulerBrain()
                logger.debug("[uid=%s] SchedulerBrain created OK", my_uid)
            except Exception:
                logger.exception("[uid=%s] SchedulerBrain creation FAILED", my_uid)
        else:
            logger.warning("[uid=%s] SchedulerBrain not available", my_uid)

        # ---- 感知缓存（sensor 写入，decide 读取）----
        self.target_list: List[Any] = []        # 视觉检测到的目标列表
        self.recv_msg: Optional[Any] = None     # 收到的队友消息

        # ---- decide 调用计数器（用于日志节流）----
        self._decide_call_count = 0

        # ---- 阶段1：状态机 + 诱饵判别 ----
        self._state: str = "SEARCH"
        self._classifier = None
        if _DecoyClassifier is not None:
            try:
                self._classifier = _DecoyClassifier()
                logger.debug("[uid=%s] DecoyClassifier created OK", my_uid)
            except Exception:
                logger.exception("[uid=%s] DecoyClassifier creation FAILED", my_uid)
        self._verify_t: float = 0.0
        self._track_t: float = 0.0
        self._last_report_t: float = -1e9
        self._sim_t: float = 0.0

        logger.info("[uid=%s] DroneAgent.__init__ completed successfully", my_uid)

    def reset(self) -> None:
        """每局仿真开始前调用：清零本类与四大模块的全部状态。"""
        logger.info("[uid=%s] reset() called", self.my_uid)
        try:
            # 清空本类感知缓存
            self.target_list = []
            self.recv_msg = None
            self._decide_call_count = 0
            # 状态机重置
            self._state = "SEARCH"
            self._verify_t = 0.0
            self._track_t = 0.0
            self._last_report_t = -1e9
            self._sim_t = 0.0
            if self._classifier is not None:
                self._classifier.reset()

            # 各模块状态清零（模块内部实现 reset，做滤波器/缓存/计时器等复位）
            for module in (self.vision, self.motion, self.comm, self.scheduler):
                if module is None:
                    continue
                try:
                    reset_fn = getattr(module, "reset", None)
                    if callable(reset_fn):
                        reset_fn()
                except Exception:
                    logger.error("[uid=%s] reset() FAILED on module %s:\n%s",
                                 self.my_uid, type(module).__name__, traceback.format_exc())
                    # 不 raise，继续重置其他模块
            logger.info("[uid=%s] reset() completed", self.my_uid)
        except Exception:
            logger.error("[uid=%s] reset() FATAL:\n%s", self.my_uid, traceback.format_exc())
            raise

    def sensor(self, obs, dt: float):
        """感知回调：执行视觉检测与通信接收，结果存入实例成员变量。

        Args:
            obs: 本帧观测（平台注入）。
            dt: 距上次调用的时间间隔（秒）。

        Returns:
            视觉检测出的目标列表（同时缓存到 self.target_list）。
        """
        try:
            # 1) 视觉检测：obs -> 目标列表
            if self.vision is not None:
                self.target_list = self.vision.detect(obs)
            else:
                self.target_list = []

            # 2) 通信接收：拉取队友广播的消息
            inbox = getattr(obs, "comm_inbox", None) or []
            if self.comm is not None:
                self.recv_msg = self.comm.receive(inbox)
            else:
                self.recv_msg = None

            return self.target_list
        except Exception:
            logger.error("[uid=%s] sensor() FAILED:\n%s", self.my_uid, traceback.format_exc())
            # 降级：返回空列表，让 decide() 至少不会崩溃
            self.target_list = []
            self.recv_msg = None
            return []

    def decide(self, obs, dt: float) -> List[Any]:
        self._decide_call_count += 1
        call_id = self._decide_call_count
        self._sim_t += dt
        _log_detail = (call_id <= 5) or (call_id % 100 == 0)

        try:
            sv = obs.self
            lat, lon, alt, yaw = sv.lat, sv.lon, sv.alt, sv.heading_deg
            det = sv.detection

            # 前 5 帧 + 每 300 帧打一次状态
            if call_id <= 5 or call_id % 300 == 0:
                logger.info("[uid=%s] decide#%d state=%s det=%s dt=%.2f",
                            self.my_uid, call_id, self._state,
                            "yes" if det.detected else "no", dt)

            # 非 uav_alpha → 搜索（代替悬停，避免 motion state 未设置）
            if str(self.my_uid) != "20001":
                return self.motion.get_action({
                    "mode": "search",
                    "lat": lat, "lon": lon, "alt": alt, "yaw": yaw,
                })

            # ── uav_alpha 状态机 ──
            if self._state == "SEARCH":
                return self._do_search(lat, lon, alt, yaw, det, call_id, _log_detail)

            if self._state == "VERIFY":
                return self._do_verify(lat, lon, alt, yaw, det, dt, call_id, _log_detail)

            if self._state == "TRACK":
                return self._do_track(lat, lon, alt, yaw, det, dt, call_id, _log_detail)

            return self._make_search_cmd(lat, lon, alt, yaw)

        except Exception:
            logger.exception("[uid=%s] decide#%d CRASH", self.my_uid, call_id)
            return []

    # ── 状态机各状态 ──

    def _make_search_cmd(self, lat, lon, alt, yaw):
        return self.motion.get_action({
            "mode": "search", "lat": lat, "lon": lon, "alt": alt, "yaw": yaw,
        })

    def _make_track_cmd(self, lat, lon, alt, yaw, tgt):
        return self.motion.get_action({
            "mode": "track", "target": tgt,
            "lat": lat, "lon": lon, "alt": alt, "yaw": yaw,
        })

    def _do_search(self, lat, lon, alt, yaw, det, call_id, log_detail):
        if det.detected and det.target_lat is not None:
            # 发现目标 → 进入验证
            self._state = "VERIFY"
            self._verify_t = 0.0
            if self._classifier is not None:
                self._classifier.reset()
            logger.info("[uid=%s] SEARCH→VERIFY target=(%.5f,%.5f) call=%d",
                        self.my_uid, det.target_lat, det.target_lon, call_id)
            return self._make_track_cmd(lat, lon, alt, yaw,
                                        (det.target_lat, det.target_lon))
        return self._make_search_cmd(lat, lon, alt, yaw)

    def _do_verify(self, lat, lon, alt, yaw, det, dt, call_id, log_detail):
        self._verify_t += dt

        # 更新分类器
        if det.detected and det.target_lat is not None:
            if self._classifier is not None:
                self._classifier.update(det.target_lat, det.target_lon, dt)
            tgt = (det.target_lat, det.target_lon)
        else:
            # 丢失 → 回搜索
            self._state = "SEARCH"
            return self._make_search_cmd(lat, lon, alt, yaw)

        # 判断
        if self._classifier is not None and self._verify_t >= 3.0:
            if self._classifier.is_real_target():
                self._state = "TRACK"
                self._track_t = 0.0
                self._last_report_t = -1e9
                logger.info("[uid=%s] VERIFY→TRACK real target call=%d", self.my_uid, call_id)
                return self._make_track_cmd(lat, lon, alt, yaw, tgt)
            else:
                # 诱饵 → 放弃
                self._state = "SEARCH"
                logger.info("[uid=%s] VERIFY→SEARCH decoy call=%d", self.my_uid, call_id)
                return self._make_search_cmd(lat, lon, alt, yaw)

        # 超时
        if self._verify_t >= 8.0:
            self._state = "SEARCH"
            return self._make_search_cmd(lat, lon, alt, yaw)

        # 继续验证，同时跟踪靠近目标
        return self._make_track_cmd(lat, lon, alt, yaw, tgt)

    def _do_track(self, lat, lon, alt, yaw, det, dt, call_id, log_detail):
        self._track_t += dt
        cmds = []

        if det.detected and det.target_lat is not None:
            tgt = (det.target_lat, det.target_lon)
            # 上报目标（每 1s 一次）
            if report_target is not None and self._sim_t - self._last_report_t >= 1.0:
                self._last_report_t = self._sim_t
                if self._classifier is not None:
                    rpt = self._classifier.get_report_position()
                    if rpt is not None:
                        cmds.append(report_target(rpt[0], rpt[1]))
                    else:
                        cmds.append(report_target(tgt[0], tgt[1]))
        else:
            # 丢失 → 回搜索
            self._state = "SEARCH"
            return self._make_search_cmd(lat, lon, alt, yaw)

        # dwell 满 20s → 释放目标回搜索
        if self._track_t >= 20.0:
            self._state = "SEARCH"
            logger.info("[uid=%s] TRACK→SEARCH dwell=%.1fs call=%d", self.my_uid, self._track_t, call_id)
            return self._make_search_cmd(lat, lon, alt, yaw)

        cmds.append(self._make_track_cmd(lat, lon, alt, yaw, tgt))
        return cmds
