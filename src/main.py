#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
  HF2026_UAV_Challenge2 — 无人机集群弱对抗跟踪竞赛
  顶层主调度框架 (main.py)
===============================================================================

  【双模式架构】
    本文件支持两种运行模式，共享同一份决策核心逻辑：

    1. 赛事平台模式（正式比赛）：
         平台通过 src.main:Agent 加载，每帧调用 Agent.decide(obs)
         → obs 数据由平台注入，Agent 不直接调用模块 API

    2. 独立测试模式（本地仿真）：
         python main.py
         → MainLoop 自行管理模块生命周期 + 主循环
         → 主动调用各模块 API 获取数据

  【模块调用关系】（严格遵循 docs/INTERFACE.md 接口契约）
      camera 帧 ──▶ vision_detect.detect() ──▶ VisualObservation
                                                      │
      IMU/GPS ──▶ motion_control.get_drone_state() ──▶ DroneState
                                                      │
                                                      ▼
                     cluster_scheduler.update(self_state, observations,
                                              teammate_states, team_reports)
                                                      │
                                                      ▼
                                              MotionCommand
                                                      │
                                                      ▼
                     motion_control.execute_command(cmd)

  【依赖】
      Python >= 3.9
      PyYAML, numpy

  【运行方式】
      # 独立测试模式（在项目根目录执行）
      python src/main.py

      # 赛事平台模式（平台内部调用，无需手动执行）
      # python -m competition run --scenario coop_decoy --agent src.main:Agent

  【作者】HF2026_UAV_Challenge2 团队
  【版本】v1.1
  【日期】2026-07-28
===============================================================================
"""

import os
import sys
import time
import signal
import logging
import traceback
from typing import Optional, Dict, Any, List, Tuple

import yaml
import numpy as np

# ============================================================================
# 项目根路径推导（适配 HF2026_UAV_Challenge2 目录结构）
# ============================================================================
# main.py 位于 src/ 子目录: HF2026_UAV_Challenge2/src/main.py
# 项目根目录 = 向上一级
PROJECT_ROOT: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# ============================================================================
# 子模块导入（路径: src/<module>/__init__.py 或 src/<module>/<module>.py）
# ============================================================================

# ---- communication 通信模块 ----
from src.communication import (  # type: ignore[import-untyped]
    init_comm,
    broadcast_drone_state,
    send_target_report,
    receive_drone_states,
    receive_target_reports,
    register_callback,
    shutdown_comm,
    CommConfig,
)

# ---- motion_control 运动控制模块 ----
from src.motion_control import (  # type: ignore[import-untyped]
    init_motion,
    execute_command,
    get_drone_state,
    cancel_command,
    emergency_stop,
    get_battery,
    shutdown_motion,
    MotionConfig,
)

# ---- cluster_scheduler 集群调度模块 ----
from src.cluster_scheduler import (  # type: ignore[import-untyped]
    init_scheduler,
    update as scheduler_update,
    set_strategy,
    get_statistics,
    shutdown_scheduler,
    SchedulerConfig,
    Strategy,
)

# ---- vision_detect 视觉感知模块 ----
from src.vision_detect import (  # type: ignore[import-untyped]
    init_vision,
    detect,
    reset_tracker,
    get_fps as vision_get_fps,
    shutdown_vision,
    VisionConfig,
)

# ---- 全局数据类型（与接口契约一致） ----
from src.common.types import (  # type: ignore[import-untyped]
    DroneState,
    DroneStatus,
    VisualObservation,
    Detection,
    MotionCommand,
    CommandType,
    TargetReport,
)

# ---- 赛事平台基类（条件导入，独立测试模式下不可用） ----
try:
    from competition.agent import CoopAgent  # type: ignore[import-untyped]
    _HAS_PLATFORM = True
except ImportError:
    CoopAgent = object  # fallback，便于 Agent 类定义
    _HAS_PLATFORM = False

# ============================================================================
# 全局常量
# ============================================================================

DEFAULT_CONFIG_PATH: str = os.path.join(PROJECT_ROOT, "config", "algorithm.yaml")
DEFAULT_LOG_FORMAT: str = (
    "%(asctime)s [%(levelname)-8s] %(name)-20s | %(message)s"
)
DEFAULT_LOG_DATE_FORMAT: str = "%Y-%m-%d %H:%M:%S"
DEFAULT_MAIN_LOOP_HZ: int = 25  # 默认主循环频率 (Hz)，与 INTERFACE.md 建议一致


# ============================================================================
# 日志配置
# ============================================================================

def setup_logging(log_config: Dict[str, Any]) -> logging.Logger:
    """
    初始化日志系统。

    Args:
        log_config: 来自 algorithm.yaml 的 logging 配置节，
                    可包含 level, file, format 等字段。

    Returns:
        根 logger 实例。
    """
    log_level_str: str = log_config.get("level", "INFO").upper()
    log_level: int = getattr(logging, log_level_str, logging.INFO)
    log_file: Optional[str] = log_config.get("file", None)

    # 控制台 handler
    console_handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(
        logging.Formatter(DEFAULT_LOG_FORMAT, datefmt=DEFAULT_LOG_DATE_FORMAT)
    )

    # 组装根 logger
    root_logger: logging.Logger = logging.getLogger()
    root_logger.setLevel(log_level)
    root_logger.handlers.clear()
    root_logger.addHandler(console_handler)

    # 可选的日志文件输出
    if log_file:
        log_file_abs: str = (
            log_file if os.path.isabs(log_file)
            else os.path.join(PROJECT_ROOT, log_file)
        )
        os.makedirs(os.path.dirname(log_file_abs), exist_ok=True)
        file_handler: logging.FileHandler = logging.FileHandler(
            log_file_abs, encoding="utf-8"
        )
        file_handler.setLevel(log_level)
        file_handler.setFormatter(
            logging.Formatter(DEFAULT_LOG_FORMAT, datefmt=DEFAULT_LOG_DATE_FORMAT)
        )
        root_logger.addHandler(file_handler)

    return root_logger


# ============================================================================
# 配置加载
# ============================================================================

def load_config(config_path: str) -> Dict[str, Any]:
    """
    加载 algorithm.yaml 算法参数配置文件。

    Args:
        config_path: YAML 配置文件的绝对或相对路径。

    Returns:
        解析后的配置字典。

    Raises:
        FileNotFoundError: 配置文件不存在。
        yaml.YAMLError: YAML 格式错误。
    """
    config_path = os.path.abspath(config_path)
    logger: logging.Logger = logging.getLogger(__name__)

    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    logger.info(f"正在加载算法配置: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        config: Dict[str, Any] = yaml.safe_load(f)

    if config is None:
        logger.warning("配置文件为空，将使用默认参数")
        return {}

    logger.info(
        "配置加载成功，包含以下顶层节点: %s",
        list(config.keys()) if isinstance(config, dict) else "非字典格式",
    )
    return config


# ============================================================================
# 模块初始化与销毁
# ============================================================================

def init_all_modules(config: Dict[str, Any]) -> bool:
    """
    按照 INTERFACE.md 接口契约，逐一初始化四大模块。

    初始化顺序：
        1. communication   — 通信链路
        2. motion_control  — 飞控连接
        3. vision_detect   — 视觉模型加载
        4. cluster_scheduler — 调度器（依赖上述模块正常运行）

    Args:
        config: algorithm.yaml 完整配置字典。

    Returns:
        True 表示全部模块初始化成功；任一模块失败则返回 False。
    """
    logger: logging.Logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("开始初始化四大模块...")
    logger.info("=" * 60)

    # ---------- 1. communication ----------
    comm_cfg_raw: Dict[str, Any] = config.get("communication", {})
    comm_config: CommConfig = CommConfig(**comm_cfg_raw)
    logger.info("[1/4] 初始化通信模块 communication ...")
    if not init_comm(comm_config):
        logger.critical("通信模块 communication 初始化失败，终止启动")
        return False
    logger.info("[1/4] communication 初始化完成")

    # ---------- 2. motion_control ----------
    motion_cfg_raw: Dict[str, Any] = config.get("motion_control", {})
    motion_config: MotionConfig = MotionConfig(**motion_cfg_raw)
    logger.info("[2/4] 初始化运动控制模块 motion_control ...")
    if not init_motion(motion_config):
        logger.critical("运动控制模块 motion_control 初始化失败，终止启动")
        return False
    logger.info("[2/4] motion_control 初始化完成")

    # ---------- 3. vision_detect ----------
    vision_cfg_raw: Dict[str, Any] = config.get("vision_detect", {})
    vision_config: VisionConfig = VisionConfig(**vision_cfg_raw)
    logger.info("[3/4] 初始化视觉感知模块 vision_detect ...")
    if not init_vision(vision_config):
        logger.critical("视觉感知模块 vision_detect 初始化失败，终止启动")
        return False
    logger.info("[3/4] vision_detect 初始化完成")

    # ---------- 4. cluster_scheduler ----------
    scheduler_cfg_raw: Dict[str, Any] = config.get("cluster_scheduler", {})
    scheduler_config: SchedulerConfig = SchedulerConfig(**scheduler_cfg_raw)
    logger.info("[4/4] 初始化集群调度模块 cluster_scheduler ...")
    if not init_scheduler(scheduler_config):
        logger.critical("集群调度模块 cluster_scheduler 初始化失败，终止启动")
        return False
    logger.info("[4/4] cluster_scheduler 初始化完成")

    logger.info("=" * 60)
    logger.info("全部模块初始化成功！")
    logger.info("=" * 60)
    return True


def shutdown_all_modules() -> None:
    """
    按照逆序安全关闭所有模块（先调度，后感知/控制/通信）。

    所有 shutdown_* 函数按 INTERFACE.md 定义为幂等操作，
    重复调用不会产生副作用。
    """
    logger: logging.Logger = logging.getLogger(__name__)
    logger.info("正在安全关闭所有模块...")

    errors: list[str] = []

    try:
        shutdown_scheduler()
        logger.info("  ✓ cluster_scheduler 已关闭")
    except Exception as e:
        errors.append(f"cluster_scheduler: {e}")

    try:
        shutdown_vision()
        logger.info("  ✓ vision_detect 已关闭")
    except Exception as e:
        errors.append(f"vision_detect: {e}")

    try:
        shutdown_motion()
        logger.info("  ✓ motion_control 已关闭")
    except Exception as e:
        errors.append(f"motion_control: {e}")

    try:
        shutdown_comm()
        logger.info("  ✓ communication 已关闭")
    except Exception as e:
        errors.append(f"communication: {e}")

    if errors:
        logger.warning("关闭过程中出现 %d 个错误: %s", len(errors), errors)
    else:
        logger.info("全部模块已安全关闭")


# ============================================================================
# ==================== 共享决策核心（纯逻辑，无 I/O 副作用） ====================
# ============================================================================

def _decide_one_frame(
    self_state: DroneState,
    observations: List[VisualObservation],
    teammate_states: List[DroneState],
    team_reports: List[TargetReport],
    last_cmd: Optional[MotionCommand] = None,
) -> Tuple[MotionCommand, bool]:
    """
    单帧决策核心 — 纯逻辑函数，不涉及模块调用或 I/O。

    此函数是 Agent.decide() 和 MainLoop._step() 的共享决策逻辑，
    两种模式的区别仅在于数据从哪来，决策过程完全一致。

    Args:
        self_state:       本机状态。
        observations:     本帧视觉观测列表。
        teammate_states:  队友状态列表。
        team_reports:     集群目标上报列表。
        last_cmd:         上一帧指令（用于判断是否需要取消旧指令）。

    Returns:
        (command, should_cancel):
            command       — 本帧决策输出的运动指令。
            should_cancel — 是否需要先取消上一条未完成指令。
    """
    logger: logging.Logger = logging.getLogger("decide")

    # ---------- Step 5: 调度器决策 ----------
    command: MotionCommand = scheduler_update(
        self_state=self_state,
        observations=observations,
        teammate_states=teammate_states,
        team_reports=team_reports,
    )

    # 若指令类型变更或为新指令，标记需要取消上一条未完成指令
    should_cancel: bool = (
        last_cmd is not None
        and command.cmd_type != last_cmd.cmd_type
    )

    logger.debug(
        "决策完成: cmd_id=%d, type=%s, cancel_prev=%s, drone=%s",
        command.cmd_id,
        command.cmd_type.value,
        should_cancel,
        self_state.drone_id,
    )

    return command, should_cancel


# ============================================================================
# ==================== Agent 类（赛事平台入口） ====================
# ============================================================================

class Agent(CoopAgent):
    """
    赛事平台加载入口 — 继承 CoopAgent，实现 decide() 方法。

    使用方法（由赛事平台自动加载，无需手动调用）：
        PYTHONPATH=. python -m competition run \\
            --scenario coop_decoy --agent src.main:Agent --duration 600

    每帧由平台调用 decide(obs)，Agent 从 obs 中解析数据，
    调用共享决策核心 _decide_one_frame() 生成指令，最后返回命令列表。

    约束（来自参赛手册 §4.2）：
        - decide() 仅允许读写自身内部状态
        - 禁止直接操作 Redis、读写本地文件、跨实体控制其他无人机
        - 通信、上报由平台自动处理，Agent 无需关注

    Attributes:
        _last_cmd:         上一帧输出的运动指令。
        _frame_count:      内部帧计数器（调试/统计用）。
        _logger:           Agent 专用 logger。
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """
        初始化 Agent。

        赛事平台会处理模块的 init/shutdown，这里只做 Agent 自身的状态初始化。
        """
        super().__init__(*args, **kwargs)
        self._last_cmd: Optional[MotionCommand] = None
        self._frame_count: int = 0
        self._logger: logging.Logger = logging.getLogger(self.__class__.__name__)
        self._logger.info(
            "Agent 初始化完成 (platform=%s)",
            "connected" if _HAS_PLATFORM else "standalone/fallback",
        )

    def decide(self, obs: Any) -> List[Dict[str, Any]]:
        """
        每帧由赛事平台调用，返回命令列表。

        平台通过 obs 对象提供以下字段（实际字段名以赛事 SDK 文档为准）：
            - obs.self_state         → 本机 DroneState
            - obs.detections         → 平台视觉检测结果
            - obs.teammate_states    → 队友状态列表
            - obs.comm_inbox         → 通信收件箱（含队友上报的目标信息）

        Args:
            obs: 平台注入的观测数据对象。

        Returns:
            平台可识别的命令字典列表（每帧可返回多条指令）。
        """
        try:
            # ---------- 1. 从 obs 解析输入数据 ----------
            self_state: DroneState = self._parse_self_state(obs)
            observations: List[VisualObservation] = self._parse_observations(obs)
            teammate_states: List[DroneState] = self._parse_teammate_states(obs)
            team_reports: List[TargetReport] = self._parse_team_reports(obs)

            self._logger.debug(
                "帧 #%d: self=%s, detections=%d, teammates=%d, reports=%d",
                self._frame_count,
                self_state.drone_id,
                sum(len(o.detections) for o in observations),
                len(teammate_states),
                len(team_reports),
            )

            # ---------- 2. 调用共享决策核心 ----------
            command, should_cancel = _decide_one_frame(
                self_state=self_state,
                observations=observations,
                teammate_states=teammate_states,
                team_reports=team_reports,
                last_cmd=self._last_cmd,
            )

            # ---------- 3. 需要先取消旧指令 ----------
            if should_cancel and self._last_cmd is not None:
                # 注意：赛事平台模式下，指令取消由平台管理，
                # 这里仅标记状态，实际的 cancel 语义通过返回 CANCEL 类型指令实现
                self._logger.debug(
                    "指令类型切换: %s → %s，将在平台端取消 cmd_id=%d",
                    self._last_cmd.cmd_type.value,
                    command.cmd_type.value,
                    self._last_cmd.cmd_id,
                )

            self._last_cmd = command
            self._frame_count += 1

            # ---------- 4. 转换为平台可识别的命令列表 ----------
            return [self._to_platform_cmd(command)]

        except Exception as e:
            self._logger.error(
                "decide() 异常: %s\n%s", e, traceback.format_exc()
            )
            # 异常时返回悬停指令作为安全兜底
            return [self._to_platform_cmd(self._build_hover_command())]

    # ------------------------------------------------------------------
    # obs 数据解析（适配平台数据格式）
    # ------------------------------------------------------------------

    def _parse_self_state(self, obs: Any) -> DroneState:
        """
        从 obs 中解析本机状态。

        适配平台提供的观测数据结构，若 obs 结构与预期不同，
        可通过修改本方法来适配。
        """
        if hasattr(obs, "self_state") and obs.self_state is not None:
            return obs.self_state
        # 兼容 dict 形式的 obs
        if isinstance(obs, dict) and "self_state" in obs:
            return obs["self_state"]
        # 兜底：尝试直接返回 obs（某些平台直接把 state 当 obs 传）
        if isinstance(obs, DroneState):
            return obs
        self._logger.warning("无法从 obs 解析 self_state，使用空状态")
        return DroneState(
            drone_id="unknown",
            timestamp=time.time(),
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            attitude=(0.0, 0.0, 0.0),
            battery=100.0,
            gps_fix=0,
            status=DroneStatus.IDLE,
            armed=False,
            home_position=(0.0, 0.0, 0.0),
        )

    def _parse_observations(self, obs: Any) -> List[VisualObservation]:
        """从 obs 中解析视觉检测结果。"""
        if hasattr(obs, "detections") and obs.detections is not None:
            dets = obs.detections
            if isinstance(dets, list):
                # 如果已经是 VisualObservation 列表，直接返回
                if all(isinstance(d, VisualObservation) for d in dets):
                    return dets
                # 如果是 Detection 列表，包装为单个 VisualObservation
                if all(isinstance(d, Detection) for d in dets):
                    return [
                        VisualObservation(
                            drone_id=self._get_obs_drone_id(obs),
                            timestamp=time.time(),
                            frame_id=self._frame_count,
                            image_width=0,
                            image_height=0,
                            detections=dets,
                            fov_h=90.0,
                            fov_v=60.0,
                        )
                    ]
        if isinstance(obs, dict) and "detections" in obs:
            return obs["detections"]
        return []

    def _parse_teammate_states(self, obs: Any) -> List[DroneState]:
        """从 obs 中解析队友状态。"""
        if hasattr(obs, "teammate_states") and obs.teammate_states is not None:
            return obs.teammate_states
        if isinstance(obs, dict) and "teammate_states" in obs:
            return obs["teammate_states"]
        return []

    def _parse_team_reports(self, obs: Any) -> List[TargetReport]:
        """从 obs 中解析集群目标上报。"""
        if hasattr(obs, "comm_inbox") and obs.comm_inbox is not None:
            # comm_inbox 可能包含混合消息，需要过滤 TargetReport
            inbox = obs.comm_inbox
            if isinstance(inbox, list):
                return [m for m in inbox if isinstance(m, TargetReport)]
        if isinstance(obs, dict):
            if "comm_inbox" in obs:
                inbox = obs["comm_inbox"]
                if isinstance(inbox, list):
                    return [m for m in inbox if isinstance(m, TargetReport)]
            if "team_reports" in obs:
                return obs["team_reports"]
        return []

    def _get_obs_drone_id(self, obs: Any) -> str:
        """从 obs 中提取无人机 ID。"""
        if hasattr(obs, "drone_id"):
            return obs.drone_id
        if hasattr(obs, "self_state") and obs.self_state is not None:
            return obs.self_state.drone_id
        if isinstance(obs, dict):
            return obs.get("drone_id", "unknown")
        return "unknown"

    # ------------------------------------------------------------------
    # 平台命令转换
    # ------------------------------------------------------------------

    def _to_platform_cmd(self, command: MotionCommand) -> Dict[str, Any]:
        """
        将 MotionCommand 转换为赛事平台可识别的命令字典。

        平台命令格式以赛事 SDK 文档为准，此处提供标准映射。
        """
        platform_cmd: Dict[str, Any] = {
            "cmd_id": command.cmd_id,
            "drone_id": command.drone_id,
            "type": command.cmd_type.value,
            "timestamp": command.timestamp,
            "priority": command.priority,
        }

        # 按指令类型附加相应字段
        if command.target_position is not None:
            platform_cmd["target_position"] = list(command.target_position)
        if command.target_velocity is not None:
            platform_cmd["target_velocity"] = list(command.target_velocity)
        if command.target_yaw is not None:
            platform_cmd["target_yaw"] = command.target_yaw
        if command.altitude is not None:
            platform_cmd["altitude"] = command.altitude
        if command.loiter_radius is not None:
            platform_cmd["loiter_radius"] = command.loiter_radius
        if command.loiter_turns is not None:
            platform_cmd["loiter_turns"] = command.loiter_turns
        if command.timeout > 0:
            platform_cmd["timeout"] = command.timeout

        # 若需要取消上一帧指令，附加 cancel_prev 标记
        if (
            self._last_cmd is not None
            and command.cmd_type != self._last_cmd.cmd_type
        ):
            platform_cmd["cancel_prev"] = self._last_cmd.cmd_id

        return platform_cmd

    def _build_hover_command(self) -> MotionCommand:
        """构建安全兜底的悬停指令。"""
        return MotionCommand(
            cmd_id=self._frame_count,
            drone_id="unknown",
            cmd_type=CommandType.HOVER,
            timestamp=time.time(),
            priority=0,
            timeout=3.0,
        )


# ============================================================================
# ==================== MainLoop 类（独立测试入口） ====================
# ============================================================================

class MainLoop:
    """
    独立测试模式的主循环调度器。

    按照 INTERFACE.md 定义的数据流（帧循环 + 目标发现路径），
    每周期依次执行：
        1. 获取本机状态               (motion_control)
        2. 获取相机帧数据             (外部传感器/仿真接口)
        3. 视觉检测                   (vision_detect)
        4. 获取队友状态 + 目标上报     (communication)
        5. 调度器决策                 (cluster_scheduler) ← 共享核心
        6. 执行运动指令               (motion_control)
        7. 广播本机状态               (communication)
        8. 上报告警目标               (communication)
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        """
        Args:
            config: algorithm.yaml 完整配置字典。
        """
        self.logger: logging.Logger = logging.getLogger(self.__class__.__name__)

        # ---- 主循环参数 ----
        main_loop_cfg: Dict[str, Any] = config.get("main_loop", {})
        self.target_hz: float = float(
            main_loop_cfg.get("frequency_hz", DEFAULT_MAIN_LOOP_HZ)
        )
        self.cycle_period: float = 1.0 / self.target_hz  # 目标周期（秒）

        # 策略广播间隔: 每 N 帧广播一次本机状态以降低通信开销
        self.broadcast_interval: int = int(
            main_loop_cfg.get("broadcast_interval_frames", 5)
        )

        # 统计信息打印间隔
        self.stats_interval: int = int(
            main_loop_cfg.get("stats_interval_frames", 100)
        )

        # ---- 运行时状态 ----
        self.running: bool = False
        self.frame_count: int = 0
        self.last_cmd: Optional[MotionCommand] = None  # 上一条指令，用于取消/续发判断

        # ---- 相机帧获取接口（占位） ----
        # 仿真环境需要对接仿真器；实飞环境需要对接相机驱动。
        # 具体实现在 src/sensor/ 或仿真适配层中完成。
        self.camera_available: bool = config.get("simulation", {}).get(
            "camera_enabled", True
        )

        self.logger.info(
            "主循环初始化完成: target_hz=%.1f Hz, cycle_period=%.3f s, "
            "broadcast_interval=%d, stats_interval=%d",
            self.target_hz,
            self.cycle_period,
            self.broadcast_interval,
            self.stats_interval,
        )

    # ------------------------------------------------------------------
    # 相机帧获取（占位函数，由仿真/实飞层具体实现）
    # ------------------------------------------------------------------

    def _capture_frame(self) -> Optional[np.ndarray]:
        """
        从相机/仿真环境捕获一帧图像。

        Returns:
            BGR 格式图像 (H, W, 3) uint8；若相机不可用或获取失败返回 None。

        Note:
            实飞时替换为相机驱动 SDK 调用；
            仿真时对接 AirSim / Gazebo / UE5 等图像接口。
        """
        if not self.camera_available:
            return None

        try:
            # TODO: 替换为实际相机/仿真帧获取逻辑
            # 示例：仿真环境通过共享内存或 RPC 获取帧
            # frame = simulation_interface.get_image()
            return None  # 占位返回 None，表示无帧
        except Exception as e:
            self.logger.warning("相机帧获取异常: %s", e)
            return None

    # ------------------------------------------------------------------
    # 广播与上报
    # ------------------------------------------------------------------

    def _broadcast_self_state(self, state: DroneState) -> None:
        """
        向集群广播本机状态（低频，降低通信开销）。

        Args:
            state: 本机 DroneState。
        """
        if self.frame_count % self.broadcast_interval == 0:
            if not broadcast_drone_state(state):
                self.logger.warning("广播本机状态失败")

    def _handle_target_reports(self, observations: list[VisualObservation]) -> None:
        """
        若视觉检测到高置信度目标，通过通信模块上报告警。

        Args:
            observations: 本帧视觉观测列表。
        """
        for obs in observations:
            for det in obs.detections:
                # 仅上报置信度超过阈值且具有 3D 位置估算的目标
                if det.confidence < 0.5 or det.world_position is None:
                    continue

                report: TargetReport = TargetReport(
                    report_id=f"{obs.drone_id}_{obs.frame_id}_{det.track_id}",
                    drone_id=obs.drone_id,
                    timestamp=obs.timestamp,
                    target_id=det.track_id,
                    target_class=det.class_name,
                    target_position=det.world_position,
                    target_velocity=None,  # 速度由调度器端估算
                    confidence=det.confidence,
                    priority=0,
                )
                if not send_target_report(report):
                    self.logger.warning(
                        "目标上报失败: drone=%s, target_id=%d, class=%s",
                        obs.drone_id,
                        det.track_id,
                        det.class_name,
                    )

    # ------------------------------------------------------------------
    # 统计信息
    # ------------------------------------------------------------------

    def _print_statistics(self, elapsed: float) -> None:
        """
        周期性打印运行统计信息（调试和性能监控用）。

        Args:
            elapsed: 本周期耗时（秒）。
        """
        if self.frame_count % self.stats_interval != 0:
            return

        try:
            stats: Dict[str, Any] = get_statistics()
            vision_fps: float = vision_get_fps()
            battery: float = get_battery()
            actual_hz: float = (
                1.0 / elapsed if elapsed > 0 else 0.0
            )
            self.logger.info(
                "[STATS] frame=%d | actual_hz=%.1f | vision_fps=%.1f | "
                "battery=%.1f%% | cycle_ms=%.1f | scheduler=%s",
                self.frame_count,
                actual_hz,
                vision_fps,
                battery,
                elapsed * 1000,
                stats,
            )
        except Exception as e:
            self.logger.debug("获取统计信息异常: %s", e)

    # ------------------------------------------------------------------
    # 命令执行（独立测试模式专用 — 直接调用 motion_control API）
    # ------------------------------------------------------------------

    def _execute_command(self, command: MotionCommand) -> None:
        """
        执行运动指令（独立测试模式）。

        Args:
            command: 由共享决策核心输出的 MotionCommand。
        """
        # 若指令类型变更，取消上一条未完成指令
        if self.last_cmd is not None and command.cmd_type != self.last_cmd.cmd_type:
            cancel_command(self.last_cmd.cmd_id)

        if not execute_command(command):
            self.logger.warning(
                "运动指令执行失败: cmd_id=%d, type=%s, drone=%s",
                command.cmd_id,
                command.cmd_type.value,
                command.drone_id,
            )
        else:
            self.logger.debug(
                "指令已下发: cmd_id=%d, type=%s",
                command.cmd_id,
                command.cmd_type.value,
            )
        self.last_cmd = command

    # ------------------------------------------------------------------
    # 单帧循环
    # ------------------------------------------------------------------

    def _step(self) -> bool:
        """
        执行主循环的一个完整周期（独立测试模式）。

        严格按照 INTERFACE.md 数据流顺序：
            state → frame → detect → comm → (共享决策核心) → execute → broadcast → report

        Returns:
            True 表示本周期正常执行；False 表示出现不可恢复错误。
        """
        try:
            # ---------- Step 1: 获取本机状态 ----------
            self_state: DroneState = get_drone_state()
            self.logger.debug(
                "本机状态: id=%s, pos=(%.3f, %.3f, %.3f), status=%s",
                self_state.drone_id,
                *self_state.position,
                self_state.status.value,
            )

            # ---------- Step 2: 获取相机帧 ----------
            frame: Optional[np.ndarray] = self._capture_frame()

            # ---------- Step 3: 视觉检测 ----------
            observations: list[VisualObservation] = []
            if frame is not None:
                observations.append(detect(frame, self_state))
                num_dets: int = sum(len(obs.detections) for obs in observations)
                if num_dets > 0:
                    self.logger.info("视觉检测到 %d 个目标", num_dets)

            # ---------- Step 4: 获取集群信息 ----------
            teammate_states: list[DroneState] = receive_drone_states()
            team_reports: list[TargetReport] = receive_target_reports()
            self.logger.debug(
                "集群信息: teammates=%d, reports=%d",
                len(teammate_states),
                len(team_reports),
            )

            # ---------- Step 5: 调用共享决策核心 ----------
            command, should_cancel = _decide_one_frame(
                self_state=self_state,
                observations=observations,
                teammate_states=teammate_states,
                team_reports=team_reports,
                last_cmd=self.last_cmd,
            )

            # ---------- Step 6: 执行运动指令（独立测试模式特有）----------
            self._execute_command(command)

            # ---------- Step 7: 广播本机状态 ----------
            self._broadcast_self_state(self_state)

            # ---------- Step 8: 目标上报告警 ----------
            self._handle_target_reports(observations)

            return True

        except Exception as e:
            self.logger.error(
                "主循环周期 #%d 异常: %s\n%s",
                self.frame_count,
                e,
                traceback.format_exc(),
            )
            return False

    # ------------------------------------------------------------------
    # 主循环入口
    # ------------------------------------------------------------------

    def run(self) -> None:
        """
        启动并运行主循环。

        循环终止条件：
            - 外部信号 (SIGINT / SIGTERM)
            - 调度器返回紧急指令 (CommandType.EMERGENCY)
            - 电量过低
            - 连续异常超过阈值
        """
        self.running = True
        consecutive_errors: int = 0
        max_consecutive_errors: int = 5  # 连续异常上限

        self.logger.info(
            "===== 主循环启动 (目标 %.1f Hz, 周期 %.3f s) =====",
            self.target_hz,
            self.cycle_period,
        )

        while self.running:
            t_start: float = time.time()

            # 执行单周期
            if self._step():
                consecutive_errors = 0
            else:
                consecutive_errors += 1
                self.logger.warning(
                    "连续异常计数: %d / %d", consecutive_errors, max_consecutive_errors
                )
                if consecutive_errors >= max_consecutive_errors:
                    self.logger.critical(
                        "连续异常达到上限 (%d)，触发安全停机", max_consecutive_errors
                    )
                    self._emergency_halt()
                    break

            self.frame_count += 1

            # ---- 检查紧急停止条件 ----
            if self.last_cmd is not None and self.last_cmd.cmd_type == CommandType.EMERGENCY:
                self.logger.critical("调度器返回紧急停止指令，终止主循环")
                break

            # 电量过低保护
            try:
                battery: float = get_battery()
                if battery < 10.0:
                    self.logger.critical(
                        "电量过低 (%.1f%%)，触发自动返航", battery
                    )
                    self._emergency_halt()
                    break
            except Exception:
                pass  # 电量查询失败不阻塞主循环

            # ---- 帧率控制 ----
            elapsed: float = time.time() - t_start
            self._print_statistics(elapsed)

            # 动态休眠以维持目标频率
            sleep_time: float = self.cycle_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.logger.info("主循环已退出，共执行 %d 帧", self.frame_count)

    def _emergency_halt(self) -> None:
        """紧急停止：尝试下发悬停/降落指令。"""
        self.logger.warning("正在执行紧急停止...")
        try:
            emergency_stop()
        except Exception as e:
            self.logger.error("紧急停止异常: %s", e)
        self.running = False


# ============================================================================
# 信号处理
# ============================================================================

def _create_signal_handler(main_loop: MainLoop):
    """
    创建信号处理器闭包，捕获 SIGINT / SIGTERM 后优雅退出。

    Args:
        main_loop: 主循环实例引用。
    """

    def handler(signum: int, frame) -> None:  # noqa: ARG001
        logger: logging.Logger = logging.getLogger("signal_handler")
        sig_name: str = signal.Signals(signum).name
        logger.info("接收到信号 %s (%d)，准备优雅退出...", sig_name, signum)
        main_loop.running = False

    return handler


# ============================================================================
# 程序入口（独立测试模式）
# ============================================================================

def main() -> None:
    """
    独立测试模式程序主入口。

    执行流程：
        1. 加载 algorithm.yaml 配置
        2. 初始化日志
        3. 逐一初始化四大模块
        4. 注册信号处理器
        5. 启动主循环 (MainLoop.run)
        6. 异常捕获 → 资源回收 → 退出

    注意：赛事平台模式下不会调用此函数，
          平台通过 src.main:Agent 加载 Agent 类并直接调用 decide()。
    """
    # ---- 临时 logger（日志系统就绪前使用） ----
    root_logger: logging.Logger = logging.getLogger()
    tmp_handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
    tmp_handler.setFormatter(
        logging.Formatter(DEFAULT_LOG_FORMAT, datefmt=DEFAULT_LOG_DATE_FORMAT)
    )
    root_logger.handlers.clear()
    root_logger.addHandler(tmp_handler)
    root_logger.setLevel(logging.INFO)

    logger: logging.Logger = logging.getLogger("main")
    config: Dict[str, Any] = {}

    try:
        # ----------------------------------------------------------------
        # Step A: 加载配置
        # ----------------------------------------------------------------
        config_path: str = os.environ.get(
            "ALGORITHM_CONFIG_PATH", DEFAULT_CONFIG_PATH
        )
        config = load_config(config_path)

        # ----------------------------------------------------------------
        # Step B: 初始化日志（使用配置文件中的日志参数）
        # ----------------------------------------------------------------
        log_config: Dict[str, Any] = config.get("logging", {})
        setup_logging(log_config)
        logger = logging.getLogger("main")  # 刷新 logger 引用
        logger.info("HF2026_UAV_Challenge2 无人机集群弱对抗跟踪竞赛系统启动")
        logger.info("运行模式: 独立测试 (Standalone)")
        logger.info("项目根目录: %s", PROJECT_ROOT)
        if _HAS_PLATFORM:
            logger.info("检测到赛事平台 SDK，Agent 类可用于平台加载")
        else:
            logger.info("未检测到赛事平台 SDK，Agent 类以 fallback 模式运行")

        # ----------------------------------------------------------------
        # Step C: 初始化所有模块
        # ----------------------------------------------------------------
        if not init_all_modules(config):
            logger.critical("模块初始化失败，系统退出")
            sys.exit(1)

        # 可选：根据配置设置初始策略
        initial_strategy: Optional[str] = config.get("main_loop", {}).get(
            "initial_strategy", None
        )
        if initial_strategy:
            try:
                strategy: Strategy = Strategy[initial_strategy.upper()]
                set_strategy(strategy)
                logger.info("初始策略已设置: %s", strategy.value)
            except KeyError:
                logger.warning(
                    "未知策略名 '%s'，使用调度器默认策略", initial_strategy
                )

        # ----------------------------------------------------------------
        # Step D: 创建主循环实例，注册信号处理器
        # ----------------------------------------------------------------
        main_loop: MainLoop = MainLoop(config)

        # 注册信号处理器（SIGINT=Ctrl+C, SIGTERM=系统终止）
        sig_handler = _create_signal_handler(main_loop)
        signal.signal(signal.SIGINT, sig_handler)
        signal.signal(signal.SIGTERM, sig_handler)
        logger.info("信号处理器已注册 (SIGINT, SIGTERM)")

        # ----------------------------------------------------------------
        # Step E: 启动主循环
        # ----------------------------------------------------------------
        logger.info("所有准备工作完成，即将进入主循环...")
        main_loop.run()

    except FileNotFoundError as e:
        logger.critical("配置文件加载失败: %s", e)
        sys.exit(1)
    except yaml.YAMLError as e:
        logger.critical("YAML 配置文件格式错误: %s", e)
        sys.exit(1)
    except KeyboardInterrupt:
        logger.info("用户中断 (Ctrl+C)，正在退出...")
    except Exception as e:
        logger.critical(
            "未捕获异常导致系统崩溃: %s\n%s", e, traceback.format_exc()
        )
        sys.exit(1)
    finally:
        # ----------------------------------------------------------------
        # Step F: 资源回收（务必执行）
        # ----------------------------------------------------------------
        logger.info("正在执行清理和资源回收...")
        shutdown_all_modules()
        logger.info("系统已安全退出")


# ============================================================================
# 脚本入口
# ============================================================================

if __name__ == "__main__":
    main()
