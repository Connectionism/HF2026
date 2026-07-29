#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
  HF2026_UAV_Challenge2 — 统一配置管理模块 (config.py)
  赛题二：多机协同识别（3 机 + 3 真目标 + 15 诱饵）
===============================================================================

  【职责】
    1. 从 config/algorithm.yaml 加载全量算法参数
    2. 提供类型安全的配置节（dataclass），各模块直接取用
    3. 支持环境变量覆盖（ALGO_<节>_<键>）
    4. 配置校验（必填字段、数值范围）
    5. 单例模式 — 全项目共享同一份配置

  【赛题二核心参数（手册 V1.1.0）】
    - 受控实体：3 架无人机，各有独立 Agent 实例
    - 目标配置：3 真目标 + 15 移动诱饵
    - 通信约束：距离 ≤1000m，单条 ≤50 字节，频率 ≤4Hz
    - 协同盯防：K=2（≥2 架同时盯防同一真目标累计 20s）
    - 仿真时长：600s
    - 评分：摧毁率 0.5 + 目指精度 0.3 + 完成速度 0.2
    - 速度窗口：240s 内全歼满分，240–420s 线性衰减
    - 目指 RMSE 基准：120m

  【文件位置】
    src/config.py — 与 src/main.py 同级
    配置文件：项目根目录 config/algorithm.yaml

  【使用方式】
    from src.config import ConfigManager

    # 加载配置（按需，通常 main.py 启动时调用一次）
    cfg = ConfigManager()
    cfg.load()  # 使用默认路径 config/algorithm.yaml

    # 各模块从配置节取值
    comm_cfg = cfg.communication    # CommunicationConfig 实例
    motion_cfg = cfg.motion_control # MotionControlConfig 实例

  【作者】HF2026_UAV_Challenge2 团队（队长）
  【版本】v1.0
  【日期】2026-07-28
===============================================================================
"""

from __future__ import annotations

import os
import sys
import copy
import logging
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List, Tuple

import yaml

# ============================================================================
# 项目根路径推导（src/config.py → 上一级 = 项目根）
# ============================================================================
_PROJECT_ROOT: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ============================================================================
# 赛题二常量（依据赛事手册）
# ============================================================================

# -- 地图边界（§2.1：6.6km × 4.4km）--
MAP_LAT_MIN: float = 26.9818
MAP_LAT_MAX: float = 27.0250
MAP_LON_MIN: float = 124.9800
MAP_LON_MAX: float = 125.0203

# -- 无人机（§3.2.2）--
DRONE_COUNT: int = 3                     # 受控实体数量
DRONE_ALTITUDE_LOCK: float = 500.0       # 飞行高度锁定 500m

# -- 目标（§3.2.2）--
REAL_TARGET_COUNT: int = 3               # 真实目标数
DECOY_COUNT: int = 15                    # 移动诱饵数

# -- 通信约束（§3.2.2）--
COMM_RANGE_M: float = 1000.0             # 通信极限距离
COMM_MAX_BYTES: int = 50                 # 单条消息上限
COMM_MAX_HZ: int = 4                     # 发送频率上限

# -- 盯防摧毁（§5.2）--
DESTROY_TIME_S: float = 20.0             # 累计盯防秒数
TRACKING_GAP_S: float = 2.0              # 中断不清零窗口（≤2s）
COOP_K: int = 2                          # 协同阈值：≥2 机同时盯防

# -- 仿真（§3.2.2）--
SIM_DURATION_S: float = 600.0            # 仿真总时长

# -- 评分（§5.3 赛题二）--
SCORE_WEIGHT_DESTROY: float = 0.5        # 目标摧毁率权重
SCORE_WEIGHT_ACCURACY: float = 0.3       # 持续目指精度权重
SCORE_WEIGHT_SPEED: float = 0.2          # 任务完成速度权重
SPEED_FULL_SCORE_S: float = 240.0        # 全歼满分时间窗口
SPEED_DECAY_START_S: float = 240.0       # 速度衰减开始
SPEED_DECAY_END_S: float = 420.0         # 速度衰减结束
RMSE_BASELINE_M: float = 120.0           # 目指 RMSE 基准

# -- 扣分（§5.3 补充说明）--
MIN_INTER_DRONE_DIST: float = 200.0      # 机间安全距离 (m)
BOUNDARY_TOLERANCE: float = 500.0        # 超出边界容忍 (m)
MAX_PENALTY_POINTS: float = 15.0         # 扣分合计上限

# -- 平台调度（§6.1）--
PLATFORM_DECIDE_HZ: int = 10             # 平台调用 decide() 频率

# -- 诱饵判别（§6.2）--
# 诱饵 detection 被伪装成 ground_vehicle，target_type 不可靠
# 靠多帧运动学模式区分


# ============================================================================
# 默认配置路径 & 环境变量
# ============================================================================
DEFAULT_CONFIG_PATH: str = os.path.join(
    _PROJECT_ROOT, "config", "algorithm.yaml"
)
ENV_CONFIG_PATH: str = "ALGORITHM_CONFIG_PATH"
ENV_OVERRIDE_PREFIX: str = "ALGO_"


# ============================================================================
# 配置节 DataClass 定义
# ============================================================================

@dataclass
class CommunicationConfig:
    """通信模块配置。

    依据手册 §3.2.2：距离 ≤1000m，单条 ≤50B，频率 ≤4Hz。
    """
    range_m:            float = COMM_RANGE_M
    max_bytes_per_msg:  int   = COMM_MAX_BYTES
    max_hz:             int   = COMM_MAX_HZ

    redis_host:         str   = "127.0.0.1"
    redis_port:         int   = 6379
    redis_db:           int   = 0
    channel_prefix:     str   = "swarm:comm"
    heartbeat_interval: float = 1.0
    state_timeout:      float = 3.0
    encrypt:            bool  = False


@dataclass
class MotionControlConfig:
    """运动控制模块配置。

    所有飞行高度锁定 500m（手册 §3.3.1）。
    """
    max_speed:        float = 20.0
    max_accel:        float = 5.0
    max_yaw_rate:     float = 45.0
    altitude_default: float = DRONE_ALTITUDE_LOCK
    goto_tolerance:   float = 5.0
    loiter_default_r: float = 100.0
    command_timeout:  float = 10.0
    takeoff_altitude: float = DRONE_ALTITUDE_LOCK

    # 安全边界（扣分触发线）
    boundary_tolerance: float = BOUNDARY_TOLERANCE
    min_inter_drone_dist: float = MIN_INTER_DRONE_DIST


@dataclass
class VisionDetectConfig:
    """视觉感知模块配置。

    赛题二难点（手册 §6.2）：
      - detection 无法区分诱饵与真目标（target_type 不可靠）
      - 诱饵被伪装成 ground_vehicle
      - 必须靠多帧运动学模式（速度、轨迹一致性）判别
    """
    model_path:          str   = ""
    input_width:         int   = 640
    input_height:        int   = 640
    conf_threshold:      float = 0.5
    nms_iou_threshold:   float = 0.45
    max_detections:      int   = 50
    tracker_type:        str   = "bytetrack"
    tracker_max_age:     int   = 30
    fov_horizontal:      float = 90.0
    fov_vertical:        float = 60.0

    # 诱饵判别
    decoy_min_track_len: int   = 15     # 判别所需最小跟踪帧数
    decoy_velocity_var:  float = 2.0    # 运动学不一致阈值 (m²/s²)
    # 诱饵误识别概率 ~50%（手册 §3.2.1），靠时序过滤


@dataclass
class SchedulerConfig:
    """集群调度模块配置。

    赛题二核心逻辑：
      - 3 机分布式协同搜索
      - K=2 协同盯防（≥2 机同时盯同一真目标 20s）
      - 基于运动学判别区分诱饵
      - 目指上报 report_target(lat, lon)
    """
    drone_count:       int   = DRONE_COUNT
    real_target_count: int   = REAL_TARGET_COUNT
    coop_k:            int   = COOP_K              # 协同盯防需 K 架
    destroy_time_s:    float = DESTROY_TIME_S       # 累计盯防秒数
    tracking_gap_s:    float = TRACKING_GAP_S       # 中断不清零窗口

    # 搜索策略
    search_strategy:      str   = "hash_partition"  # 哈希分区搜索
    search_altitude:      float = DRONE_ALTITUDE_LOCK
    search_spacing:       float = 800.0             # 搜索线间距 (m)
    revisit_cooldown:     float = 30.0              # 区域重访冷却 (s)

    # 协同盯防
    tracking_formation:   str = "loose_circle"      # 盯防编队
    tracking_standoff:    float = 200.0             # 盯防站位距离 (m)
    min_overlap_frames:   int = 3                   # 队友同目标确认帧数

    # 目指上报
    report_min_confidence: float = 0.7              # 上报置信度阈值
    report_interval_s:     float = 1.0              # 上报间隔（每秒最多 1 次）
    rmse_baseline_m:       float = RMSE_BASELINE_M  # RMSE 评分基准

    # 策略枚举
    available_strategies: tuple = ("search", "track", "loiter", "return_home")


@dataclass
class MainLoopConfig:
    """主循环调度配置（独立测试模式专用）。"""
    frequency_hz:              float = 25.0
    broadcast_interval_frames: int   = 5
    stats_interval_frames:     int   = 100
    initial_strategy:          str   = "search"
    max_consecutive_errors:    int   = 5
    low_battery_threshold:     float = 10.0


@dataclass
class SimulationConfig:
    """仿真环境配置。"""
    camera_enabled:   bool  = True
    seed:             int   = -1               # -1 表示随机
    duration_s:       float = SIM_DURATION_S
    scenario:         str   = "coop_decoy"     # 赛题二场景名
    output_dir:       str   = "output"


@dataclass
class LoggingConfig:
    """日志配置。"""
    level:  str  = "INFO"
    file:   str  = ""
    format: str  = "%(asctime)s [%(levelname)-8s] %(name)-20s | %(message)s"
    datefmt: str = "%Y-%m-%d %H:%M:%S"


# ============================================================================
# 顶层配置容器
# ============================================================================

@dataclass
class AlgorithmConfig:
    """algorithm.yaml 的完整映射。"""
    communication:    CommunicationConfig    = field(default_factory=CommunicationConfig)
    motion_control:   MotionControlConfig    = field(default_factory=MotionControlConfig)
    vision_detect:    VisionDetectConfig     = field(default_factory=VisionDetectConfig)
    cluster_scheduler: SchedulerConfig       = field(default_factory=SchedulerConfig)
    main_loop:        MainLoopConfig         = field(default_factory=MainLoopConfig)
    simulation:       SimulationConfig       = field(default_factory=SimulationConfig)
    logging:          LoggingConfig          = field(default_factory=LoggingConfig)


# ============================================================================
# ConfigManager 单例
# ============================================================================

class ConfigManager:
    """统一配置管理器（单例）。

    生命周期：
        1. main() 启动时调用 load() 加载 YAML
        2. 后续通过属性访问各配置节
        3. 各模块 init 时传入对应配置节实例

    线程安全：本类本身不保证线程安全；
    调用者应在加载阶段（单线程）完成 load()，
    运行时只读属性访问天然线程安全。
    """

    _instance: Optional[ConfigManager] = None
    _config: Optional[AlgorithmConfig] = None
    _config_path: str = ""
    _raw: Dict[str, Any] = {}

    def __new__(cls) -> ConfigManager:
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    # ------------------------------------------------------------------
    # 加载
    # ------------------------------------------------------------------

    def load(self, config_path: Optional[str] = None) -> AlgorithmConfig:
        """加载并解析 algorithm.yaml。

        Args:
            config_path: YAML 路径；None 则依次尝试：
                         1. 环境变量 ALGORITHM_CONFIG_PATH
                         2. 默认路径 config/algorithm.yaml

        Returns:
            解析后的 AlgorithmConfig 实例。

        Raises:
            FileNotFoundError: 配置文件不存在。
            yaml.YAMLError: YAML 语法错误。
            ValueError: 配置校验失败。
        """
        resolved = config_path or os.environ.get(
            ENV_CONFIG_PATH, DEFAULT_CONFIG_PATH
        )
        resolved = os.path.abspath(resolved)
        self._config_path = resolved

        logger = logging.getLogger(__name__)

        if not os.path.isfile(resolved):
            raise FileNotFoundError(f"配置文件不存在: {resolved}")

        logger.info("加载算法配置: %s", resolved)
        with open(resolved, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}

        self._raw = raw
        self._config = self._build_config(raw)
        self._apply_env_overrides()
        self._validate()

        logger.info(
            "配置加载完成: scenario=%s, drones=%d, targets=%d, coop_k=%d, duration=%.0fs",
            self._config.simulation.scenario,
            self._config.cluster_scheduler.drone_count,
            self._config.cluster_scheduler.real_target_count,
            self._config.cluster_scheduler.coop_k,
            self._config.simulation.duration_s,
        )
        return self._config

    # ------------------------------------------------------------------
    # 属性：按节访问
    # ------------------------------------------------------------------

    @property
    def config(self) -> AlgorithmConfig:
        """返回完整配置容器。未加载时自动使用默认值。"""
        if self._config is None:
            self._config = AlgorithmConfig()
        return self._config

    @property
    def communication(self) -> CommunicationConfig:
        return self.config.communication

    @property
    def motion_control(self) -> MotionControlConfig:
        return self.config.motion_control

    @property
    def vision_detect(self) -> VisionDetectConfig:
        return self.config.vision_detect

    @property
    def cluster_scheduler(self) -> SchedulerConfig:
        return self.config.cluster_scheduler

    @property
    def main_loop(self) -> MainLoopConfig:
        return self.config.main_loop

    @property
    def simulation(self) -> SimulationConfig:
        return self.config.simulation

    @property
    def logging(self) -> LoggingConfig:
        return self.config.logging

    @property
    def raw(self) -> Dict[str, Any]:
        """返回原始 YAML 字典（兼容旧代码逐节取 dict 的用法）。"""
        return copy.deepcopy(self._raw)

    @property
    def config_path(self) -> str:
        return self._config_path

    # ------------------------------------------------------------------
    # 内部：构建配置对象
    # ------------------------------------------------------------------

    @staticmethod
    def _merge_defaults(
        raw_section: Dict[str, Any], defaults: dataclass
    ) -> dataclass:
        """用 YAML 值覆盖 dataclass 默认字段。

        只覆盖 dataclass 已定义的字段，忽略 YAML 中的未知键。
        """
        default_fields = {f.name: getattr(defaults, f.name)
                          for f in defaults.__dataclass_fields__.values()}
        merged = {**default_fields}
        for k in raw_section:
            if k in merged:
                merged[k] = raw_section[k]
        return defaults.__class__(**merged)

    def _build_config(self, raw: Dict[str, Any]) -> AlgorithmConfig:
        """从 YAML 字典构建带默认值的 AlgorithmConfig。"""
        return AlgorithmConfig(
            communication=self._merge_defaults(
                raw.get("communication", {}), CommunicationConfig()
            ),
            motion_control=self._merge_defaults(
                raw.get("motion_control", {}), MotionControlConfig()
            ),
            vision_detect=self._merge_defaults(
                raw.get("vision_detect", {}), VisionDetectConfig()
            ),
            cluster_scheduler=self._merge_defaults(
                raw.get("cluster_scheduler", {}), SchedulerConfig()
            ),
            main_loop=self._merge_defaults(
                raw.get("main_loop", {}), MainLoopConfig()
            ),
            simulation=self._merge_defaults(
                raw.get("simulation", {}), SimulationConfig()
            ),
            logging=self._merge_defaults(
                raw.get("logging", {}), LoggingConfig()
            ),
        )

    # ------------------------------------------------------------------
    # 内部：环境变量覆盖
    # ------------------------------------------------------------------

    def _apply_env_overrides(self) -> None:
        """根据环境变量 ALGO_<节>_<键>=值 覆盖配置字段。

        映射规则（全大写，_ 对应层级）：
            ALGO_COMMUNICATION_RANGE_M=800
                → config.communication.range_m = 800.0
            ALGO_CLUSTER_SCHEDULER_COOP_K=3
                → config.cluster_scheduler.coop_k = 3
        """
        if self._config is None:
            return

        section_map = {
            "COMMUNICATION": self._config.communication,
            "MOTION_CONTROL": self._config.motion_control,
            "VISION_DETECT": self._config.vision_detect,
            "CLUSTER_SCHEDULER": self._config.cluster_scheduler,
            "MAIN_LOOP": self._config.main_loop,
            "SIMULATION": self._config.simulation,
            "LOGGING": self._config.logging,
        }
        logger = logging.getLogger(__name__)

        for env_key, env_val in os.environ.items():
            if not env_key.startswith(ENV_OVERRIDE_PREFIX):
                continue
            # ALGO_COMMUNICATION_RANGE_M → ["COMMUNICATION", "RANGE", "M"]
            parts = env_key[len(ENV_OVERRIDE_PREFIX):].split("_", 1)
            if len(parts) < 2:
                continue
            section_name = parts[0]
            field_key = parts[1].lower()

            section = section_map.get(section_name)
            if section is None:
                continue

            if not hasattr(section, field_key):
                logger.debug("忽略未知覆盖字段: %s", env_key)
                continue

            field_type = type(getattr(section, field_key))
            try:
                converted = field_type(env_val)
                setattr(section, field_key, converted)
                logger.info(
                    "环境变量覆盖: %s.%s = %s (来自 %s)",
                    section_name, field_key, converted, env_key,
                )
            except (ValueError, TypeError) as e:
                logger.warning(
                    "环境变量 %s 类型转换失败 (%s)，保持默认值", env_key, e
                )

    # ------------------------------------------------------------------
    # 内部：校验
    # ------------------------------------------------------------------

    def _validate(self) -> None:
        """校验配置的合法性，不合法则抛出 ValueError。"""
        if self._config is None:
            return

        errors: List[str] = []
        mc = self._config.motion_control
        sc = self._config.cluster_scheduler
        sim = self._config.simulation

        # 运动控制
        if mc.max_speed <= 0:
            errors.append("motion_control.max_speed 必须 > 0")
        if mc.max_accel <= 0:
            errors.append("motion_control.max_accel 必须 > 0")
        if mc.altitude_default <= 0:
            errors.append("motion_control.altitude_default 必须 > 0")

        # 集群调度
        if sc.drone_count < 2:
            errors.append(
                f"cluster_scheduler.drone_count 至少为 2（赛题二需 ≥2 机协同盯防），"
                f"当前值: {sc.drone_count}"
            )
        if sc.coop_k < 1 or sc.coop_k > sc.drone_count:
            errors.append(
                f"cluster_scheduler.coop_k 需在 [1, {sc.drone_count}] 之间，"
                f"当前值: {sc.coop_k}"
            )
        if sc.real_target_count < 1:
            errors.append("cluster_scheduler.real_target_count 必须 ≥ 1")
        if sc.destroy_time_s <= 0:
            errors.append("cluster_scheduler.destroy_time_s 必须 > 0")

        # 仿真
        if sim.duration_s <= 0:
            errors.append("simulation.duration_s 必须 > 0")

        if errors:
            msg = "配置校验失败:\n  - " + "\n  - ".join(errors)
            raise ValueError(msg)

    # ------------------------------------------------------------------
    # 便捷方法
    # ------------------------------------------------------------------

    def to_dict(self) -> Dict[str, Any]:
        """将当前配置导出为字典（用于序列化/日志）。"""
        if self._config is None:
            return {}
        result: Dict[str, Any] = {}
        for section_name in self._config.__dataclass_fields__:
            section = getattr(self._config, section_name)
            result[section_name] = {
                f.name: getattr(section, f.name)
                for f in section.__dataclass_fields__.values()
            }
        return result

    def reset(self) -> None:
        """重置配置（仅用于测试）。"""
        self._config = None
        self._config_path = ""
        self._raw = {}

    def reload(self) -> AlgorithmConfig:
        """按上次路径重新加载（适用于配置热更新场景）。"""
        if not self._config_path:
            raise RuntimeError("尚未加载过配置，请先调用 load()")
        return self.load(self._config_path)


# ============================================================================
# 模块级便捷入口
# ============================================================================

def get_config() -> AlgorithmConfig:
    """返回当前全局配置实例（未加载时自动使用默认值）。"""
    return ConfigManager().config


def load_config(config_path: Optional[str] = None) -> AlgorithmConfig:
    """加载配置并返回。

    这是 main.py 等入口文件推荐使用的便捷函数。

    Args:
        config_path: YAML 路径，None 则用默认路径。

    Returns:
        AlgorithmConfig 实例。
    """
    return ConfigManager().load(config_path)
