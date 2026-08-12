"""
红枫2026 赛题2 coop_decoy —— 阶段2三机协同智能体（模块化重构版）

目录结构:
    drone_agent.py                    ← 智能体主体（只做模块编排，不写业务算法）
    search_track/
    ├─ communication/main.py          ← CommHandler（通信解析 + 指令构建）
    ├─ vision_detect/main.py          ← YOLODetector + get_detect_result + EMATracker + DecoyClassifier
    │                                  （YOLO检测/像素→经纬度坐标转换 + EMA滤波 + 诱饵判别）
    ├─ cluster_scheduler/main.py     ← CooperativeCoordinator（K=2 协同分配）
    └─ motion_control/main.py        ← geo_utils + SearchController + TrackController（航点生成）

数据流:
    obs.self.photo → vision_detect.get_detect_result()（YOLO推理 + 坐标转换）
    → sensor() 返回 List[Detection] → SDK 注入 obs.self.detection
    → vision_detect(EMATracker + DecoyClassifier)
    → cluster_scheduler(CooperativeCoordinator)
    → motion_control(SearchController + TrackController)
    → communication.build_broadcast() / build_report()

来源: new drone_agent.py 完整业务逻辑（第 1-1767 行）
     全部工具函数、搜索/跟踪/协同/判别算法原封不动保留
"""
from __future__ import annotations

import logging
import os
import sys
import traceback
from typing import Dict, List, Optional, Tuple

# ── 日志系统（仅 stderr，符合 SDK §1.2 不变量：不读写文件） ──────────────────
logger = logging.getLogger("coop_decoy.DroneAgent")
logger.setLevel(logging.INFO)
if not logger.handlers:
    _fmt = logging.Formatter(
        "[DroneAgent] %(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    _sh = logging.StreamHandler(sys.stderr)
    _sh.setFormatter(_fmt)
    logger.addHandler(_sh)

# ── SDK 导入 ────────────────────────────────────────────────────────────────
from competition.sdk.core.commands import (Command, fly_to,
                                           point_gimbal,
                                           set_gimbal_fov)
from competition.sdk.core.observation import Detection
from competition.sdk.scenarios.coop_decoy import CoopAgent
from competition.sdk.scenarios.coop_decoy.observation import CoopObs

# ═══ 四模块 main.py 导入（唯一对外入口，禁止直接访问底层文件） ═══
from .search_track.communication.main import CommHandler
from .search_track.vision_detect.main import (
    EMATracker, DecoyClassifier, get_detect_result, YOLODetector)
from .search_track.cluster_scheduler.main import CooperativeCoordinator
from .search_track.motion_control.main import SearchController, TrackController, geo_utils

logger.info("==== session start ====")


# ══════════════════════════════════════════════════════════════════════════════
# ── 工具函数（来源：coordinator.py:91-169 + ema_filter.py:185-194） ──────────
# 注: 这些工具函数实际已迁移到 motion_control/geo.py，此处保留别名以兼容 new 样本逻辑
# ══════════════════════════════════════════════════════════════════════════════

_BBOX: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (26.982, 124.980), (27.025, 125.020))
_SAFEBOX_MARGIN_M = 600.0

_bbox_inset = geo_utils.bbox_inset
_SAFEBOX = _bbox_inset(_BBOX, _SAFEBOX_MARGIN_M)
_haversine_m = geo_utils.haversine_m
_bearing_deg = geo_utils.bearing_deg
_clamp_to_safebox = geo_utils.clamp_to_safebox
_point_on_circle = geo_utils.point_on_circle
_partition_centers = geo_utils.partition_centers
_PARTITION_CENTERS = _partition_centers(_BBOX, 3)
_uid_phase = geo_utils.uid_phase
_uid_partition_idx = geo_utils.uid_partition_idx


# ══════════════════════════════════════════════════════════════════════════════
# ── 智能体主体（来源：coordinator.py:380-673，类名改为 DroneAgent） ────────────
# ══════════════════════════════════════════════════════════════════════════════

class DroneAgent(CoopAgent):
    """
    阶段2：三机协同诱饵对抗智能体

    功能：
      - 扇区分区搜索（三机各搜索不同经度子区域）
      - R:/T:/C: 机间通信协议
      - K=2 双机 20 秒协同摧毁
      - 槽位分离安全避让（北 200m / 南 200m）
      - 多特征投票诱饵判别（速度方差 + 方向方差 + 位移 + 平均速度）
    """

    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"

    def __init__(self, my_uid: str):
        """构造智能体。SDK §1.1 要求必须显式实现 __init__。"""
        super().__init__(my_uid)
        logger.info("[uid=%s] DroneAgent.__init__", my_uid)

        # ═══ 模块实例化（drone_agent.py 只做编排，不写业务算法） ═══
        self._comm = CommHandler(my_uid)
        self._search_ctrl = SearchController(my_uid)
        self._track_ctrl = TrackController(my_uid)
        self._coord = CooperativeCoordinator(my_uid, k=2)

    def configure(self, config) -> None:
        """读取静态任务/算法参数，整局不变。SDK §1.1 要求可选实现。"""
        # 先调用基类 configure（确保基类初始化不丢失）
        try:
            super().configure(config)
        except AttributeError:
            pass  # 基类未实现 configure 时忽略

        # 搜索几何参数（高度锁500m，fly_to的alt参数被引擎忽略）
        self._search_radius: float = 300.0    # 螺旋最大搜索半径（m），原 700→300 避免盘旋过大
        self._growth: float = 15.0            # 每圈向外扩张步长（m），原 50→15 保证相机覆盖
        self._ang_speed: float = 30.0         # 角速度（°/s）
        self._sweep_period: float = 4.0       # 云台俯仰扫摆周期
        self._pitch_min: float = -60.0        # 俯仰角下限
        self._pitch_max: float = -30.0        # 俯仰角上限
        # 视场角（上限 50°，符合 SDK-API §4 赛规）
        self._track_fov: float = 30.0         # 跟踪时用小 FOV 提高精度
        self._search_fov: float = 50.0        # 搜索时用大 FOV 扩大覆盖
        # 验证阶段参数
        self._verify_timeout: float = 8.0
        self._verify_warmup: float = 3.5          # 从 4.0 降到 3.5，缩短预热时间
        self._verify_speed_confirm: float = 3.0   # 真目标最低速度（m/s），真目标约 5~15 m/s
        self._verify_speed_reject: float = 1.0    # 低于此值直接拒绝（速度计算修复后，静止诱饵应接近 0）
        self._verify_speed_max: float = 15.0      # 真目标最高速度（m/s），超过此值大概率是高速诱饵
        self._verify_reject_min_t: float = 3.0
        self._ema_alpha: float = 0.3
        self._decoy_alpha: float = 0.25  # 诱饵判别器 alpha
        # 跟踪参数
        self._dwell_target: float = 20.0
        self._dwell_grace: float = 2.0
        self._track_timeout: float = 90.0
        self._loiter_close: float = 250.0
        # 通信周期（T消息约0.33Hz，R/C冷却3s，避免3机同时TRACK触及4Hz限速）
        self._status_period: int = 30
        self._r_cooldown: float = 3.0
        # 多机网格扫描参数（三机各负责一个经度条带，条带内做南北蛇形扫描）
        self._multi_search: bool = True
        # 网格扫描线间距（米）：FOV=50° 高度 500m 地面覆盖直径≈466m，取 400m 保证重叠
        self._grid_scan_spacing_m: float = 400.0
        # 全局地图中心（用于 idle 盘旋，后续首次decide时从briefing更新）
        self._map_center_lat: float = (26.982 + 27.025) / 2.0
        self._map_center_lon: float = (124.980 + 125.020) / 2.0
        self._map_center_initialized: bool = False
        # 单机盘旋跟踪参数（来源：tracker.py:39-48）
        self._loiter_radius: float = 450.0  # 增大到 450m，避免追近诱饵扣分（<200m 扣分）
        self._turn_direction: str = "right"
        self._multi_loiter_radius: float = 450.0  # 多机盘旋半径，确保安全距离 >200m
        # 自研 YOLO 识别配置（SDK 选择②：sensor() 返回自研 Detection 列表）
        # 检测与坐标转换全部委托 vision_detect 模块（detect.py），此处只做实例化
        self._yolo_warned: bool = False
        self._init_yolo_detector(config)

        # 运行时状态（由 reset() 初始化）
        self._init_runtime_state()
        logger.info("[uid=%s] DroneAgent.configure completed", self.my_uid)

    def _init_yolo_detector(self, config) -> None:
        """
        初始化自研 YOLO 检测器（SDK 选择②：覆盖 sensor() 返回自研 Detection）。
        - 模型路径优先取 config.perception.yolo.model_path（兼容 dict/对象），
          fallback 到包内相对路径 weights/best.pt
        - 检测器内部容错：模型加载失败/推理超时 → 自动降级，
          get_detect_result() 返回 None/[] → SDK 自动回退默认识别器，
          比赛不中断（SDK-API §2.4 容错语义）
        """
        model_path: Optional[str] = None
        imgsz: int = 640
        conf: float = 0.25
        if isinstance(config, dict):
            perception = config.get("perception") or {}
            yolo_cfg = perception.get("yolo") or {}
            model_path = yolo_cfg.get("model_path")
            imgsz = int(yolo_cfg.get("imgsz", 640))
            conf = float(yolo_cfg.get("conf", 0.25))
        else:
            perception = getattr(config, "perception", None)
            yolo_cfg = getattr(perception, "yolo", None) if perception is not None else None
            if yolo_cfg is not None:
                model_path = getattr(yolo_cfg, "model_path", None)
                imgsz = int(getattr(yolo_cfg, "imgsz", 640))
                conf = float(getattr(yolo_cfg, "conf", 0.25))

        if not model_path:
            # fallback：包内相对路径（提交包 coop_decoy/weights/best.pt）
            # __file__ = coop_decoy/drone_agent.py → dirname 一次即 coop_decoy/
            pkg_root = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(pkg_root, "weights", "best.pt")
            # 若包内不存在（如直接以脚本方式运行），回退到仓库根 weights/
            if not os.path.isfile(model_path):
                alt = os.path.join(
                    os.path.dirname(pkg_root), "weights", "best.pt")
                if os.path.isfile(alt):
                    model_path = alt

        self._yolo_detector = YOLODetector(
            model_path=model_path, imgsz=imgsz, conf=conf)
        logger.info("[uid=%s] YOLO detector inited: %s (imgsz=%d conf=%.2f)",
                    self.my_uid, model_path, imgsz, conf)

    def _init_runtime_state(self) -> None:
        """初始化运行时可变状态（configure 和 reset 共用）。"""
        self._t: float = 0.0
        self._tick: int = 0
        self._region = _PARTITION_CENTERS[_uid_partition_idx(self.my_uid)]
        self._phase: float = _uid_phase(self.my_uid)
        self._state = self.SEARCH
        self._candidate: Optional[Tuple[float, float]] = None
        self._ema = EMATracker(self._ema_alpha)
        self._decoy_clf = DecoyClassifier(self._decoy_alpha)
        self._verify_t: float = 0.0
        self._dwell_time: float = 0.0
        self._last_det_tick: float = -1e9
        self._last_det_pos: Optional[Tuple[float, float]] = None  # 最新检测原始坐标，用于目指上报
        self._track_t: float = 0.0
        self._last_report_t: float = -1e9
        self._known_decoys: Dict[str, Tuple[float, float]] = {}  # stable_id → (lat, lon)，自动去重
        self._track_target_id: Optional[str] = None
        self._track_slot: int = 0  # 多机跟踪槽位
        # decide 调用计数器（日志节流）
        self._decide_call_count: int = 0
        self._last_track_exit_t: float = -1e9  # 从 TRACK 退出的时间戳，用于 SEARCH 冷却
        # photo 图像调试统计
        self._photo_recv_count: int = 0
        self._photo_miss_count: int = 0
        self._photo_last_log_t: float = -1e9
        self._photo_log_interval: float = 30.0  # 每 30s 输出一次 photo 统计
        # 多机网格扫描运行时状态
        self._grid_scan_sector: Tuple[float, float, float, float] = (0, 0, 0, 0)
        self._grid_scan_phase: float = 0.0
        self._grid_scan_inited: bool = False
        # 目指上报统计
        self._report_count: int = 0
        self._report_last_log_t: float = -1e9
        self._report_log_interval: float = 30.0  # 每 30s 输出一次上报统计
        # 任务完成速度标记
        self._all_destroyed: bool = False
        # 最后一次检测到真实目标的时间（仅真目标更新，诱饵不更新）
        self._last_detected_time: float = -1e9

    def reset(self) -> None:
        """重置智能体状态（每回合开始调用），带容错保护。SDK §1.1 要求可选实现。"""
        logger.info("[uid=%s] reset() called", self.my_uid)
        try:
            self._init_runtime_state()
            self._comm.reset()
            self._coord.reset()
            self._search_ctrl.reset()
            self._track_ctrl.reset()
            logger.info("[uid=%s] reset() completed", self.my_uid)
        except Exception:
            logger.error("[uid=%s] reset() FAILED:\n%s",
                         self.my_uid, traceback.format_exc())
            raise

    # ── 搜索命令快捷生成（委托子模块） ──

    def _make_search_cmds(self) -> List[Command]:
        """生成搜索命令列表。委托 SearchController 生成螺旋搜索 + 云台扫描。"""
        return self._search_ctrl.make_search_cmds(self._t)

    def _make_idle_cmds(self) -> List[Command]:
        """全歼后低功耗模式。委托 SearchController 生成低速大半径盘旋。"""
        return self._search_ctrl.make_idle_cmds(self._map_center_lat, self._map_center_lon)

    # ── 图像传感器回调（SDK 选择②：自研 YOLO 识别） ──
    # 返回语义（SDK-API §2.4/§2.6）：
    #   None                  → 回退 SDK 默认识别器（无画面 / 模型不可用 / 异常容错）
    #   [Detection(...)]      → 使用自研识别结果（list[0] 作为 obs.self.detection 主检测）
    #   []                    → 明确本帧无目标（Detection(detected=False)）

    def sensor(self, obs: CoopObs, dt: float) -> Optional[List[Detection]]:
        """
        图像传感器回调：读取 obs.self.photo（PNG bytes）→ 委托 vision_detect
        get_detect_result() 完成 YOLO 推理 + 像素→经纬度坐标转换，返回自研
        Detection 列表，平台以 list[0] 填充 obs.self.detection 传入 decide()。

        视觉模块内部已做超时监控/降级，这里只做编排与调试统计。
        """
        try:
            photo = obs.self.photo
            if photo is not None:
                self._photo_recv_count += 1
                if self._photo_recv_count <= 5 or self._photo_recv_count % 300 == 0:
                    img_size = len(photo) if hasattr(photo, '__len__') else '?'
                    logger.info("[uid=%s] sensor() photo#%d size=%s bytes t=%.1f",
                                self.my_uid, self._photo_recv_count, img_size, self._t)
            else:
                self._photo_miss_count += 1

            # 定期输出 photo 接收率统计
            if self._t - self._photo_last_log_t >= self._photo_log_interval:
                total = self._photo_recv_count + self._photo_miss_count
                if total > 0:
                    rate = self._photo_recv_count / total * 100.0
                    logger.info("[uid=%s] photo stats: recv=%d miss=%d rate=%.1f%%",
                                self.my_uid, self._photo_recv_count, self._photo_miss_count, rate)
                self._photo_last_log_t = self._t

            # ── 自研识别主流程：检测 + 坐标转换全部委托 vision_detect 模块 ──
            # 返回语义（SDK-API §2.4/§2.6）:
            #   None → 回退 SDK 默认识别器（无画面/解码失败/检测器不可用/异常容错）
            #   []   → 明确本帧无目标
            #   [...]→ 自研检测结果（list[0] 为主检测）
            return get_detect_result(photo, obs.self, self._yolo_detector)
        except Exception:
            # 异常容错：回退默认识别器，避免帧中断（SDK-API §2.4）
            if not self._yolo_warned:
                self._yolo_warned = True
                logger.warning("[uid=%s] sensor() inference error -> fallback to "
                               "default detector (logged once): %s",
                               self.my_uid, traceback.format_exc())
            else:
                logger.debug("[uid=%s] sensor() inference error (non-fatal): %s",
                             self.my_uid, traceback.format_exc())
            return None

    # ── 主决策函数 ──

    def decide(self, obs: CoopObs, dt: float) -> List[Command]:
        """每帧调用，生成当前控制指令（带崩溃降级保护）。SDK §1.1 要求必须实现。"""
        self._decide_call_count += 1
        call_id = self._decide_call_count
        self._tick += 1
        self._t += dt
        sim_t = self._t

        # 前 5 帧 + 每 300 帧打一次状态
        if call_id <= 5 or call_id % 300 == 0:
            logger.info("[uid=%s] decide#%d state=%s t=%.1f",
                        self.my_uid, call_id, self._state, sim_t)

        # ── 惰性初始化：首次 decide 时从 briefing 读取地图中心和扇区中心 ──
        if not self._map_center_initialized:
            ma = obs.briefing.mission_area
            if ma is not None:
                self._map_center_lat = (ma.lat_min + ma.lat_max) / 2.0
                self._map_center_lon = (ma.lon_min + ma.lon_max) / 2.0
            self._map_center_initialized = True

        try:
            det = obs.self.detection
            cmds: List[Command] = []

            # ── 视觉判别增强：YOLO target_type → decoy_clf 投票信号 ──
            self._decoy_clf.apply_yolo_hint(det)

            # ── 120s 无真目标检测：全部无人机返回地图中心，重置搜索状态 ──
            if (self._last_detected_time > 0
                    and sim_t - self._last_detected_time > 120.0):
                logger.info("[uid=%s] 120s no real target → return to map center, reset search",
                            self.my_uid)
                # 重置协同调度器（清空所有目标）
                self._coord.reset()
                self._track_target_id = None
                self._candidate = None
                self._ema.reset()
                self._decoy_clf.reset()
                self._dwell_time = 0.0
                self._last_report_t = -1e9
                self._last_det_pos = None
                self._last_track_exit_t = -1e9
                self._last_detected_time = sim_t  # 重置计时器
                # 切换到 SEARCH，返回地图中心
                self._state = self.SEARCH
                center_lat, center_lon = _clamp_to_safebox(
                    self._map_center_lat, self._map_center_lon)
                return [
                    fly_to(center_lat, center_lon, speed=22.0),
                    point_gimbal(0.0, -45.0),
                    set_gimbal_fov(self._search_fov),
                ]

            # ── 任务完成速度感知：利用 score_view 获取摧毁进度 ──
            n_destroyed = 0
            sv = obs.briefing.score_view
            if sv is not None:
                n_destroyed = sv.n_destroyed
                # 全歼标志：3个目标全部摧毁
                if n_destroyed >= 3 and not getattr(self, '_all_destroyed', False):
                    self._all_destroyed = True
                    logger.info("[uid=%s] ALL TARGETS DESTROYED at t=%.1f",
                                self.my_uid, sim_t)

            # ═══ 数据流1: 解析队友通信 ═══
            self._coord.ingest_comms(obs.comm_inbox, sim_t)

            # ── 全歼后：低功耗搜索（低速+大半径，节省边界扣分） ──
            if getattr(self, '_all_destroyed', False):
                return self._make_idle_cmds()

            # ── 搜索状态 ──
            if self._state == self.SEARCH:
                cmds = self._handle_search(obs, det, sim_t)

            # ── 验证状态（真假目标判别） ──
            elif self._state == self.VERIFY:
                cmds = self._handle_verify(obs, det, dt, sim_t)

            # ── 跟踪状态（协同摧毁） ──
            elif self._state == self.TRACK:
                # 提前检查：目标是否已被队友摧毁（收到 C 广播）
                # 若本机 dwell < 5s（非主要贡献者），立即释放回 SEARCH
                if (self._track_target_id is not None
                        and self._coord.is_target_destroyed(self._track_target_id)):
                    logger.info("[uid=%s] TRACK→SEARCH target=%s already destroyed by teammate, releasing",
                                self.my_uid, self._track_target_id)
                    self._coord.unregister_discoverer(self._track_target_id, self.my_uid)
                    self._state = self.SEARCH
                    self._track_target_id = None
                    self._candidate = None
                    self._ema.reset()
                    self._decoy_clf.reset()
                    self._dwell_time = 0.0
                    self._last_report_t = -1e9
                    self._last_det_pos = None
                    self._last_track_exit_t = sim_t
                    cmds = self._make_search_cmds()
                else:
                    cmds = self._handle_track(obs, det, dt, sim_t)

            # 兜底返回搜索指令
            if not cmds:
                cmds = self._make_search_cmds()

            # ── 目指上报：TRACK 状态有最新检测坐标即上报
            # 优先用最新检测原始坐标（_last_det_pos），fallback 到 EMA 平滑值
            # 传入 target_id 作为 audit label（不影响评分，但方便调试）
            report_pos: Optional[Tuple[float, float]] = None  # 提前声明，避免作用域问题
            if (self._state == self.TRACK
                    and self._track_target_id is not None):
                report_pos = self._last_det_pos if self._last_det_pos is not None else self._ema.value
                if report_pos is not None and sim_t - self._last_report_t >= 1.0:
                    self._last_report_t = sim_t
                    self._report_count += 1
                    cmds.append(self._comm.build_report(report_pos[0], report_pos[1],
                                                        target_id=self._track_target_id))

            # 定期输出目指上报统计
            if (self._state == self.TRACK
                    and self._report_count > 0
                    and sim_t - self._report_last_log_t >= self._report_log_interval):
                self._report_last_log_t = sim_t
                # 用 EMA 值或最后检测坐标作为统计位置（report_pos 本帧可能为 None）
                log_pos = self._ema.value or self._last_det_pos
                logger.info("[uid=%s] report_target stats: count=%d tgt=%s pos=(%.5f,%.5f) dwell=%.1f",
                            self.my_uid, self._report_count, self._track_target_id,
                            log_pos[0] if log_pos else 0,
                            log_pos[1] if log_pos else 0,
                            self._dwell_time)

            return cmds

        except Exception:
            logger.exception("[uid=%s] decide#%d CRASH", self.my_uid, call_id)
            # 崩溃降级：尝试返回搜索指令，再失败则返回空列表（SDK §1.2 合法）
            try:
                return self._make_search_cmds()
            except Exception:
                return []

    # ── 状态处理 ──

    def _handle_search(self, obs, det, sim_t: float) -> List[Command]:
        """处理 SEARCH 状态。"""
        # 冷却期：刚从 TRACK 退出后至少等 3 秒才能重新分配，避免反复跳同一目标
        # 冷却期内：不做任何目标分配和检测，纯搜索飞行
        last_exit = getattr(self, '_last_track_exit_t', -1e9)
        in_cooldown = (sim_t - last_exit) < 3.0
        if in_cooldown:
            return self._make_search_cmds()

        # 任务分配制：优先加入已有队友在跟踪的目标（K=2 配对）
        # compute_assignment 按 peer_count=1 优先 + peer_count=0 次之 + distance 升序
        # 注意：冷却期之后才执行，防止刚退出 TRACK 又被 compute_assignment 拉回去
        assigned_tgt = self._coord.compute_assignment(
            obs.self.lat, obs.self.lon, self._track_target_id)
        if assigned_tgt is not None:
            # 安全检查：如果该目标 peer_dwell=0（无队友在跟踪），
            # 说明这是来自 R 广播但未经双机验证的目标，需要自己先 VERIFY
            peer_dwell = self._coord.peer_dwell(assigned_tgt)
            if peer_dwell < 1.0:
                # 队友还没开始积累 dwell，可能是刚 R 广播但未确认
                # 不要盲跳 TRACK，而是对该位置做一次快速 VERIFY
                tgt_pos = self._coord.target_pos(assigned_tgt)
                if tgt_pos is not None:
                    # 跳过已知假目标
                    decoy_positions = list(self._known_decoys.values())
                    near_decoy = any(
                        _haversine_m(tgt_pos[0], tgt_pos[1], d[0], d[1]) < 150.0
                        for d in decoy_positions) if decoy_positions else False
                    if near_decoy:
                        logger.info("[uid=%s] SEARCH skip assigned tgt=%s (known decoy)",
                                    self.my_uid, assigned_tgt)
                        return self._make_search_cmds()
                    logger.info("[uid=%s] SEARCH→VERIFY (assigned, peer_dwell=0) tgt=%s",
                                self.my_uid, assigned_tgt)
                    self._state = self.VERIFY
                    self._candidate = tgt_pos
                    self._ema = EMATracker(self._ema_alpha)
                    self._decoy_clf = DecoyClassifier(self._decoy_alpha)
                    self._verify_t = 0.0
                    return self._make_search_cmds()
            logger.info("[uid=%s] SEARCH→TRACK (assigned) tgt=%s peer_dwell=%.1f",
                        self.my_uid, assigned_tgt, peer_dwell)
            self._track_target_id = assigned_tgt
            self._state = self.TRACK
            self._dwell_time = 0.0
            self._track_t = 0.0
            self._last_det_tick = sim_t
            # 用 EMA 值或目标位置初始化 _last_det_pos，确保首帧就能上报
            tgt_pos = self._coord.target_pos(assigned_tgt)
            self._last_det_pos = self._ema.value or tgt_pos
            # 立即进入 TRACK 本帧处理
            return self._handle_track(obs, det, 0.1, sim_t) or self._make_search_cmds()

        if det.detected and det.target_lat is not None:
            # 跳过已知假目标附近（用 stable_id 去重后的坐标列表）
            decoy_positions = list(self._known_decoys.values())
            near_decoy = any(
                _haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                for d in decoy_positions) if decoy_positions else False
            if not near_decoy:
                logger.info("[uid=%s] SEARCH→VERIFY target=(%.5f,%.5f)",
                            self.my_uid, det.target_lat, det.target_lon)
                self._state = self.VERIFY
                self._candidate = (det.target_lat, det.target_lon)
                self._ema = EMATracker(self._ema_alpha)
                self._decoy_clf = DecoyClassifier(self._decoy_alpha)
                self._verify_t = 0.0

        return self._make_search_cmds()

    def _handle_verify(self, obs, det, dt: float, sim_t: float) -> List[Command]:
        """处理 VERIFY 状态。"""
        self._verify_t += dt
        tgt = self._candidate
        if det.detected and det.target_lat is not None:
            d = _haversine_m(det.target_lat, det.target_lon,
                             tgt[0], tgt[1])
            # 首帧门槛收窄到 300m，防止不同实体混入同一个 EMA 缓冲区
            # 后续帧收紧到 200m，确保关联的是同一目标
            ema_empty = self._ema.value is None
            threshold = 300.0 if ema_empty else 200.0
            if d < threshold:
                self._ema.append(det.target_lat, det.target_lon, sim_t)
                self._decoy_clf.update(det.target_lat, det.target_lon, dt, sim_t)
                self._candidate = self._ema.value
                tgt = self._candidate

        # 基础速度判别：真目标速度应在合理范围（3~15 m/s）
        # 速度计算修复后，速度值应准确。真目标约 5~15 m/s
        speed = self._ema.speed_mps()
        speed_ok = (self._verify_speed_confirm <= speed <= self._verify_speed_max)
        speed_confirmed = (self._verify_t >= self._verify_warmup and speed_ok)
        # 拒绝条件放宽：速度 < 1.0（完全静止）或 > 20（高速诱饵）才拒绝
        speed_rejected = (self._verify_t >= self._verify_reject_min_t
                          and (speed < 1.0
                               or speed > 20.0))

        # 多特征投票判别（decoy_classifier.py 增强）
        # 降低门槛：VERIFY 期间帧数有限，conf≥0.55 即可接受
        multi_confirmed = (
            self._verify_t >= self._verify_warmup
            and self._decoy_clf.is_real_target
            and self._decoy_clf.confidence >= 0.55
        )
        multi_rejected = (
            self._verify_t >= self._verify_reject_min_t
            and not self._decoy_clf.is_real_target
            and self._decoy_clf.confidence < 0.30
        )

        # 确认条件：OR 逻辑 —— 速度判别 OR 多特征投票任一通过即可
        # 增加安全网：如果多特征明确拒绝（multi_rejected），则不能确认
        confirmed = (speed_confirmed or multi_confirmed) and not multi_rejected

        # 拒绝条件：
        # 1. 速度超限（>_verify_speed_max*1.25≈22.5m/s）或极慢（<1.5m/s）→ 立即拒绝
        #    但必须有足够的 EMA 样本（暖机期）才检查 speed_extreme，避免 EMA 为空时 speed=0.0 误杀
        # 2. 速度判别明确拒绝（speed_rejected）
        # 3. 多特征投票明确拒绝
        # 4. 超时后未确认
        ema_has_data = self._ema.value is not None and len(self._ema.raw_history) >= 4
        speed_extreme = ema_has_data and (speed > 20.0 or speed < 0.5)
        rejected = (speed_extreme
                    or speed_rejected
                    or multi_rejected
                    or (self._verify_t >= self._verify_reject_min_t
                        and not speed_confirmed and not multi_confirmed))

        if confirmed:
            # 确认真目标，注册并召唤队友
            lat, lon = tgt

            # K=2 配对检查：先查 nearby 目标是否已满员，防止三机追同一目标
            # 使用 find_nearby + target_discoverers_count 跨 ID 检查，避免 ID 抖动导致漏检
            nearby_id = self._coord.find_nearby(lat, lon)
            current_peers = 0
            if nearby_id is not None:
                current_peers = self._coord.target_discoverers_count(nearby_id)
            if current_peers >= 2:
                logger.info("[uid=%s] VERIFY→SEARCH tgt=(%.5f,%.5f) nearby=%s already has %d peers, skipping",
                            self.my_uid, lat, lon, nearby_id, current_peers)
                # 标记为已知诱饵避免反复尝试
                sid = self._coord.stable_id(lat, lon)
                if len(self._known_decoys) < 20:
                    self._known_decoys[sid] = (lat, lon)
                self._coord.confirm_decoy(lat, lon)
                self._state = self.SEARCH
                self._candidate = None
                self._ema.reset()
                self._decoy_clf.reset()
                return self._make_search_cmds()

            tgt_id = self._coord.confirm_target(lat, lon)

            # confirm_target 可能 merge 到已有 ID，重新检查 peer_count
            current_peers = self._coord.target_discoverers_count(tgt_id)
            if current_peers >= 2:
                logger.info("[uid=%s] VERIFY→SEARCH tgt=%s already has %d peers (post-merge), skipping",
                            self.my_uid, tgt_id, current_peers)
                sid = self._coord.stable_id(lat, lon)
                if len(self._known_decoys) < 20:
                    self._known_decoys[sid] = (lat, lon)
                self._coord.confirm_decoy(lat, lon)
                self._state = self.SEARCH
                self._candidate = None
                self._ema.reset()
                self._decoy_clf.reset()
                return self._make_search_cmds()

            # 注册本机为发现者
            self._coord.register_discoverer(tgt_id, self.my_uid)
            cmds: List[Command] = []
            if self._coord.need_r_broadcast(tgt_id, sim_t, self._r_cooldown):
                cmds.append(self._comm.build_r_msg(lat, lon, self.my_uid))
                self._coord.mark_r_sent(tgt_id, sim_t)
            logger.info("[uid=%s] VERIFY→TRACK confirmed tgt=%s speed=%.1f multi_ok=%s clf_conf=%.2f peers=%d",
                        self.my_uid, tgt_id, speed, multi_confirmed, self._decoy_clf.confidence, current_peers)
            self._track_target_id = tgt_id
            self._state = self.TRACK
            self._dwell_time = 0.0
            self._track_t = 0.0
            self._last_det_tick = sim_t
            self._last_detected_time = sim_t  # 仅真目标确认时更新，诱饵不更新
            # 初始化 _last_det_pos：用 EMA 值或候选位置，确保首帧就能上报
            self._last_det_pos = self._ema.value or self._candidate
            return cmds  # 返回 R 广播，TRACK 下帧处理

        elif rejected or self._verify_t >= self._verify_timeout:
            # 假目标或超时，放弃并记忆
            if rejected and self._ema.value:
                # 用 stable_id 去重：同一假目标位置不重复记录
                lat, lon = self._ema.value
                sid = self._coord.stable_id(lat, lon)
                if sid not in self._known_decoys:
                    # 上限裁剪：只保留最近 20 条诱饵记录，防止内存泄漏
                    if len(self._known_decoys) >= 20:
                        oldest_key = next(iter(self._known_decoys))
                        del self._known_decoys[oldest_key]
                    self._known_decoys[sid] = (lat, lon)
                self._coord.confirm_decoy(lat, lon)
                logger.info("[uid=%s] VERIFY→SEARCH decoy speed=%.1f spd_ok=%s multi_ok=%s clf_conf=%.2f",
                            self.my_uid, speed, speed_ok, multi_confirmed, self._decoy_clf.confidence)
            elif self._verify_t >= self._verify_timeout:
                logger.info("[uid=%s] VERIFY→SEARCH timeout", self.my_uid)
            self._state = self.SEARCH
            self._candidate = None
            self._ema.reset()
            self._decoy_clf.reset()
            return self._make_search_cmds()

        else:
            # 验证中：飞向目标并保持观察
            tlat, tlon = _clamp_to_safebox(*tgt)
            pan, tilt = geo_utils.los_angles(
                obs.self.lat, obs.self.lon, obs.self.alt,
                obs.self.heading_deg, tgt[0], tgt[1])
            return [
                fly_to(tlat, tlon, speed=22.0,
                       loiter_radius=self._loiter_close),
                point_gimbal(pan, tilt),
                set_gimbal_fov(self._search_fov),  # 验证期用大 FOV 确保不漏检
            ]

    def _handle_track(self, obs, det, dt: float, sim_t: float) -> List[Command]:
        """处理 TRACK 状态（协同摧毁）。"""
        self._track_t += dt
        tgt_id = self._track_target_id
        if tgt_id is None:
            self._state = self.SEARCH
            self._last_track_exit_t = sim_t
            return self._make_search_cmds()

        # 获取目标位置
        tgt_pos = self._coord.target_pos(tgt_id)
        if tgt_pos is None:
            self._state = self.SEARCH
            self._last_track_exit_t = sim_t
            return self._make_search_cmds()

        # 自身检测更新目标位置（TRACK 状态也持续更新）
        # 注意：直接用 tgt_id 更新位置，避免 confirm_target 内部 merge 导致 ID 抖动
        # 放宽到 600m：诱饵高速移动时不容易脱锁，保证持续 dwell 积累
        if det.detected and det.target_lat is not None:
            d = _haversine_m(det.target_lat, det.target_lon,
                             tgt_pos[0], tgt_pos[1])
            if d < 600.0:
                self._coord.update_target_pos(tgt_id, det.target_lat, det.target_lon)
                tgt_pos = (det.target_lat, det.target_lon)

        # 积累 dwell 时间 + 更新 EMA 位置（只有自己检测到时才积累/更新）
        tracking = det.detected and det.target_lat is not None and \
            _haversine_m(det.target_lat, det.target_lon,
                         tgt_pos[0], tgt_pos[1]) < 600.0
        if tracking:
            gap = sim_t - self._last_det_tick
            if gap <= self._dwell_grace + dt:
                self._dwell_time += dt
            else:
                # 断连过久，重置
                self._dwell_time = dt
            self._last_det_tick = sim_t
            # 记录最新检测原始坐标，用于目指上报（避免 EMA 滞后）
            self._last_det_pos = (det.target_lat, det.target_lon)
            # TRACK 期间持续更新 EMA，确保 report_target 使用最新平滑位置
            self._ema.append(det.target_lat, det.target_lon, sim_t)

        cmds: List[Command] = []

        # 定期广播本机状态（基于 sim_t 时间间隔 3s，替代 tick%30 避免帧率波动影响）
        if self._coord.need_t_broadcast(sim_t):
            self._coord.mark_t_broadcast(sim_t)
            cmds.append(self._comm.build_t_msg(tgt_pos[0], tgt_pos[1], self._dwell_time))

        # 检查协同摧毁条件 —— K=2 要求两机同时照射 20s
        # peer_dwell 来自队友 T 广播，取附近目标的最大值作为协同参考
        # 必须本机 dwell >= 20s 且队友也在跟踪（peer_dwell > 0）才判定摧毁
        # 纯单机照射 20s 不够，引擎层面 K=2 不认可
        peer_dwell = self._coord.peer_dwell(tgt_id)
        my_dwell_ok = self._dwell_time >= self._dwell_target
        peer_active = peer_dwell >= 1.0  # 队友至少积累了 1s，说明在跟踪
        destroyed_by_env = my_dwell_ok and peer_active

        # 定期日志：输出协同状态
        if self._tick % 300 == 0 or (my_dwell_ok and not peer_active):
            logger.info("[uid=%s] TRACK dwell=%.1f peer=%.1f tgt=%s my_ok=%s peer_ok=%s",
                        self.my_uid, self._dwell_time, peer_dwell,
                        tgt_id, my_dwell_ok, peer_active)

        if destroyed_by_env:
            if self._coord.need_c_broadcast(tgt_id, sim_t):
                cmds.append(self._comm.build_c_msg(tgt_pos[0], tgt_pos[1]))
                self._coord.mark_c_sent(tgt_id, sim_t)
            self._coord.mark_destroyed(tgt_id)
            self._coord.unregister_discoverer(tgt_id, self.my_uid)
            logger.info("[uid=%s] TRACK→SEARCH destroyed tgt=%s dwell=%.1f",
                        self.my_uid, tgt_id, self._dwell_time)
            self._state = self.SEARCH
            self._track_target_id = None
            self._dwell_time = 0.0
            self._last_report_t = -1e9
            self._last_det_pos = None
            self._last_track_exit_t = sim_t
            cmds.extend(self._make_search_cmds())
            return cmds

        # 超时判断：如果很长时间没有进展，放弃
        # 放宽 timeout：允许 peer summon 后飞过来（需要更长时间）
        if self._track_t >= self._track_timeout:
            logger.info("[uid=%s] TRACK→SEARCH timeout tgt=%s dwell=%.1f",
                        self.my_uid, tgt_id, self._dwell_time)
            if self._dwell_time < 5.0:
                # dwell 太少，标记为假目标
                self._coord.confirm_decoy(*tgt_pos)
            self._coord.unregister_discoverer(tgt_id, self.my_uid)
            self._state = self.SEARCH
            self._track_target_id = None
            self._dwell_time = 0.0
            self._last_report_t = -1e9  # 补全状态重置
            self._last_det_pos = None
            self._last_track_exit_t = sim_t  # 记录退出时间，用于 SEARCH 冷却
            cmds.extend(self._make_search_cmds())
            return cmds

        # 飞向目标：始终使用盘旋环绕模式，避免追近诱饵扣分（<200m）
        tgt_lat, tgt_lon = tgt_pos
        if self._multi_search:
            slot = self._coord.my_slot(tgt_id)
            aim_lat, aim_lon = self._track_ctrl.get_multi_loiter_waypoint(
                obs.self.lat, obs.self.lon, tgt_lat, tgt_lon, slot)
        else:
            aim_lat, aim_lon = self._track_ctrl.get_single_loiter_waypoint(
                obs.self.lat, obs.self.lon, tgt_lat, tgt_lon)
        aim_lat, aim_lon = _clamp_to_safebox(aim_lat, aim_lon)
        # not tracking 时用较快速度逼近，tracking 时用较慢速度盘旋
        approach_speed = 30.0 if not tracking else 22.0
        cmds.append(fly_to(aim_lat, aim_lon, speed=approach_speed,
                           loiter_radius=self._loiter_close))

        # 云台始终对准目标位置
        pan, tilt = geo_utils.los_angles(
            obs.self.lat, obs.self.lon, obs.self.alt,
            obs.self.heading_deg,
            tgt_pos[0], tgt_pos[1])
        cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(self._track_fov))

        return cmds
