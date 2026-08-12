"""
vision_detect/detect.py —— YOLO 目标检测 + 像素坐标→仿真世界坐标转换

职责（与智能体解耦，只做"检测 + 坐标转换"，不含跟踪/诱饵判别/协同决策）:
    1. 接收相机图像帧（numpy 数组 HxWx3，或 PNG bytes）做 YOLO 推理
    2. 输出目标列表：类别 / 像素 bbox(x1,y1,x2,y2) / 置信度
    3. 用 FOV + 图像尺寸 + 云台姿态 + 本机位姿 反算目标经纬度
       （pan_tilt_to_latlon，来自赛题 SDK）
    4. 推理耗时监控 + 连续超时自动降级（不阻塞 Agent 主线程）

参赛注意事项:
    - 只写推理代码，不提交 pt 权重文件；模型路径由
      config/algorithm.yaml → perception.yolo.model_path 指定（或包内 weights/best.pt 兜底）
    - import 只使用赛题 SDK（competition.sdk.*），不依赖 CoopAgent 本体，
      本模块供 CoopAgent 子类的 sensor() 回调调用
    - 任何失败/超时均返回空结果或 None，由上层回退 SDK 默认识别器，比赛不中断
"""
from __future__ import annotations

import logging
import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

# numpy/cv2/ultralytics 均为延迟导入（降低 SDK 顶层加载负担）

# 赛题 SDK（视觉模块唯一依赖的 SDK 组件）
from competition.sdk.core.observation import Detection
from competition.sdk.core.perception.bbox_to_latlon import pan_tilt_to_latlon

logger = logging.getLogger("coop_decoy.vision_detect.detect")
if not logger.handlers:
    _fmt = logging.Formatter("[VisionDetect] %(asctime)s %(levelname)s %(message)s",
                             datefmt="%H:%M:%S")
    _sh = logging.StreamHandler(sys.stderr)
    _sh.setFormatter(_fmt)
    logger.addHandler(_sh)

# ── 默认配置常量（可在实例化 YOLODetector 时覆盖） ──────────────────────────
DEFAULT_IMGSZ: int = 640                # YOLO 输入尺寸
DEFAULT_CONF: float = 0.25              # 置信度阈值（可配置）
DEFAULT_INFER_TIMEOUT_S: float = 0.15   # 单帧推理耗时预算（仿真控制频率约 10Hz → 0.1s/帧）
DEFAULT_MAX_CONSECUTIVE_TIMEOUT: int = 3  # 连续超时多少次进入降级态
DEFAULT_DEGRADE_SKIP_FRAMES: int = 30   # 降级后跳过多少帧再尝试恢复（约 3s）
_FOV_V_MIN: float = 5.0                 # 垂直视场角钳制下限（防止除零/极端值）
_FOV_V_MAX: float = 50.0                # 垂直视场角钳制上限（SDK-API §4 赛规上限）


@dataclass
class DetBox:
    """
    单个目标检测框（像素坐标系，原点为图像左上角）。

    属性:
        cls      类别 id（int）
        cls_name 类别名（ultralytics names 表映射，缺失时用数字字符串）
        conf     置信度 [0,1]
        x1,y1,x2,y2  像素框左上/右下坐标（float）
    """
    cls: int
    cls_name: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        """框中心 x（像素）。"""
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        """框中心 y（像素）。"""
        return (self.y1 + self.y2) / 2.0

    @property
    def width(self) -> float:
        """框宽（像素）。"""
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        """框高（像素）。"""
        return self.y2 - self.y1


class YOLODetector:
    """
    YOLO 推理器：模型加载 + 前向推理 + 耗时监控 + 超时自动降级。

    超时降级机制（不阻塞 Agent 主线程的方案）:
        - 每次推理用 time.perf_counter() 计时，超过预算计一次"慢帧"
        - 连续多次慢帧 → 进入降级态：暂停推理 N 帧（DEGRADE_SKIP_FRAMES）
          让 CPU 喘息，N 帧后自动尝试恢复
        - 模型加载失败 → self._model=None → available=False → detect() 返回 []
          （上层据此回退 SDK 默认识别器）
    """

    def __init__(
        self,
        model_path: str,
        imgsz: int = DEFAULT_IMGSZ,
        conf: float = DEFAULT_CONF,
        infer_timeout_s: float = DEFAULT_INFER_TIMEOUT_S,
        max_consecutive_timeouts: int = DEFAULT_MAX_CONSECUTIVE_TIMEOUT,
        degrade_skip_frames: int = DEFAULT_DEGRADE_SKIP_FRAMES,
    ) -> None:
        """
        参数:
            model_path   pt 权重路径（不存在/加载失败 → 降级为不可用，不抛异常）
            imgsz        YOLO 输入边长
            conf         置信度阈值
            infer_timeout_s            单帧推理耗时预算（秒）
            max_consecutive_timeouts   连续超时次数阈值
            degrade_skip_frames        降级后跳过的帧数
        """
        self._imgsz: int = int(imgsz)
        self._conf: float = float(conf)
        self._infer_timeout_s: float = float(infer_timeout_s)
        self._max_consecutive_timeouts: int = int(max_consecutive_timeouts)
        self._degrade_skip_frames: int = int(degrade_skip_frames)

        # 运行时降级状态
        self._consecutive_timeouts: int = 0   # 连续超时计数
        self._degrade_skip_left: int = 0      # 剩余跳过帧数（>0 表示处于降级态）
        self._degraded: bool = False          # 是否处于降级态（供上层查询）
        self._degrade_warned: bool = False    # 降级日志只打一次

        self._model = self._load_model(model_path)

    # ── 模型加载（失败不抛异常，置 self._model=None） ──
    def _load_model(self, model_path: str):
        """延迟导入 ultralytics 并加载模型；失败返回 None（上层回退默认识别器）。"""
        try:
            from ultralytics import YOLO  # 延迟导入，降低 SDK 顶层加载负担
            model = YOLO(model_path)
            logger.info("YOLO model loaded: %s", model_path)
            return model
        except Exception:
            logger.warning(
                "YOLO model load FAILED (%s) -> detector unavailable, "
                "fallback to SDK default detector", model_path)
            return None

    # ── 可用性 ──
    @property
    def available(self) -> bool:
        """模型是否可用（False 时上层应回退 SDK 默认识别器）。"""
        return self._model is not None

    @property
    def degraded(self) -> bool:
        """当前是否处于超时降级态（暂停推理中）。"""
        return self._degraded

    # ── 前向推理（含耗时监控 + 超时降级） ──
    def detect(self, img_bgr: np.ndarray) -> List[DetBox]:
        """
        对一帧 BGR 图像（HxWx3 numpy 数组）做 YOLO 推理。

        返回:
            检测框列表（按模型输出顺序，含 cls/conf/bbox），
            无目标 → []；模型不可用/降级/异常 → []（绝不抛异常）
        """
        # 降级态：跳过推理，让出 CPU；跳过帧数用完自动恢复
        if self._degrade_skip_left > 0:
            self._degrade_skip_left -= 1
            if self._degrade_skip_left == 0:
                self._degraded = False
                logger.info("YOLO inference recovered after degradation skip")
            return []

        if self._model is None:
            return []  # 模型不可用 → 无结果（上层回退默认识别器）

        # ── 计时前向推理 ──
        t0 = time.perf_counter()
        try:
            results = self._model.predict(
                img_bgr, imgsz=self._imgsz, conf=self._conf, verbose=False)
        except Exception:
            # 推理异常：记一次慢帧并尝试降级，不向主线程抛异常
            self._on_slow_frame(log=True, reason="inference exception")
            return []
        elapsed_s = time.perf_counter() - t0

        # ── 耗时监控：单帧超预算 → 连续超时计数 → 触发降级 ──
        if elapsed_s > self._infer_timeout_s:
            self._on_slow_frame(
                log=True,
                reason=f"inference {elapsed_s * 1000.0:.0f}ms > budget "
                       f"{self._infer_timeout_s * 1000.0:.0f}ms")
            # 降级态仍返回本帧结果（已算出），只是下一帧起暂停
        else:
            self._consecutive_timeouts = 0  # 恢复正常，清零连续超时

        # ── 解析输出 → DetBox 列表 ──
        boxes: List[DetBox] = []
        for r in results:
            r_boxes = getattr(r, "boxes", None)
            if r_boxes is None or len(r_boxes) == 0:
                continue
            names = getattr(r, "names", None) or {}
            for b in r_boxes:
                conf = float(b.conf[0])
                if conf < self._conf:
                    continue  # 双重过滤（predict 已过滤，此处防御）
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                cls_id = int(b.cls[0]) if (hasattr(b, "cls") and len(b.cls) > 0) else 0
                cls_name = names.get(cls_id, str(cls_id))
                boxes.append(DetBox(cls=cls_id, cls_name=cls_name, conf=conf,
                                    x1=x1, y1=y1, x2=x2, y2=y2))
        return boxes

    # ── 慢帧处理：累计连续超时，达到阈值进入降级态 ──
    def _on_slow_frame(self, log: bool, reason: str) -> None:
        self._consecutive_timeouts += 1
        if log and self._consecutive_timeouts <= self._max_consecutive_timeouts:
            logger.warning("slow frame (%d/%d): %s",
                           self._consecutive_timeouts,
                           self._max_consecutive_timeouts, reason)
        if self._consecutive_timeouts >= self._max_consecutive_timeouts:
            self._degraded = True
            self._degrade_skip_left = self._degrade_skip_frames
            if not self._degrade_warned:
                self._degrade_warned = True
                logger.warning(
                    "YOLO inference DEGRADED: skip next %d frames then retry "
                    "(timeout protection)",
                    self._degrade_skip_frames)

    # ── 像素坐标 → 仿真世界经纬度 ──
    def pixel_to_latlon(
        self,
        cx: float, cy: float,
        img_w: int, img_h: int,
        fov_v_deg: float,
        uav_lat: float, uav_lon: float, uav_alt: float,
        gimbal_pan_deg: float, gimbal_tilt_deg: float,
    ) -> Tuple[float, float]:
        """
        像素偏移 → 角偏移 → 世界经纬度（利用 pan_tilt_to_latlon）。

        原理:
            - 图像中心为光轴参考点；像素中心(cx,cy)相对图像中心的偏移量
              按"偏移比例 × 半视场角"换算为云台 pan/tilt 的角增量
            - 垂直 FOV 直接取云台视场角；水平 FOV 由宽高比与垂直 FOV 反推
              （tan(fov_h/2) = tan(fov_v/2) * w/h）
            - 云台绝对角 + 角增量 → pan_tilt_to_latlon 结合本机位姿反算地面经纬度
        """
        fov_v = max(_FOV_V_MIN, min(_FOV_V_MAX, float(fov_v_deg)))
        # 水平视场角：由图像宽高比从垂直视场角反推
        fov_h = 2.0 * math.degrees(
            math.atan(math.tan(math.radians(fov_v / 2.0)) * (img_w / img_h)))
        # 像素偏移 → 云台角增量（右/上为正；y 向下故 tilt 用 h/2 - cy）
        pan_delta = (cx - img_w / 2.0) / (img_w / 2.0) * (fov_h / 2.0)
        tilt_delta = (img_h / 2.0 - cy) / (img_h / 2.0) * (fov_v / 2.0)
        # 反算目标经纬度（赛题 SDK：本机位姿 + 云台绝对角 + 角增量）
        tlat, tlon = pan_tilt_to_latlon(
            uav_lat, uav_lon, uav_alt,
            gimbal_pan_deg, gimbal_tilt_deg,
            pan_delta, tilt_delta)
        return tlat, tlon

    # ── 静态工具：PNG bytes → BGR numpy 数组 ──
    @staticmethod
    def decode_photo(photo: bytes) -> "Optional[np.ndarray]":
        """把仿真推送的 PNG/JPG bytes 解码为 BGR 数组（HxWx3）；失败返回 None。"""
        import cv2        # 延迟导入 opencv-python
        import numpy as np  # 延迟导入
        if photo is None:
            return None
        try:
            img = cv2.imdecode(np.frombuffer(photo, dtype=np.uint8),
                               cv2.IMREAD_COLOR)
        except Exception:
            return None
        if img is None or img.size == 0:
            return None
        return img


# ══════════════════════════════════════════════════════════════════════════════
# 模块对外统一入口：供 CoopAgent 子类的 sensor() 调用
# ══════════════════════════════════════════════════════════════════════════════

def get_detect_result(
    photo,
    uav,
    detector: YOLODetector,
    fov_deg: Optional[float] = None,
) -> Optional[List[Detection]]:
    """
    视觉检测统一接口：一帧输入 → YOLO 推理 → 坐标转换 → List[Detection]。

    参数:
        photo     PNG bytes（obs.self.photo）或 BGR numpy 数组（HxWx3）
        uav       本机状态对象（obs.self），需含属性:
                  lat / lon / alt / gimbal_pan / gimbal_tilt
                  （gimbal_fov_deg 用于取视场角，fov_deg 传参时可不读）
        detector  YOLODetector 实例（内部含超时降级）
        fov_deg   云台垂直视场角（度）；None 则读取 uav.gimbal_fov_deg

    返回（SDK-API §2.4/§2.6 语义，供 sensor() 直接 return）:
        None         → 回退 SDK 默认识别器（无画面 / 解码失败 / 检测器不可用 / 异常容错）
        []           → 明确本帧无目标
        [Detection]  → 自研检测结果（按置信度降序，list[0] 作为 obs.self.detection 主检测）
    """
    try:
        # ── 检测器可用性检查：不可用 → 回退默认识别器 ──
        if detector is None or not detector.available:
            return None

        # ── 输入归一化：bytes → BGR 数组；numpy 数组直接使用 ──
        import numpy as np  # 延迟导入
        if isinstance(photo, np.ndarray):
            img = photo
        elif isinstance(photo, (bytes, bytearray)):
            img = YOLODetector.decode_photo(bytes(photo))
            if img is None:
                return None  # 解码失败 → 回退默认识别器
        else:
            return None  # 无画面 → 回退默认识别器

        h, w = img.shape[:2]
        if h <= 0 or w <= 0:
            return None

        # ── 防御性读取本机位姿 / 云台姿态（缺失时用默认值） ──
        uav_lat = float(getattr(uav, "lat", 0.0))
        uav_lon = float(getattr(uav, "lon", 0.0))
        uav_alt = float(getattr(uav, "alt", 0.0))
        gimbal_pan = float(getattr(uav, "gimbal_pan", 0.0))
        gimbal_tilt = float(getattr(uav, "gimbal_tilt", 0.0))
        if fov_deg is None:
            fov_deg = float(getattr(uav, "gimbal_fov_deg", _FOV_V_MAX))

        # ── YOLO 推理（耗时监控 + 超时降级在内部处理） ──
        boxes = detector.detect(img)
        if not boxes:
            return []  # 本帧无目标 → 空列表（明确无检测）

        # ── 坐标转换：像素框 → 经纬度 → Detection 列表 ──
        dets: List[Detection] = []
        for b in boxes:
            tlat, tlon = detector.pixel_to_latlon(
                b.cx, b.cy, w, h, fov_deg,
                uav_lat, uav_lon, uav_alt, gimbal_pan, gimbal_tilt)
            dets.append(Detection(
                detected=True,
                confidence=b.conf,
                target_lat=tlat,
                target_lon=tlon,
                target_type=b.cls_name,  # 类别名；真假判别由决策层运动学多特征投票完成
            ))

        # 按置信度降序：list[0] 为最可信目标（SDK 仅取 list[0] 作为主检测）
        dets.sort(key=lambda d: d.confidence, reverse=True)
        return dets
    except Exception:
        # 异常容错：回退默认识别器，绝不中断比赛（SDK-API §2.4）
        logger.warning("get_detect_result() FAILED -> fallback to default detector",
                       exc_info=True)
        return None
