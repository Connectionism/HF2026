---
name: fix-ema-speed-and-yolo-integration
overview: 修复 EMA 速度计算 bug（time.monotonic() → 仿真时间）并正确接入 YOLO 模型（路径 + visual confidence + target_type），使真目标能被正确识别和摧毁。
todos:
  - id: fix-ema-filter
    content: 修改 ema_filter.py：append() 增加 sim_t 可选参数替代 time.monotonic()，speed_mps() 重写为纯仿真时间线性回归移除 tick_hz 回退路径，speed_variance() 改用仿真时间差
    status: completed
  - id: fix-decoy-classifier
    content: 修改 decoy_classifier.py：update() 增加 sim_t 参数并传递给 _ema.append()，确保 EMA 时间戳一致
    status: completed
    dependencies:
      - fix-ema-filter
  - id: fix-drone-agent
    content: 修改 drone_agent.py：第 494 行 _ema.append() 传入 sim_t，第 674 行 _ema.append() 传入 sim_t，第 495 行 _decoy_clf.update() 传入 sim_t，第 287-294 行利用 det.target_type 优化 visual confidence 逻辑
    status: completed
    dependencies:
      - fix-ema-filter
  - id: update-yolo-config
    content: 更新 config/algorithm.yaml：model_path 改为实际 best.pt 路径，imgsz 从 1024 改为 640
    status: completed
  - id: verify-and-clean
    content: 清理所有 __pycache__，验证代码语法正确性
    status: completed
    dependencies:
      - fix-ema-filter
      - fix-decoy-classifier
      - fix-drone-agent
      - update-yolo-config
---

## 用户需求
修复两个核心问题使仿真能正常检测和追踪真目标，最终完成摧毁任务得分：

### 问题一：EMA 速度计算 bug
当前 `ema_filter.py` 的 `EMATracker.append()` 使用 `time.monotonic()` 记录挂钟时间戳，`speed_mps()` 在时间跨度小于 1 秒时回退到 `tick_hz=10.0` 的帧序号回归，将 AccuracySimulator 的 50m 噪声放大成 60~800 m/s 虚假速度。日志确认所有目标速度异常，全部被误判为诱饵，dwell 始终为 0。

修复目标：用仿真时间 `sim_t` 替代 `time.monotonic()`，使速度估计准确反映目标真实运动（5~15 m/s），真目标能通过 VERIFY 判别进入 TRACK 状态。

### 问题二：YOLO 模型接入
用户已训练好 YOLOv8n 二分类模型（区分 TargetVehicle 和 DecoyVehicle），模型路径为 `D:\HF资料\yolov8_target_decoy\runs\detect\runs\train\weights\best.pt`。需要更新配置文件中的模型路径和参数（imgsz 从 1024 改为 640），并优化 visual confidence 逻辑使其正确利用 `Detection.target_type` 字段。

## 技术栈
- Python 3.12
- 现有 SDK：CoopAgent 基类 + DetectionResolver 三态分发
- YOLOv8n（ultralytics）
- 60Hz 仿真环境

## 实现方案

### 一、EMA 时间戳修复

**策略**：`EMATracker.append()` 增加可选参数 `sim_t: Optional[float] = None`，传入时使用仿真时间，未传入时回退到 `time.monotonic()` 保持向后兼容。`speed_mps()` 和 `speed_variance()` 直接使用存储的时间戳做线性回归，移除 `tick_hz=10.0` 帧序号回退路径。

**修改链路**：
```
drone_agent.py: self._ema.append(lat, lon, sim_t)     ← 传入仿真时间
decoy_classifier.py: self._ema.append(lat, lon, sim_t) ← DecoyClassifier.update() 传入 sim_t
  → ema_filter.py: EMATracker.append(lat, lon, sim_t)  ← 存储仿真时间戳
  → ema_filter.py: EMATracker.speed_mps()              ← 纯仿真时间线性回归
  → ema_filter.py: EMATracker.speed_variance()         ← 仿真时间差
```

**关键设计决策**：
- `append()` 增加 `sim_t` 可选参数，默认 `None` 时用 `time.monotonic()` 保持向后兼容
- `speed_mps()` 移除 `tick_hz` 参数和帧序号回退分支，统一用存储的时间戳做 2D 线性回归
- `speed_variance()` 中 `dt = t2 - t1` 直接用存储的仿真时间差
- 60Hz 下 80 帧缓冲区 ≈ 1.3 秒仿真时间，足够准确

### 二、YOLO 配置更新

- `config/algorithm.yaml`：`model_path` 改为 `D:/HF资料/yolov8_target_decoy/runs/detect/runs/train/weights/best.pt`，`imgsz` 从 1024 改为 640
- 无需修改 `sensor()` 方法：SDK 的 `DetectionResolver` 已自动处理 eval 模式下 YoloDetector → obs.self.detection 的流程

### 三、Visual Confidence 优化

`drone_agent.py` 第 287-294 行的 visual confidence 逻辑当前仅基于 confidence 数值判断 label。改进为：
- 利用 `det.target_type` 字段：`"ground_vehicle"` → label "target"，`"decoy_vehicle"` → label "decoy"
- 结合 YOLO confidence 和 target_type 综合判断，confidence 高 + target_type="ground_vehicle" → 强支持真目标

### 四、修改文件清单

| 文件 | 修改内容 |
|------|---------|
| `ema_filter.py` | `append()` 增加 `sim_t` 参数；`speed_mps()` 重写为纯仿真时间线性回归；`speed_variance()` 改用仿真时间差 |
| `decoy_classifier.py` | `update()` 增加 `sim_t` 参数并传递给 `_ema.append()` |
| `drone_agent.py` | 第 494、674 行 `_ema.append()` 传入 `sim_t`；第 495 行 `_decoy_clf.update()` 传入 `sim_t`；第 287-294 行利用 `det.target_type` 优化 visual confidence |
| `config/algorithm.yaml` | 更新 `model_path` 和 `imgsz` |
