---
name: fix-ema-speed-and-yolo-integration
overview: 修复 EMA 速度计算 bug（time.monotonic() → 仿真时间）并正确接入 YOLO 视觉检测结果，使真目标能被正确识别和摧毁。
todos:
  - id: explore-ema-callsites
    content: 使用 [subagent:code-explorer] 搜索所有调用 EMATracker.append() 和 DecoyClassifier.update() 的位置，确认完整修改范围
    status: pending
  - id: fix-ema-timestamp
    content: 修改 ema_filter.py：EMATracker.append() 增加 sim_t 参数替代 time.monotonic()，speed_mps() 重写为基于仿真时间差的线性回归，speed_variance() 改用仿真时间差
    status: pending
    dependencies:
      - explore-ema-callsites
  - id: fix-decoy-classifier
    content: 修改 decoy_classifier.py：DecoyClassifier.update() 将 dt 参数传递给 _ema.append()，确保 EMA 时间戳一致
    status: pending
    dependencies:
      - fix-ema-timestamp
  - id: fix-drone-agent-calls
    content: 修改 drone_agent.py：所有 _ema.append() 调用处传入 sim_t，优化 YOLO visual confidence 逻辑结合 target_type 字段
    status: pending
    dependencies:
      - fix-ema-timestamp
  - id: verify-yolo-config
    content: 检查 config/algorithm.yaml 和仿真启动参数，确认 eval 模式 + photo_mode=auto + yolo_model_path 正确配置，确保 UE 相机帧能到达 Redis
    status: pending
---

## 用户需求
修复 EMA 速度计算 bug 并正确接入 YOLO 模型，使仿真能够正常检测和追踪真目标，最终完成摧毁任务。

## 产品概述
针对 coop_decoy 赛题，当前智能体存在两个核心问题：
1. EMA 速度计算使用挂钟时间而非仿真时间，导致 50m 位置噪声被放大成 60~800 m/s 的虚假速度，所有目标被误判为诱饵
2. YOLO 模型已训练但未正确接入 eval 模式，且 visual confidence 未流入诱饵判别器

## 核心功能
- **EMA 速度计算修复**：用仿真时间 `sim_t` 替代 `time.monotonic()`，使速度估计准确反映目标真实运动（5~15 m/s）
- **YOLO 正确接入**：确保 eval 模式下 YOLO 检测结果正常流入 decide()，并将 YOLO 的 confidence 传递给 DecoyClassifier
- **诱饵判别正确性**：修复后真目标速度应在合理范围内（3~18 m/s），多特征投票能正确区分真目标与诱饵


## 技术栈
- Python 3.12
- 现有项目框架：SDK CoopAgent 基类 + 四模块架构（communication / vision_detect / cluster_scheduler / motion_control）
- YOLOv8s 模型（`models/target_vehicle_yolov8s.pt`）

## 实现方案

### 一、EMA 速度计算 bug 修复

**策略**：将 `EMATracker` 的 `append()` 方法增加仿真时间参数 `sim_t: float`，替代 `time.monotonic()`。同时重写 `speed_mps()` 和 `speed_variance()` 以直接使用仿真时间差进行线性回归，移除基于 `tick_hz=10.0` 的回退路径。

**修改链路**：
```
drone_agent.py: self._ema.append(lat, lon, sim_t)
  → ema_filter.py: EMATracker.append(lat, lon, sim_t)  # 用 sim_t 替代 time.monotonic()
  → ema_filter.py: EMATracker.speed_mps()  # 直接用仿真时间差做线性回归
  → ema_filter.py: EMATracker.speed_variance()  # 直接用仿真时间差
  → decoy_classifier.py: DecoyClassifier.update() → _ema.append(lat, lon, sim_t)
```

**关键设计决策**：
- `speed_mps()` 移除 `tick_hz` 参数和帧序号回退路径，统一用仿真时间差做线性回归
- `speed_variance()` 中 `dt = t2 - t1` 直接用仿真时间差
- 保留 `history=80` 的缓冲区大小，但时间戳改为仿真时间

### 二、YOLO 正确接入

**当前问题**：`drone_agent.py` 的 `sensor()` 方法返回 `None`，SDK 走默认识别器。在 eval 模式下，Runner 创建 `YoloDetector(primary) + AccuracySimulator(fallback)`，但当前所有 UAV 的 photo 都是 None，导致 YOLO 降级到 AccuracySimulator。

**YOLO 接入不需要修改 agent 代码**：SDK 的 `DetectionResolver` 已经正确实现了三态分发（sensor 返回 None → 走默认识别器 → eval 模式走 YoloDetector）。YOLO 检测结果会自动填充到 `obs.self.detection` 中，`decide()` 方法直接读取 `det = obs.self.detection` 即可。

**photo 为 None 的根因**：UE 渲染端可能未正确推送相机帧到 Redis。需要检查：
1. UE 仿真是否正常启动（非 dry_run 模式）
2. `photo_mode` 是否设为 `auto` 或 `on`（已在 `config/algorithm.yaml` 中配置为 `auto`）
3. Redis 中是否存在 `sync_camera:{uid}:frame:*` 键

**YOLO detection confidence 流入诱饵判别器**：`drone_agent.py` 第 287-294 行已实现将 detection confidence 传给 `DecoyClassifier.set_visual_confidence()`。但当前逻辑仅基于 confidence 数值判断 label（>=0.7 为 target，<0.4 为 decoy）。YOLO 的 `target_type` 字段（`ground_vehicle` 或 `decoy_vehicle`）也可以作为补充信号。改进方案：结合 YOLO 的 `confidence` 和 `target_type` 综合判断。

### 三、性能考虑

- EMA 速度计算改为仿真时间后，线性回归的精度取决于样本的时间跨度（80 帧 × 0.0167s ≈ 1.3s），足够准确
- 不需要额外存储，仅改变时间戳来源

### 四、实现注意事项

- **不修改 SDK 代码**：只改 `coop_decoy` 目录下的选手代码
- **保持向后兼容**：`append()` 增加 `sim_t` 可选参数，默认值保留以兼容旧调用
- **日志验证**：修复后在 VERIFY 日志中观察 speed 是否在 5~15 m/s 合理范围


## Agent Extensions

### SubAgent
- **code-explorer**
  - 目的：在修复 EMA 速度计算 bug 前，全面确认所有调用 `EMATracker.append()` 的位置，确保不遗漏任何调用点
  - 预期结果：找出 `drone_agent.py` 和 `decoy_classifier.py` 中所有 `_ema.append()` 和 `_decoy_clf.update()` 的调用点，确认需要同步修改的位置
