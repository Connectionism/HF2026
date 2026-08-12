# HF2026 - 无人机集群自主协同跟踪系统

> **红枫2026无人集群自主协同智能算法挑战赛** | 赛题二：多机协同识别（coop_decoy）

---

## 一、项目简介

本项目为红枫2026无人集群自主协同智能算法挑战赛参赛工程，参赛赛道为**赛题二：多机协同识别**。

### 任务背景

3架固定翼无人机分散部署至比赛区域，区域内存在3台真实移动目标、15台移动诱饵车。诱饵进入相机视场时按约50%概率被误识别为真目标。队伍需通过有限通信（≤50字节/条、≤4Hz、约1000m）实现多机信息共享，协同区分诱饵与真实目标，完成目标清除与精准坐标上报。

### 核心能力

- **自主搜索**：3架UAV按扇区分工巡航，不重复、不扎堆
- **目标识别**：自研 YOLO 检测（sensor 层选择②）+ 多帧运动学一致性（EMA滤波 + 线性回归测速）区分真目标与诱饵
- **协同打击**：K=2协同盯防——2架UAV同时盯防同一真目标累计20秒即判定摧毁
- **精准上报**：持续用 `report_target(lat, lon)` 上报判定为真目标的位置，按RMSE精度评分

### 评分体系

| 维度 | 权重 | 计算方式 | 目标 |
|------|------|----------|------|
| 摧毁率 (kill) | 50% | 100 × 摧毁真目标数 / 3 | 3/3 全歼 |
| 上报精度 (accuracy) | 30% | 100 × max(0, 1−RMSE/120m) | RMSE < 60m |
| 全歼耗时 (mission_time) | 20% | ≤240s满分，至420s衰减到0 | ≤300s |

---

## 二、目录结构

```
HF2026/
├── coop_decoy/                     # 参赛智能体包（提交根目录）
│   ├── drone_agent.py              # Agent 入口：DroneAgent（模块编排，不写业务算法）
│   ├── manifest.json               # 智能体清单（agent_class: DroneAgent）
│   ├── config/
│   │   └── algorithm.yaml          # 算法参数配置（感知/搜索/协同/进阶）
│   ├── weights/
│   │   └── best.pt                 # 自研 YOLO 权重（sensor 层选择②）
│   └── search_track/               # 四子模块（均通过 main.py 对外暴露）
│       ├── communication/          # CommHandler：通信解析 + 指令构建（成员1）
│       │   ├── main.py             #   └─ 对外入口（parse_inbox/build_broadcast/build_report）
│       │   ├── client.py           #     CommClient（SDK broadcast/send_to 封装）
│       │   ├── protocol.py         #     消息协议编解码（R:/T:/C:）
│       │   └── config.py           #     通信配置
│       ├── vision_detect/          # YOLO检测 + 坐标转换 + EMA + 诱饵判别（成员4）
│       │   ├── main.py             #   └─ 对外入口（get_detect_result）
│       │   ├── detect.py           #     YOLODetector（推理 + 像素→经纬度）
│       │   ├── ema_filter.py       #     EMATracker（EMA滤波 + 线性回归测速）
│       │   └── decoy_classifier.py #     DecoyClassifier（诱饵判别）
│       ├── cluster_scheduler/      # K=2协同分配 + 状态机（成员3）
│       │   ├── main.py             #   └─ 对外入口（CooperativeCoordinator + States）
│       │   └── coordinator.py      #     协同目标分配、召唤/汇聚/盯防逻辑
│       └── motion_control/         # 航迹规划 + 云台跟踪（成员2）
│           ├── main.py             #   └─ 对外入口（SearchController/TrackController/geo_utils）
│           ├── geo.py              #     hav/gis 工具（haversine/bearing/clamp/point_on_circle）
│           ├── search.py           #     搜索航点生成（螺旋/扇区扫描）
│           └── tracker.py          #     盘旋跟踪航点 + 云台LOS瞄准
├── config/                         # 全局配置（预留）
│   └── algorithm.yaml
├── docs/
│   └── INTERFACE.md                # 模块内部接口规范文档
├── install_offline/                # 离线安装
│   ├── offline_requirements.txt    # 离线依赖清单
│   └── wheels/                     # 离线whl依赖包
├── scripts/                        # 工具脚本
│   ├── eval_runner.py              # 批量评估脚本
│   └── pack_submission.py          # 提交包打包脚本
├── requirements.txt                # Python依赖
├── README.md                       # 项目说明
├── SDK-API.md                      # 平台 SDK 接口参考（权威）
└── .gitignore                      # Git忽略规则
```

---

## 三、环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10 1809+ / Windows 11 / Ubuntu 24.04 |
| Python | 3.10+ |
| 核心依赖 | ultralytics, opencv-python, numpy, pyyaml |
| 仿真平台 | 红枫2026赛事平台（`python -m competition ...`） |

> 说明：`requirements.txt` 中 `redis>=4.5.0` 为历史遗留项，当前实现已改为平台 SDK 通信（`broadcast`/`send_to`/`report_target`），**无需 Redis**，可安全移除。

### 安装依赖

```bash
pip install -r requirements.txt
```

### 离线安装（断网环境）

```bash
# 1. 在有网环境预下载whl包
pip download -r install_offline/offline_requirements.txt -d install_offline/wheels/

# 2. 在目标机器离线安装
pip install --no-index --find-links=install_offline/wheels/ -r install_offline/offline_requirements.txt
```

---

## 四、启动方式

### 方式1：赛事平台命令行运行（推荐）

```bash
python -m competition run \
  --scenario coop_decoy \
  --agent coop_decoy.drone_agent:DroneAgent \
  --duration 600
```

### 方式2：赛事平台前端界面加载

在前端算法输入框填写：`coop_decoy.drone_agent:DroneAgent`，点击「开始仿真」。

### 方式3：通过 manifest 加载

平台按 `coop_decoy/manifest.json`（`runner_module: drone_agent`、`agent_class: DroneAgent`）自动定位 Agent 入口，无需手动指定模块路径。

### 调试参数

| 参数 | 说明 |
|------|------|
| `--mode train` | 选手本地开发（默认）：AccuracySimulator 模拟检出概率 0.85、噪声 50m |
| `--mode eval` | 官方评测模式：YOLOv8 识别 UE 渲染图（选手本地一般用不到） |
| `--accuracy <p>` | train 模式检出概率，默认 0.85，上限钳 0.9 |
| `--noise-sigma <m>` | train 模式位置噪声标准差，默认 50，下限钳 30 |
| `--seed N` | 固定随机种子，场景完全复现 |
| `--visualize` | 开启浏览器可视化视图 |
| `--photo-mode auto` | 相机帧拉取开关（auto/on/off） |
| `--output DIR` | 自定义评分文件输出目录 |

### 批量评估

```bash
python scripts/eval_runner.py --seeds 20 --scenario coop_decoy \
  --agent coop_decoy.drone_agent:DroneAgent
```

---

## 五、模块分工说明

| 角色 | 代号 | 核心模块 | 技术报告章节 |
|------|------|----------|-------------|
| **队长** | 成员5 | 统筹协调 + `drone_agent.py`主框架 + 集成调试 + 离线打包 | 系统总体架构、创新点、应用价值 |
| **成员1** | 通信 | `search_track/communication/` 协议编解码 + 平台SDK封装 | 研究背景、通信方案 |
| **成员2** | 控制 | `search_track/motion_control/` 航迹规划 + 云台跟踪 + 地理工具 | 航迹规划、云台跟踪控制 |
| **成员3** | 调度 | `search_track/cluster_scheduler/` K=2协同分配 + 状态机 | 多机协同调度（核心创新章节） |
| **成员4** | 感知 | `search_track/vision_detect/` YOLO检测 + EMA滤波 + 诱饵判别 + 上报优化 | 视觉检测、诱饵判别 |

### 数据流

```
obs.self.photo → vision_detect.get_detect_result()（YOLO推理 + 像素→经纬度）
→ sensor() 返回 List[Detection] → SDK 注入 obs.self.detection
→ vision_detect（EMATracker + DecoyClassifier）
→ cluster_scheduler（CooperativeCoordinator，K=2 分配）
→ motion_control（SearchController + TrackController）
→ communication.build_broadcast() / build_report()
```

### 各模块职责

#### 智能体入口 (`drone_agent.py`) — 队长

- `DroneAgent(CoopAgent)`：继承赛题二专用基类 `competition.sdk.scenarios.coop_decoy.CoopAgent`，实现 `sensor()`/`decide()` 生命周期
- 只做模块编排，业务算法全部下沉至 `search_track/` 四子模块
- 从 `coop_decoy/config/algorithm.yaml` 加载参数

#### 通信模块 (`search_track/communication/`) — 成员1

- `main.py` 暴露 `CommHandler`：解析 `obs.comm_inbox` + 构建指令
- 消息协议（`protocol.py`）：
  - `R:lat,lon,uid` 发现真目标广播（召唤汇聚）
  - `T:lat,lon,dwell` 本机盯防状态广播
  - `C:lat,lon` 目标已摧毁广播
- 底层基于平台 SDK `broadcast`/`report_target` 命令构造器，非 Redis

#### 航迹控制模块 (`search_track/motion_control/`) — 成员2

- `geo.py`：地理工具（haversine距离、bearing方位角、边界裁剪、圆上取点）— **全员依赖，最先完成**
- `search.py`：搜索航点生成——螺旋搜索 / 扇区扩张扫描（3架UAV均分扇区）
- `tracker.py`：跟踪loiter——K=2站位盘旋，云台LOS瞄准

#### 集群调度模块 (`search_track/cluster_scheduler/`) — 成员3

- `coordinator.py`：`CooperativeCoordinator` K=2协同分配——召唤→汇聚→同时盯防→摧毁→释放→轮转
- 状态机：`States` 常量 SEARCH→VERIFY→TRACK 流转（相关逻辑已并入 coordinator）

#### 视觉识别模块 (`search_track/vision_detect/`) — 成员4

- `detect.py`：`YOLODetector` 推理（sensor 层选择②自研识别）+ 像素→经纬度坐标转换
- `ema_filter.py`：`EMATracker` EMA滤波 + 线性回归速度估计
- `decoy_classifier.py`：`DecoyClassifier` 多特征融合诱饵判别（速度+方向一致性+位移跨度）
- `report.py` 逻辑并入 `drone_agent`：report_target上报优化（只报告EMA平滑后位置，降低RMSE）

---

## 六、战场参数速查

| 维度 | 数据 |
|------|------|
| 控制对象 | 3架固定翼UAV（速度15-40 m/s，高度500m） |
| 真目标 | 3辆移动小车（速度5/9/12 m/s） |
| 诱饵 | 15辆移动诱饵车（~50%误识别率） |
| 感知噪声 | σ=50m位置噪声，85%检出率 |
| 通信约束 | ≤50字节/条，≤4Hz，~1000m范围，收件箱32条 |
| 协同门槛 | K=2：≥2架UAV同时盯防20s → 摧毁 |
| 仿真时长 | 600s（10分钟） |
| 地图范围 | 6.6km×4.4km（纬度26.9818~27.0250, 经度124.9800~125.0203） |

---

## 七、开发规范

- **代码统一使用相对路径**，禁止写死个人绝对路径
- **Agent 入口固定**：`coop_decoy.drone_agent:DroneAgent`（对应 `manifest.json` 中 `runner_module: drone_agent` + `agent_class: DroneAgent`）
- **子模块仅通过各 `search_track/*/main.py` 对外暴露**，禁止跨模块直接访问底层文件
- **Git工作流**：main（稳定版）← develop（集成分支）← feature/*（各成员分支）
- **接口契约**：SDK 见 `SDK-API.md`（权威），模块内部接口见 `docs/INTERFACE.md`，变更需队长审批
- **文件编码**：UTF-8
