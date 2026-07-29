# HF2026 - 无人机集群自主协同跟踪系统

> **红枫2026无人集群自主协同智能算法挑战赛** | 赛题二：多机协同识别（coop_decoy）

---

## 一、项目简介

本项目为红枫2026无人集群自主协同智能算法挑战赛参赛工程，参赛赛道为**赛题二：多机协同识别**。

### 任务背景

3架固定翼无人机分散部署至比赛区域，区域内存在3台真实移动目标、15台移动诱饵车。诱饵进入相机视场时按约50%概率被误识别为真目标。队伍需通过有限通信实现多机信息共享，协同区分诱饵与真实目标，完成目标清除与精准坐标上报。

### 核心能力

- **自主搜索**：3架UAV按扇区分工巡航，不重复、不扎堆
- **目标识别**：通过多帧运动学一致性（EMA滤波 + 线性回归测速）区分真目标与诱饵
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
├── src/                            # 源代码
│   ├── __init__.py
│   ├── communication/              # 通信模块 (成员1)
│   │   ├── __init__.py
│   │   ├── protocol.py             # 通信协议编解码 (T:/D:/A:/C:/R:)
│   │   └── client.py               # Redis通信客户端封装
│   ├── motion_control/             # 航迹控制模块 (成员2)
│   │   ├── __init__.py
│   │   ├── search.py               # 扇区搜索 + 螺旋扫描
│   │   ├── tracker.py              # 跟踪loiter + LOS云台控制
│   │   └── geo.py                  # 地理工具 (haversine/bearing/clamp)
│   ├── cluster_scheduler/          # 集群调度模块 (成员3)
│   │   ├── __init__.py
│   │   ├── coordinator.py          # K=2协同分配 + 站位策略
│   │   └── state_machine.py        # SEARCH→VERIFY→TRACK→RELEASE状态机
│   ├── vision_detect/              # 视觉识别模块 (成员4)
│   │   ├── __init__.py
│   │   ├── ema_filter.py           # EMA滤波 + 速度回归
│   │   ├── decoy_classifier.py     # 诱饵判别 (多特征融合)
│   │   └── report.py               # report_target上报 + RMSE优化
│   └── main.py                     # 主程序入口 (队长/成员5)
├── config/                         # 配置文件
│   └── algorithm.yaml              # 算法参数配置
├── docs/                           # 文档
│   └── INTERFACE.md                # 模块接口规范文档
├── dataset/                        # 视觉采集样本 (仅本地, 打包时移除)
├── sim_test_log/                   # 仿真测试日志
├── install_offline/                # 离线安装
│   ├── offline_requirements.txt    # 离线依赖清单
│   └── wheels/                     # 离线whl依赖包
├── scripts/                        # 工具脚本
│   ├── eval_runner.py              # 批量评估脚本
│   └── pack_submission.py          # 提交包打包脚本
├── requirements.txt                # Python依赖
├── README.md                       # 项目说明
└── .gitignore                      # Git忽略规则
```

---

## 三、环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10 1809+ / Windows 11 / Ubuntu 24.04 |
| Python | 3.10+ |
| 核心依赖 | redis, pyyaml, numpy |
| 仿真平台 | 红枫2026赛事平台 (发行包自带Redis 7.4.2, Node.js v22) |

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
PYTHONPATH=. python -m competition run \
  --scenario coop_decoy \
  --agent src.main:HF2026Agent \
  --duration 600
```

### 方式2：赛事平台前端界面加载

在前端算法输入框填写：`src/main.py:HF2026Agent`，点击「开始仿真」。

### 方式3：直接运行主程序（冒烟测试）

```bash
python src/main.py
```

### 调试参数

| 参数 | 说明 |
|------|------|
| `--seed N` | 固定随机种子，场景完全复现 |
| `--visualize` | 开启浏览器可视化视图 |
| `--dry-run` | 空转冒烟测试，不启动仿真引擎 |
| `--output DIR` | 自定义评分文件输出目录 |

### 批量评估

```bash
python scripts/eval_runner.py --seeds 20 --scenario coop_decoy
```

---

## 五、模块分工说明

| 角色 | 代号 | 核心模块 | 技术报告章节 |
|------|------|----------|-------------|
| **队长** | 成员5 | 统筹协调 + `main.py`主框架 + 集成调试 + 离线打包 | 系统总体架构、创新点、应用价值 |
| **成员1** | 通信 | `src/communication/` 通信协议编解码 + 平台API封装 | 研究背景、通信方案 |
| **成员2** | 控制 | `src/motion_control/` 航迹规划 + 云台跟踪 + 地理工具 | 航迹规划、云台跟踪控制 |
| **成员3** | 调度 | `src/cluster_scheduler/` K=2协同分配 + 站位策略 + 状态机 | 多机协同调度（核心创新章节） |
| **成员4** | 感知 | `src/vision_detect/` EMA滤波 + 诱饵判别 + 上报优化 | 视觉检测、诱饵判别 |

### 各模块职责

#### 通信模块 (`src/communication/`) — 成员1

- `protocol.py`：五类消息编解码
  - `T:lat,lon` 确认真目标位置共享
  - `D:lat,lon` 确认诱饵位置共享
  - `A:tgtidx,rank` 目标认领声明
  - `C:tgtidx` 目标已摧毁通知
  - `R:lat,lon` 召唤队友汇聚
- `client.py`：Redis通信客户端封装（字节限制、频率限制、收件箱管理）

#### 航迹控制模块 (`src/motion_control/`) — 成员2

- `geo.py`：地理工具（haversine距离、bearing方位角、边界裁剪）— **全员依赖，最先完成**
- `search.py`：扇区搜索——3架UAV均分360°，各自扇区内螺旋扩张扫描
- `tracker.py`：跟踪loiter——K=2站位盘旋，云台LOS瞄准，间距>200m

#### 集群调度模块 (`src/cluster_scheduler/`) — 成员3

- `coordinator.py`：K=2协同分配——召唤→汇聚→同时盯防→摧毁→释放→轮转
- `state_machine.py`：状态机 SEARCH→VERIFY→TRACK→RELEASE

#### 视觉识别模块 (`src/vision_detect/`) — 成员4

- `ema_filter.py`：EMA滤波 + 线性回归速度估计
- `decoy_classifier.py`：多特征融合诱饵判别（速度+加速度方差+方向一致性+位移跨度）
- `report.py`：report_target上报优化（只报告EMA平滑后位置，降低RMSE）

#### 主程序 (`src/main.py`) — 队长

- Agent主类定义，实现 `decide(obs, dt)` 主循环
- 调用各子模块、组装控制指令列表
- 从 `config/algorithm.yaml` 加载参数

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
- **启动命令固定**：`python src/main.py`
- **Git工作流**：main（稳定版）← develop（集成分支）← feature/*（各成员分支）
- **接口契约**：参见 `docs/INTERFACE.md`，变更需队长审批
- **文件编码**：UTF-8
