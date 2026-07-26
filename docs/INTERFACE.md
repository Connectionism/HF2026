# HF2026 模块接口规范文档

> **项目**: 弱对抗条件下无人机群自主协同跟踪
> **赛题**: 赛题二 - 多机协同识别 (coop_decoy)
> **版本**: V1.0
> **日期**: 2026-07-26

---

## 一、文档概述

本文档定义 HF2026 项目四大核心模块的对外调用接口规范，作为全员开发的基准契约。
所有模块接口在 Day 1 晚间全体确认后锁定，变更需队长审批并通知全体。

### 模块总览

| 模块 | 路径 | 负责人 | 核心职责 |
|------|------|--------|----------|
| 通信模块 | `src/communication/` | 成员1 | 通信协议编解码 + 平台API封装 |
| 航迹控制 | `src/motion_control/` | 成员2 | 航迹规划 + 云台跟踪 + 地理工具 |
| 集群调度 | `src/cluster_scheduler/` | 成员3 | K=2协同分配 + 站位策略 + 状态机 |
| 视觉识别 | `src/vision_detect/` | 成员4 | EMA滤波 + 诱饵判别 + 上报优化 |

### 数据流

```
                    ┌─────────────────┐
                    │   obs (观测)     │
                    │  obs.self       │
                    │  obs.comm_inbox  │
                    │  obs.briefing   │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   main.py       │  ← 队长: Agent主循环
                    │   decide()      │
                    └────────┬────────┘
                             │
          ┌──────────────────┼──────────────────┐
          │                  │                  │
   ┌──────▼──────┐  ┌───────▼───────┐  ┌──────▼──────┐
   │ vision_detect│  │cluster_scheduler│  │communication│
   │ (感知判别)   │  │ (调度决策)     │  │ (通信收发)  │
   └──────┬──────┘  └───────┬───────┘  └──────┬──────┘
          │                 │                  │
          └────────┬────────┘                  │
                   │                           │
          ┌────────▼────────┐                  │
          │ motion_control  │                  │
          │ (航迹+云台控制) │                  │
          └────────┬────────┘                  │
                   │                           │
                   └─────── commands ──────────┘
                            (控制指令)
```

---

## 二、地理工具接口 (`src/motion_control/geo.py`)

> **负责人**: 成员2 | **优先级**: 最高（全员依赖，Day 2 最先完成）

### 2.1 函数签名

```python
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算两点间大圆距离（米）

    参数:
        lat1, lon1: 起点纬度、经度
        lat2, lon2: 终点纬度、经度

    返回:
        距离（米），float
    """

def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    计算方位角

    参数:
        lat1, lon1: 起点纬度、经度
        lat2, lon2: 终点纬度、经度

    返回:
        方位角（度），0=正北，顺时针，范围[0, 360)
    """

def clamp_to_safebox(lat: float, lon: float) -> tuple[float, float]:
    """
    裁剪坐标到飞行区域安全边界内（避免越界扣分）

    参数:
        lat, lon: 原始纬度、经度

    返回:
        (clamped_lat, clamped_lon) 裁剪后的坐标

    边界:
        纬度: 26.9818 ~ 27.0250
        经度: 124.9800 ~ 125.0203
        越界阈值: 超出边界500m开始扣分
    """
```

---

## 三、通信接口 (`src/communication/`)

> **负责人**: 成员1

### 3.1 通信协议 (`src/communication/protocol.py`)

#### 五类消息格式

| 类型 | 格式 | 示例 | 含义 | 最大字节 |
|------|------|------|------|----------|
| Target | `T:lat,lon` | `T:27.00512,125.00134` | 确认真目标位置共享 | ~24B |
| Decoy | `D:lat,lon` | `D:27.00345,125.00211` | 确认诱饵位置共享 | ~24B |
| Claim | `A:tgtidx,rank` | `A:0,20002` | 目标认领声明 | ~14B |
| Destroyed | `C:tgtidx` | `C:0` | 目标已摧毁通知 | ~6B |
| Summon | `R:lat,lon` | `R:27.00512,125.00134` | 召唤队友汇聚 | ~24B |

#### 编解码函数签名

```python
def encode_target(lat: float, lon: float) -> str:
    """编码真目标位置 -> 'T:lat,lon'"""

def decode_target(payload: str) -> tuple[float, float] | None:
    """解码真目标消息 -> (lat, lon) 或 None"""

def encode_decoy(lat: float, lon: float) -> str:
    """编码诱饵位置 -> 'D:lat,lon'"""

def decode_decoy(payload: str) -> tuple[float, float] | None:
    """解码诱饵消息 -> (lat, lon) 或 None"""

def encode_claim(tgt_idx: int, rank: int) -> str:
    """编码目标认领 -> 'A:tgtidx,rank'"""

def decode_claim(payload: str) -> tuple[int, int] | None:
    """解码认领消息 -> (tgt_idx, rank) 或 None"""

def encode_destroyed(tgt_idx: int) -> str:
    """编码摧毁通知 -> 'C:tgtidx'"""

def decode_destroyed(payload: str) -> int | None:
    """解码摧毁通知 -> tgt_idx 或 None"""

def encode_summon(lat: float, lon: float) -> str:
    """编码召唤消息 -> 'R:lat,lon'"""

def decode_summon(payload: str) -> tuple[float, float] | None:
    """解码召唤消息 -> (lat, lon) 或 None"""
```

### 3.2 通信客户端 (`src/communication/client.py`)

```python
class CommClient:
    """
    Redis通信客户端封装

    约束:
        - 单条消息 <= 50 字节
        - 发送频率 <= 4 Hz
        - 通信距离 ~1000 m
        - 收件箱容量 32 条
    """

    def __init__(self, my_uid: int, max_rate_hz: float = 4.0):
        """
        参数:
            my_uid: 本机无人机UID
            max_rate_hz: 最大发送频率(Hz)
        """

    def broadcast(self, payload: str) -> bool:
        """
        广播消息给所有队友

        参数:
            payload: 消息内容(已编码字符串)
        返回:
            True=发送成功, False=被限流/超长
        """

    def send_to(self, uid: int, payload: str) -> bool:
        """
        定向发送消息给指定队友

        参数:
            uid: 目标无人机UID
            payload: 消息内容
        返回:
            True=发送成功, False=失败
        """

    def ingest(self, comm_inbox: list) -> list[dict]:
        """
        解析收件箱消息

        参数:
            comm_inbox: obs.comm_inbox 原始消息列表
        返回:
            解析后的消息字典列表 [{type, lat, lon, ...}, ...]
        """
```

---

## 四、航迹控制接口 (`src/motion_control/`)

> **负责人**: 成员2

### 4.1 扇区搜索 (`src/motion_control/search.py`)

```python
class SectorSearch:
    """
    扇区搜索策略

    3架UAV将360度均分为3个扇区，各自在扇区内做扩张螺旋扫描。
    按UID哈希分配扇区，避免重复覆盖。
    """

    def __init__(self, my_uid: int, fleet_size: int, map_bounds: dict):
        """
        参数:
            my_uid: 本机无人机UID
            fleet_size: 编队规模(3)
            map_bounds: 地图边界 {lat_min, lat_max, lon_min, lon_max}
        """

    def sector_index(self) -> int:
        """返回本机分配的扇区索引(0/1/2)"""

    def next_waypoint(self, current_lat: float, current_lon: float) -> tuple[float, float]:
        """
        返回下一个搜索航点

        参数:
            current_lat, current_lon: 当前位置
        返回:
            (target_lat, target_lon) 下一个搜索航点
        """
```

### 4.2 跟踪控制 (`src/motion_control/tracker.py`)

```python
class TrackController:
    """
    K=2站位跟踪控制器

    2架UAV在目标周围不同方位(SLOT_0/SLOT_1)盘旋，
    保持两机间距>200m避免扣分，云台LOS瞄准保持目标在FOV中心。
    """

    def __init__(self, track_radius: float = 330, track_speed: float = 20):
        """
        参数:
            track_radius: 跟踪环半径(米), 范围200-400
            track_speed: 跟踪盘旋速度(m/s)
        """

    def slot_position(self, target_lat: float, target_lon: float,
                      slot: int) -> tuple[float, float]:
        """
        计算站位位置

        参数:
            target_lat, target_lon: 目标位置
            slot: 站位槽位(0或1)
        返回:
            (lat, lon) 站位位置坐标
        """

    def loiter_waypoint(self, target_lat: float, target_lon: float,
                        slot: int, current_heading: float) -> tuple[float, float]:
        """
        返回盘旋航点

        参数:
            target_lat, target_lon: 目标位置
            slot: 站位槽位
            current_heading: 当前航向(度)
        返回:
            (lat, lon) 盘旋航点
        """

    def gimbal_point(self, target_lat: float, target_lon: float) -> dict:
        """
        返回云台瞄准指令

        参数:
            target_lat, target_lon: 目标位置
        返回:
            {'cmd': 'point_gimbal', 'lat': ..., 'lon': ...}
        """
```

---

## 五、集群调度接口 (`src/cluster_scheduler/`)

> **负责人**: 成员3（核心难点模块）

### 5.1 协同调度器 (`src/cluster_scheduler/coordinator.py`)

```python
class CooperativeCoordinator:
    """
    K=2协同调度器

    核心逻辑:
        1. UAV-A发现确认真目标 -> 广播R:召唤
        2. UAV-B收到R:消息 -> 飞向目标(SLOT_1方位)
        3. UAV-A在目标SLOT_0方位盘旋 -> 间距>200m
        4. 2架同时检测到目标 -> dwell累计 -> 满20s
        5. 广播C:摧毁通知 -> 释放 -> 转下一目标
    """

    def __init__(self, my_uid: int, k: int = 2):
        """
        参数:
            my_uid: 本机无人机UID
            k: 协同盯防阈值(赛题二K=2)
        """

    def ingest_comms(self, comm_inbox: list) -> None:
        """
        摄入通信消息，更新内部目标/摧毁状态

        参数:
            comm_inbox: 解析后的通信消息列表
        """

    def confirm_target(self, lat: float, lon: float) -> None:
        """
        确认真目标，加入目标列表并广播召唤

        参数:
            lat, lon: 真目标位置
        """

    def confirm_decoy(self, lat: float, lon: float) -> None:
        """
        确认诱饵，加入诱饵列表并广播共享

        参数:
            lat, lon: 诱饵位置
        """

    def select_target(self, self_lat: float, self_lon: float) -> dict | None:
        """
        贪婪自选最优目标

        参数:
            self_lat, self_lon: 本机当前位置
        返回:
            目标字典 {lat, lon, idx, slot} 或 None(无可用目标)
        """

    def my_slot(self, tgt: dict, fleet_size: int) -> int:
        """
        计算本机对指定目标的站位槽位

        参数:
            tgt: 目标字典
            fleet_size: 编队规模
        返回:
            槽位编号(0或1)
        """

    def aim_point(self, tgt: dict, slot: int) -> tuple[float, float]:
        """
        计算站位瞄准点

        参数:
            tgt: 目标字典
            slot: 站位槽位
        返回:
            (lat, lon) 瞄准点坐标
        """

    def is_destroyed(self, tgt: dict) -> bool:
        """
        判断目标是否已被摧毁

        参数:
            tgt: 目标字典
        返回:
            True=已摧毁, False=未摧毁
        """
```

### 5.2 状态机 (`src/cluster_scheduler/state_machine.py`)

```python
from enum import Enum

class UAVState(Enum):
    """UAV状态枚举"""
    SEARCH = "SEARCH"       # 扇区搜索巡航
    VERIFY = "VERIFY"       # 多帧验证目标真伪
    TRACK = "TRACK"         # K=2协同盯防
    RELEASE = "RELEASE"     # 目标摧毁后释放


class StateMachine:
    """
    UAV状态机

    状态转换:
        SEARCH -> VERIFY -> TRACK -> RELEASE -> SEARCH
    """

    def __init__(self):
        """初始化状态机，默认状态为SEARCH"""

    @property
    def state(self) -> UAVState:
        """当前状态"""

    def transition(self, obs, dt: float) -> UAVState:
        """
        状态转换逻辑

        参数:
            obs: 观测信息
            dt: 时间步长(秒)
        返回:
            转换后的新状态

        转换条件:
            SEARCH -> VERIFY: 检测到目标
            VERIFY -> TRACK:  确认真目标 + 召唤队友就位
            VERIFY -> SEARCH: 判定诱饵
            TRACK -> RELEASE: dwell >= 20s (目标摧毁)
            RELEASE -> SEARCH: 释放完成
        """

    @property
    def dwell_timer(self) -> float:
        """当前盯防累计时间(秒)"""
```

---

## 六、视觉识别接口 (`src/vision_detect/`)

> **负责人**: 成员4

### 6.1 EMA滤波器 (`src/vision_detect/ema_filter.py`)

```python
class EMATracker:
    """
    指数移动平均(EMA)滤波器 + 线性回归速度估计

    平滑GPS位置噪声(sigma=50m)，估计目标运动速度。
    """

    def __init__(self, alpha: float = 0.3, history: int = 80):
        """
        参数:
            alpha: EMA平滑系数(0.2-0.5), 越大越跟踪噪声
            history: 历史队列长度(帧), 8s@10Hz
        """

    def append(self, lat: float, lon: float) -> None:
        """
        添加新观测位置

        参数:
            lat, lon: 观测到的目标纬度、经度
        """

    @property
    def value(self) -> tuple[float, float] | None:
        """
        返回EMA平滑后的位置

        返回:
            (lat, lon) 或 None(无数据时)
        """

    def speed_mps(self, tick_hz: float = 10.0) -> float:
        """
        估计目标运动速度

        参数:
            tick_hz: 采样频率(Hz)
        返回:
            速度(m/s)
        """

    def reset(self) -> None:
        """重置滤波器状态"""
```

### 6.2 诱饵判别器 (`src/vision_detect/decoy_classifier.py`)

```python
class DecoyClassifier:
    """
    多特征融合诱饵判别器

    诱饵视觉特征与真目标完全一致，仅靠运动学时序变化判别。
    多特征投票: 速度 + 加速度方差 + 运动方向一致性 + 位移跨度
    """

    def __init__(self, speed_confirm: float = 3.0,
                 speed_reject: float = 1.0,
                 timeout: float = 8.0):
        """
        参数:
            speed_confirm: 速度确认阈值(m/s), 真目标>此值
            speed_reject: 速度拒绝阈值(m/s), 诱饵<此值
            timeout: 验证超时(秒)
        """

    def update(self, lat: float, lon: float, dt: float) -> None:
        """
        更新判别器状态

        参数:
            lat, lon: 目标观测位置
            dt: 时间步长(秒)
        """

    @property
    def is_real_target(self) -> bool:
        """
        判定是否为真目标

        返回:
            True=真目标, False=诱饵
        """

    @property
    def confidence(self) -> float:
        """
        返回判别置信度

        返回:
            0.0-1.0, 越高越确信为真目标
        """

    def reset(self) -> None:
        """重置判别器状态"""
```

### 6.3 上报管理器 (`src/vision_detect/report.py`)

```python
class ReportManager:
    """
    report_target上报优化管理器

    只报告EMA平滑后的位置，降低RMSE。
    评分: accuracy = 100 * max(0, 1 - RMSE/120m)
    """

    def __init__(self, report_interval: float = 1.0):
        """
        参数:
            report_interval: 上报间隔(秒)
        """

    def should_report(self, target_id: int) -> bool:
        """
        判断是否应该上报该目标

        参数:
            target_id: 目标ID
        返回:
            True=应该上报
        """

    def get_report_position(self, ema_tracker: 'EMATracker') -> tuple[float, float] | None:
        """
        返回EMA平滑后的上报位置

        参数:
            ema_tracker: EMA滤波器实例
        返回:
            (lat, lon) 或 None(无数据时)
        """

    def mark_destroyed(self, target_id: int) -> None:
        """
        标记目标已摧毁，停止对该目标的上报

        参数:
            target_id: 目标ID
        """
```

---

## 七、主程序接口 (`src/main.py`)

> **负责人**: 队长(成员5)

```python
class HF2026Agent:
    """
    HF2026 无人机集群自主协同跟踪Agent

    继承平台基类 CoopAgent，实现 decide() 主循环。
    """

    def __init__(self, my_uid: int):
        """
        初始化Agent

        参数:
            my_uid: 本机无人机唯一ID
        """

    def configure(self, config: dict) -> None:
        """
        读取全局静态任务参数

        参数:
            config: 从 algorithm.yaml 加载的配置字典
        """

    def reset(self) -> None:
        """每局推演开始清空内部状态"""

    def decide(self, obs, dt: float) -> list:
        """
        主决策循环 (10Hz固定频率调用)

        参数:
            obs: 观测信息(obs.self / obs.comm_inbox / obs.briefing)
            dt: 时间步长(秒)

        返回:
            控制指令列表 [fly_to(...), point_gimbal(...), broadcast(...), ...]

        硬性约束:
            - 仅允许读写自身内部状态
            - 禁止直接操作Redis、读写本地文件
            - 禁止跨实体控制其他无人机
        """
```

---

## 八、接口变更管理

1. 接口契约在 Day 1 晚间全体确认后锁定
2. 任何接口变更需提交变更申请，经队长审批
3. 变更通过后，队长通知全体成员并更新本文档
4. 使用 Mock 对象解耦模块间依赖，接口变更不影响其他模块开发

---

## 九、平台API参考 (参赛手册 6)

### 9.1 Agent生命周期

| 方法 | 必选 | 说明 |
|------|------|------|
| `__init__(my_uid)` | 是 | 初始化，记录本机无人机唯一ID |
| `configure(config)` | 否 | 读取全局静态任务参数 |
| `reset()` | 否 | 每局推演开始清空内部状态 |
| `decide(obs, dt)` | 是 | 解析观测、输出控制指令 |

### 9.2 观测信息Obs三层结构

| 层级 | 字段 | 说明 |
|------|------|------|
| `obs.self` | 坐标/航向/速度/云台/检测结果/通信状态/是否被击毁/是否被干扰 | 本机状态 |
| `obs.comm_inbox` | 队友通信消息列表 | 仅赛题二、三 |
| `obs.briefing` | 地图边界/目标总数/威胁区近似范围/实时得分快照 | 全局静态简报 |

### 9.3 控制命令

| 命令 | 说明 | 适用赛题 |
|------|------|----------|
| `fly_to(lat, lon, alt)` | 导航控制 | 全部 |
| `set_heading(heading)` | 航向控制 | 全部 |
| `set_speed(speed)` | 速度控制 | 全部 |
| `point_gimbal(lat, lon)` | 云台视觉控制 | 全部 |
| `set_gimbal_fov(fov)` | 视场角控制 | 全部 |
| `broadcast(payload)` | 通信控制(<=50字节, <=4Hz) | 赛二/三 |
| `send_to(uid, payload)` | 定向通信 | 赛二/三 |
| `report_target(lat, lon)` | 任务上报 | 全部 |
