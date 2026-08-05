"""
src/motion_control/main.py
motion_control 模块的统一入口，对外只暴露 MotionCtrl 类及其两个方法：
- hover() : 让无人机悬停（绕小圈）
- get_action(cmd) : 根据命令返回相应的控制指令列表

内部封装了搜索 (SectorSearch)、跟踪 (LoiterTracker) 和几何工具 (geo)。
支持通过 multi_drone 参数切换单机/多机模式。
"""
from typing import List, Union, Tuple, Dict, Any, Optional

from .geo import DEFAULT_ALTITUDE
from .search import SectorSearch
from .tracker import LoiterTracker
from competition.sdk.core.commands import fly_to, Command


class MotionCtrl:
    """
    运动控制类，统一管理搜索、跟踪、悬停。

    用法：
        初始化时传入 uav_name 和 multi_drone 标志。
        在 decide() 循环中，先通过 get_action 传入当前无人机状态和命令，
        即可获得对应的 Command 列表。

        示例：
            ctrl = MotionCtrl(uav_name="uav_alpha", multi_drone=False)

            # 在 decide 中：
            # 1. 搜索模式
            cmds = ctrl.get_action({
                'mode': 'search',
                'lat': obs.self.lat,
                'lon': obs.self.lon,
                'alt': obs.self.alt,
                'yaw': obs.self.heading
            })

            # 2. 跟踪目标（传入目标坐标）
            cmds = ctrl.get_action({
                'mode': 'track',
                'target': (target_lat, target_lon),
                'lat': obs.self.lat,
                'lon': obs.self.lon,
                'alt': obs.self.alt,
                'yaw': obs.self.heading
            })

            # 3. 悬停（先确保之前已更新过状态）
            cmds = ctrl.hover()   # 或 ctrl.get_action('hover')
    """

    def __init__(self, uav_name: str, multi_drone: bool = False):
        """
        Args:
            uav_name: 无人机标识，如 "uav_alpha" 或 "20001"
            multi_drone: True 开启多机模式（扇区搜索 + 双槽跟踪），False 为单机模式
        """
        self.uav_name = uav_name
        self.multi_drone = multi_drone

        # 内部模块
        self.search = SectorSearch(uav_name, multi_drone=multi_drone)
        self.tracker = LoiterTracker(uav_name, multi_drone=multi_drone)

        # 内部状态缓存
        self._state: Optional[Dict[str, float]] = None  # {'lat', 'lon', 'alt', 'yaw'}
        self._target: Optional[Tuple[float, float]] = None
        self._mode: str = "search"   # "search" | "track" | "hover"

    def _ensure_state(self):
        """确保已缓存无人机状态，否则抛出异常"""
        if self._state is None:
            raise RuntimeError("UAV state not set. Call get_action with state first.")

    def _update_state(self, lat: float, lon: float, alt: float, yaw: float):
        """更新内部缓存的无人机状态"""
        self._state = {'lat': lat, 'lon': lon, 'alt': alt, 'yaw': yaw}

    def _generate_search_commands(self) -> List[Command]:
        """生成搜索命令（根据内部缓存的状态）"""
        self._ensure_state()
        return self.search.generate_commands(
            self.uav_name,
            self._state['lat'],
            self._state['lon']
        )

    def _generate_track_commands(self) -> List[Command]:
        """生成跟踪命令（根据内部缓存的状态和目标）"""
        self._ensure_state()
        if self._target is None:
            raise RuntimeError("No target set for track mode")
        self.tracker.set_target(self._target[0], self._target[1], slot=0)
        return self.tracker.generate_commands(
            self.uav_name,
            self._state['lat'],
            self._state['lon'],
            self._state['alt'],
            self._state['yaw']
        )

    def hover(self) -> List[Command]:
        """
        悬停命令：让无人机在当前位置绕小圈（半径 100m，速度 15m/s）。
        需要已经通过 get_action 更新过状态。
        """
        self._ensure_state()
        lat, lon, alt = self._state['lat'], self._state['lon'], self._state['alt']
        # 固定翼不能悬停，绕小圈模拟
        return [fly_to(lat, lon, alt=alt, speed=15.0, loiter_radius=100.0)]

    def get_action(self, cmd: Union[str, Dict, Tuple]) -> List[Command]:
        """
        统一命令接口，根据 cmd 返回相应的控制指令列表。

        cmd 支持三种格式：
            1. 字符串:
                - "search" : 切换到搜索模式，返回搜索命令
                - "track"  : 切换到跟踪模式（需已设置目标），返回跟踪命令
                - "hover"  : 切换到悬停模式，返回悬停命令
            2. 字典:
                {
                    'mode': 'search' | 'track' | 'hover',      # 可选，默认为当前模式
                    'target': (lat, lon),                       # 可选，用于跟踪模式
                    'lat': float, 'lon': float,                # 必须，用于更新状态
                    'alt': float, 'yaw': float                 # 必须
                }
            3. 元组 (lat, lon):
                直接视为目标坐标，自动切换到跟踪模式，并返回跟踪命令
                （需要之前已更新过状态）
        """
        # --- 处理字典格式 ---
        if isinstance(cmd, dict):
            # 提取状态（如果提供）
            if all(k in cmd for k in ('lat', 'lon', 'alt', 'yaw')):
                self._update_state(cmd['lat'], cmd['lon'], cmd['alt'], cmd['yaw'])

            # 提取模式和目标
            mode = cmd.get('mode', self._mode)
            target = cmd.get('target')  # 可能是 (lat, lon) 或 None

            if target is not None:
                self._target = (target[0], target[1])
                self._mode = 'track'
            else:
                self._mode = mode

            # 根据模式生成命令
            if self._mode == 'search':
                return self._generate_search_commands()
            elif self._mode == 'track':
                return self._generate_track_commands()
            elif self._mode == 'hover':
                return self.hover()
            else:
                return []

        # --- 处理元组格式 (lat, lon) ---
        elif isinstance(cmd, tuple) and len(cmd) == 2:
            self._target = (cmd[0], cmd[1])
            self._mode = 'track'
            return self._generate_track_commands()

        # --- 处理字符串格式 ---
        elif isinstance(cmd, str):
            self._mode = cmd
            if cmd == 'search':
                return self._generate_search_commands()
            elif cmd == 'track':
                return self._generate_track_commands()
            elif cmd == 'hover':
                return self.hover()
            else:
                return []

        else:
            raise ValueError(f"Unsupported cmd type: {type(cmd)}")

    def reset(self):
        """重置所有内部状态（每局开始时调用，可选）"""
        self.search.reset()
        self.tracker.reset()
        self._state = None
        self._target = None
        self._mode = "search"


'''在Agent中的使用示例
# 在 Agent 的 __init__ 中
self.ctrl = MotionCtrl(uav_name="uav_alpha", multi_drone=False)   # 单机模式
# 或 self.ctrl = MotionCtrl(uav_name="uav_alpha", multi_drone=True)  # 多机模式

# 在 decide() 中
if not self.target_found:
    # 搜索模式：传入状态
    cmds = self.ctrl.get_action({
        'mode': 'search',
        'lat': obs.self.lat,
        'lon': obs.self.lon,
        'alt': obs.self.alt,
        'yaw': obs.self.heading
    })
else:
    # 跟踪目标：传入状态和目标
    cmds = self.ctrl.get_action({
        'mode': 'track',
        'target': (target_lat, target_lon),
        'lat': obs.self.lat,
        'lon': obs.self.lon,
        'alt': obs.self.alt,
        'yaw': obs.self.heading
    })

# 或者悬停（假设状态已经缓存在内部）
cmds = self.ctrl.hover()

# 返回命令列表
return cmds
'''