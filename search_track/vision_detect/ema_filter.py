import math
from collections import deque
from typing import Optional, Tuple, List


class EMATracker:
    """
    指数移动平均（EMA）滤波器
    维护一个固定长度的历史队列，用于平滑位置和估计运动特征
    
    Attributes:
        alpha: 平滑系数 (0~1)，越大对最新观测越敏感
        history: 位置历史队列，用于速度估计
        max_history: 最大历史长度
        _value: 当前平滑后的位置 (lat, lon)
        _initialized: 是否已初始化
    """
    
    def __init__(self, alpha: float = 0.25, max_history: int = 80):
        """
        初始化 EMA 滤波器
        
        Args:
            alpha: 平滑系数，推荐范围 0.2~0.4（新规则下诱饵也移动，适当降低alpha增强平滑）
            max_history: 历史队列最大长度，用于速度估计
        """
        self.alpha = alpha
        self.max_history = max_history
        self.history: deque = deque(maxlen=max_history)
        self._value: Optional[Tuple[float, float]] = None
        self._initialized = False
    
    def append(self, lat: float, lon: float) -> None:
        """
        添加一个新的观测值
        
        Args:
            lat: 纬度
            lon: 经度
        """
        if not self._initialized:
            self._value = (lat, lon)
            self._initialized = True
        else:
            prev_lat, prev_lon = self._value
            new_lat = self.alpha * lat + (1 - self.alpha) * prev_lat
            new_lon = self.alpha * lon + (1 - self.alpha) * prev_lon
            self._value = (new_lat, new_lon)
        
        self.history.append((lat, lon, self._get_timestamp()))
    
    def _get_timestamp(self) -> float:
        import time
        return time.monotonic()
    
    @property
    def value(self) -> Optional[Tuple[float, float]]:
        return self._value
    
    @property
    def raw_history(self) -> List[Tuple[float, float, float]]:
        return list(self.history)
    
    def speed_mps(self, tick_hz: float = 10.0) -> float:
        """通过线性回归估计当前速度（米/秒）"""
        if len(self.history) < 4:
            return 0.0
        
        samples = list(self.history)
        if len(samples) > 25:
            samples = samples[-25:]
        
        n = len(samples)
        if n < 4:
            return 0.0
        
        t0 = samples[0][2]
        times = [(s[2] - t0) for s in samples]
        
        ref_lat = sum(s[0] for s in samples) / n
        ref_lon = sum(s[1] for s in samples) / n
        lat_scale = 111320.0
        lon_scale = 111320.0 * math.cos(math.radians(ref_lat))
        
        xs = [(s[0] - ref_lat) * lat_scale for s in samples]
        ys = [(s[1] - ref_lon) * lon_scale for s in samples]
        
        t_mean = sum(times) / n
        x_mean = sum(xs) / n
        y_mean = sum(ys) / n
        
        cov_tx = sum((times[i] - t_mean) * (xs[i] - x_mean) for i in range(n))
        cov_ty = sum((times[i] - t_mean) * (ys[i] - y_mean) for i in range(n))
        var_t = sum((times[i] - t_mean) ** 2 for i in range(n))
        
        if var_t < 1e-10:
            return 0.0
        
        vx = cov_tx / var_t
        vy = cov_ty / var_t
        
        return math.sqrt(vx * vx + vy * vy)
    
    def speed_variance(self, window: int = 15) -> float:
        """
        计算速度方差——用于区分真目标和诱饵
        
        真目标（匀速行驶）：速度方差小
        诱饵（随机游走/噪声驱动）：速度方差大
        """
        if len(self.history) < window + 1:
            return 0.0
        
        samples = list(self.history)[-window-1:]
        speeds = []
        for i in range(1, len(samples)):
            lat1, lon1, t1 = samples[i-1]
            lat2, lon2, t2 = samples[i]
            dt = t2 - t1
            if dt < 0.001:
                continue
            dist = haversine_distance(lat1, lon1, lat2, lon2)
            speeds.append(dist / dt)
        
        if len(speeds) < 3:
            return 0.0
        
        mean_speed = sum(speeds) / len(speeds)
        variance = sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)
        return variance
    
    def displacement(self) -> float:
        """计算历史窗口内的总位移（米）"""
        if len(self.history) < 2:
            return 0.0
        
        first = self.history[0]
        last = self.history[-1]
        return haversine_distance(first[0], first[1], last[0], last[1])
    
    def direction_change_variance(self, window: int = 20) -> float:
        """
        计算方向变化方差——真目标运动有规律，诱饵方向变化大
        """
        if len(self.history) < window + 1:
            return 0.0
        
        samples = list(self.history)[-window-1:]
        directions = []
        
        for i in range(1, len(samples)):
            lat1, lon1, _ = samples[i-1]
            lat2, lon2, _ = samples[i]
            dx = (lon2 - lon1) * 111320 * math.cos(math.radians((lat1 + lat2) / 2))
            dy = (lat2 - lat1) * 111320
            angle = math.atan2(dy, dx)
            directions.append(angle)
        
        if len(directions) < 3:
            return 0.0
        
        # 计算方向变化
        changes = []
        for i in range(1, len(directions)):
            diff = directions[i] - directions[i-1]
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            changes.append(diff)
        
        if len(changes) < 2:
            return 0.0
        
        mean_change = sum(changes) / len(changes)
        variance = sum((c - mean_change) ** 2 for c in changes) / len(changes)
        return variance
    
    def reset(self) -> None:
        self.history.clear()
        self._value = None
        self._initialized = False


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点之间的球面距离（米）"""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c