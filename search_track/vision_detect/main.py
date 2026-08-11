# src/main.py
from typing import List
import math

# 导入你的视觉识别模块
from src.vision_detect import DecoyClassifier # type: ignore

# 导入比赛 SDK
try:
    from competition.sdk.core.agent import CoopAgent # type: ignore
    from competition.sdk.core.commands import fly_to, point_gimbal, report_target, set_speed # type: ignore
    from competition.sdk.core.obs import Obs # type: ignore
except ImportError:
    # 本地测试用占位
    class CoopAgent:
        def __init__(self, my_uid: int):
            self.my_uid = my_uid
    class Obs:
        pass
    def fly_to(lat, lon, alt=500): return f"fly_to({lat},{lon},{alt})"
    def point_gimbal(lat, lon): return f"point_gimbal({lat},{lon})"
    def report_target(lat, lon): return f"report_target({lat},{lon})"
    def set_speed(speed): return f"set_speed({speed})"


class MyAgent(CoopAgent):
    """
    赛题二：多机协同识别 Agent
    集成视觉识别模块，实现诱饵判别与目标上报
    """
    
    def __init__(self, my_uid: int):
        super().__init__(my_uid)
        self.my_uid = my_uid
        
        # 每个目标维护一个分类器
        self.classifiers = {}
        self.destroyed_targets = set()
        
        # 计数
        self.frame_count = 0
        self.report_count = 0
        
        print(f"[Agent {my_uid}] 初始化完成，视觉识别模块已加载")
    
    def decide(self, obs, dt: float) -> List:
        """
        核心决策函数，平台以 10Hz 调用
        """
        self.frame_count += 1
        commands = []
        
        # 1. 提取检测目标
        detections = []
        if hasattr(obs, 'self') and hasattr(obs.self, 'detections'):
            for det in obs.self.detections:
                detections.append({
                    'id': getattr(det, 'id', -1),
                    'lat': getattr(det, 'lat', 0.0),
                    'lon': getattr(det, 'lon', 0.0),
                })
        
        # 调试：每50帧打印一次检测数量
        if self.frame_count % 50 == 0:
            print(f"[Agent {self.my_uid}] 帧{self.frame_count}: 检测到{len(detections)}个目标")
        
        # 2. 对每个检测目标进行识别
        for det in detections:
            target_id = det['id']
            lat, lon = det['lat'], det['lon']
            
            # 跳过已摧毁目标
            if target_id in self.destroyed_targets:
                continue
            
            # 获取或创建分类器
            if target_id not in self.classifiers:
                self.classifiers[target_id] = DecoyClassifier(
                    target_id=target_id,
                    debug=True  # 开启调试日志
                )
                print(f"[Agent {self.my_uid}] 创建新目标分类器: ID={target_id}")
            
            cls = self.classifiers[target_id]
            
            # 更新分类器
            cls.update(lat, lon, dt)
            
            # 判断是否上报
            if cls.should_report():
                smooth_lat, smooth_lon = cls.get_report_position()
                if smooth_lat is not None:
                    # 上报平滑位置
                    commands.append(report_target(smooth_lat, smooth_lon))
                    cls.mark_reported()
                    self.report_count += 1
                    print(f"[Agent {self.my_uid}] ✅ 上报目标 {target_id}: "
                          f"({smooth_lat:.6f}, {smooth_lon:.6f}) 置信度={cls.confidence:.2f}")
        
        # 3. 如果有目标，飞向第一个目标
        if detections:
            target = detections[0]
            commands.append(fly_to(target['lat'], target['lon'], altitude=500))
            commands.append(point_gimbal(target['lat'], target['lon']))
        else:
            # 无目标时巡航（扇区搜索由控制模块负责）
            pass
        
        return commands


# ==========================================
# 本地测试入口
# ==========================================
if __name__ == "__main__":
    print("=== 视觉识别模块测试 ===")
    agent = MyAgent(my_uid=20001)
    
    # 模拟观测对象
    class MockSelf:
        detections = []
    
    class MockObs:
        def __init__(self):
            self.self = MockSelf()
            self.comm_inbox = []
            self.briefing = {}
    
    obs = MockObs()
    
    import random
    print("模拟真目标运动...")
    for i in range(50):
        lat = 27.0 + i * 0.0002 + random.uniform(-0.00005, 0.00005)
        lon = 125.0 + i * 0.0003 + random.uniform(-0.00005, 0.00005)
        
        class MockDet:
            def __init__(self, id, lat, lon):
                self.id = id
                self.lat = lat
                self.lon = lon
        
        obs.self.detections = [MockDet(1, lat, lon)]
        cmds = agent.decide(obs, 0.1)
        
        if i % 10 == 0 and cmds:
            print(f"  第{i}帧: 输出{len(cmds)}条指令")
    
    print(f"\n测试完成！共上报 {agent.report_count} 次")
    print(f"分类器状态: {agent.classifiers[1].get_debug_info() if 1 in agent.classifiers else '无'}")