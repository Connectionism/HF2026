"""
cluster_scheduler 模块对外统一入口

对外暴露:
    CooperativeCoordinator  — K=2 双机协同目标分配器
    States                  — 状态机常量（SEARCH / VERIFY / TRACK）

内部实现:
    coordinator.py  — CooperativeCoordinator 类

说明:
    状态机处理逻辑（_handle_search/_handle_verify/_handle_track）
    由 drone_agent.py 的 DroneAgent 类实现，本模块只负责协同调度。
"""
from .coordinator import CooperativeCoordinator


class States:
    """状态机常量（SEARCH → VERIFY → TRACK 流转）。"""
    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"


__all__ = ["CooperativeCoordinator", "States"]
