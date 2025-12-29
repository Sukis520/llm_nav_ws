"""
LLM Nav Agent - 基于大语言模型的ROS2导航代理

这个包提供了一个将 LangChain 与 ROS2 Nav2 结合的解决方案，
允许用户通过自然语言控制机器人导航。
"""

__version__ = "0.1.0"
__author__ = "Your Name"

# 从子模块导入主要组件
from .utils import create_pose, load_locations, LocationManager
from .nav_tools import (
    Nav2ToolKit,
    navigate_to_coordinate,
    navigate_to_named_location,
    cancel_navigation,
    get_current_position,
    get_available_locations,
    ALL_NAV_TOOLS,
)

__all__ = [
    # 版本信息
    "__version__",
    # 工具函数
    "create_pose",
    "load_locations",
    "LocationManager",
    # Nav2 工具包
    "Nav2ToolKit",
    # LangChain 工具
    "navigate_to_coordinate",
    "navigate_to_named_location",
    "cancel_navigation",
    "get_current_position",
    "get_available_locations",
    "ALL_NAV_TOOLS",
]
