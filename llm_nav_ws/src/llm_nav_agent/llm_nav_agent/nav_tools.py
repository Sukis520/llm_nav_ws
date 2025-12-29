"""
Nav2 工具集 - 供 LangChain Agent 调用
提供导航相关的所有工具函数
"""

import time
from typing import Optional, Any
from langchain.tools import tool
from langchain_core.tools import ToolException

# Nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 本地模块
from .utils import create_pose, LocationManager, get_default_location_manager


class Nav2ToolKit:
    """
    Nav2 工具包装类
    使用单例模式管理 Navigator 实例
    """

    _navigator: Optional[BasicNavigator] = None
    _initialized: bool = False
    _location_manager: Optional[LocationManager] = None

    @classmethod
    def get_navigator(cls) -> BasicNavigator:
        """获取单例 Navigator 实例"""
        if cls._navigator is None:
            cls._navigator = BasicNavigator()
        return cls._navigator

    @classmethod
    def get_location_manager(cls) -> LocationManager:
        """获取位置管理器"""
        if cls._location_manager is None:
            cls._location_manager = get_default_location_manager()
        return cls._location_manager

    @classmethod
    def set_location_manager(cls, manager: LocationManager) -> None:
        """设置位置管理器"""
        cls._location_manager = manager

    @classmethod
    def wait_for_nav2(cls, timeout: float = 30.0) -> bool:
        """
        等待 Nav2 服务就绪

        Args:
            timeout: 超时时间（秒）

        Returns:
            是否成功初始化
        """
        if cls._initialized:
            return True

        try:
            nav = cls.get_navigator()
            # 等待 Nav2 激活
            nav.waitUntilNav2Active(localizer="amcl")
            cls._initialized = True
            return True
        except Exception as e:
            print(f"等待 Nav2 失败: {e}")
            return False

    @classmethod
    def is_initialized(cls) -> bool:
        """检查是否已初始化"""
        return cls._initialized

    @classmethod
    def shutdown(cls) -> None:
        """关闭导航器"""
        if cls._navigator is not None:
            cls._navigator.lifecycleShutdown()
            cls._navigator = None
            cls._initialized = False


def _execute_navigation(goal_pose, timeout: float = 120.0) -> str:
    """
    执行导航任务的内部函数

    Args:
        goal_pose: 目标位姿
        timeout: 超时时间（秒）

    Returns:
        结果描述字符串
    """
    navigator = Nav2ToolKit.get_navigator()

    # 设置时间戳
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # 发送导航目标
    navigator.goToPose(goal_pose)

    # 等待任务完成
    start_time = time.time()
    while not navigator.isTaskComplete():
        # 检查超时
        if time.time() - start_time > timeout:
            navigator.cancelTask()
            return "导航超时，任务已取消"

        # 获取反馈信息
        feedback = navigator.getFeedback()
        if feedback:
            # 可以在这里处理进度反馈
            pass

        time.sleep(0.5)

    # 获取结果
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        return "SUCCESS"
    elif result == TaskResult.CANCELED:
        return "CANCELED"
    elif result == TaskResult.FAILED:
        return "FAILED"
    else:
        return f"UNKNOWN:{result}"


# ==================== LangChain 工具定义 ====================


@tool
def navigate_to_coordinate(x: float, y: float, yaw: float = 0.0) -> str:
    """
    导航机器人到指定的地图坐标位置。

    当用户要求机器人移动到某个具体的坐标点时使用此工具。
    例如："去坐标(3, 2)"、"移动到位置x=5, y=3"。

    Args:
        x: 目标位置的 X 坐标（单位：米）
        y: 目标位置的 Y 坐标（单位：米）
        yaw: 到达后的朝向角度（单位：弧度），默认0表示朝向X轴正方向。
             常用值：0=东, 1.57=北, 3.14=西, -1.57=南

    Returns:
        导航结果的描述字符串

    Examples:
        navigate_to_coordinate(3.0, 2.0) -> 导航到坐标(3, 2)
        navigate_to_coordinate(5.0, 0.0, 1.57) -> 导航到(5, 0)并朝北
    """
    try:
        # 验证输入
        x = float(x)
        y = float(y)
        yaw = float(yaw)

        # 创建目标位姿
        goal_pose = create_pose(x, y, yaw)

        # 执行导航
        result = _execute_navigation(goal_pose)

        if result == "SUCCESS":
            return f"成功到达坐标 ({x:.1f}, {y:.1f})"
        elif result == "CANCELED":
            return f"导航到 ({x:.1f}, {y:.1f}) 被取消"
        elif result == "FAILED":
            return f"无法到达坐标 ({x:.1f}, {y:.1f})，可能存在障碍物或路径不可达"
        else:
            return f"导航返回未知状态: {result}"

    except ValueError as e:
        return f"坐标参数错误: {e}"
    except Exception as e:
        return f"导航过程出错: {str(e)}"


@tool
def navigate_to_named_location(location_name: str) -> str:
    """
    导航机器人到预设的命名位置。

    当用户提到具体的地点名称时使用此工具，如"去厨房"、"到客厅"、"回卧室"等。
    这比使用坐标更方便，因为用户不需要知道具体坐标。

    Args:
        location_name: 位置名称。可用的位置包括：厨房、客厅、卧室、门口、充电桩、阳台、书房、餐厅等。
                      使用 get_available_locations 工具可以查看所有可用位置。

    Returns:
        导航结果的描述字符串

    Examples:
        navigate_to_named_location("厨房") -> 导航到厨房
        navigate_to_named_location("客厅") -> 导航到客厅
    """
    try:
        location_manager = Nav2ToolKit.get_location_manager()

        # 查找位置
        location = location_manager.get_location(location_name)

        if location is None:
            # 尝试查找相似位置
            similar = location_manager.find_similar_locations(location_name)
            available = location_manager.get_location_names()

            if similar:
                return (
                    f"未找到位置'{location_name}'。您是否想去：{', '.join(similar)}？"
                )
            else:
                return f"未知位置：'{location_name}'。可用位置：{', '.join(available)}"

        # 创建目标位姿
        goal_pose = create_pose(location.x, location.y, location.yaw)

        # 执行导航
        result = _execute_navigation(goal_pose)

        if result == "SUCCESS":
            return f"已到达{location_name}"
        elif result == "CANCELED":
            return f"前往{location_name}的导航被取消"
        elif result == "FAILED":
            return f"无法到达{location_name}，路径可能被阻挡"
        else:
            return f"导航到{location_name}返回未知状态"

    except Exception as e:
        return f"导航到{location_name}时出错: {str(e)}"


@tool
def cancel_navigation() -> str:
    """
    取消当前正在进行的导航任务。

    当用户要求停止移动、取消导航、别动了、停下来等时使用此工具。
    这会立即停止机器人的移动。

    Returns:
        取消操作的结果描述

    Examples:
        用户说"停下"、"别动了"、"取消导航" -> 调用此工具
    """
    try:
        navigator = Nav2ToolKit.get_navigator()
        navigator.cancelTask()
        return "已取消当前导航任务，机器人已停止"
    except Exception as e:
        return f"取消导航时出错: {str(e)}"


@tool
def get_current_position() -> str:
    """
    获取机器人当前在地图中的位置坐标。

    当用户询问机器人在哪里、当前位置是什么时使用此工具。

    Returns:
        当前位置的坐标和描述信息

    Examples:
        用户问"你在哪"、"现在的位置"、"当前坐标" -> 调用此工具
    """
    try:
        # 注意：BasicNavigator 没有直接提供获取当前位置的方法
        # 实际应用中需要通过 TF2 或订阅 AMCL pose 来获取
        # 这里提供一个框架，实际实现需要根据系统配置

        navigator = Nav2ToolKit.get_navigator()

        # 方法1：如果 navigator 有 initial_pose 记录
        # 方法2：通过 TF 查询 base_link 在 map 坐标系中的位置

        # 这里返回一个说明，实际使用时需要实现具体逻辑
        return "当前位置查询功能需要 TF2 支持。请确保 AMCL 或其他定位模块正在运行。"

    except Exception as e:
        return f"获取位置时出错: {str(e)}"


@tool
def get_available_locations() -> str:
    """
    获取所有可用的预设位置列表及其描述。

    当用户询问可以去哪些地方、有哪些位置、能导航到哪里时使用此工具。

    Returns:
        包含所有可用位置名称和描述的列表

    Examples:
        用户问"你能去哪"、"有哪些地方"、"可用位置" -> 调用此工具
    """
    try:
        location_manager = Nav2ToolKit.get_location_manager()
        locations = location_manager.get_all_locations()

        if not locations:
            return "当前没有预设位置。请先配置位置信息。"

        # 构建位置列表字符串
        location_list = []
        for name, loc in locations.items():
            if loc.description:
                location_list.append(f"  • {name}: {loc.description}")
            else:
                location_list.append(f"  • {name}")

        result = "可用的预设位置：\n" + "\n".join(location_list)
        return result

    except Exception as e:
        return f"获取位置列表时出错: {str(e)}"


@tool
def set_initial_pose(x: float, y: float, yaw: float = 0.0) -> str:
    """
    设置机器人的初始位姿（用于定位初始化）。

    当需要告诉机器人它当前在地图上的位置时使用此工具。
    这通常在启动时或机器人迷失方向时使用。

    Args:
        x: 当前 X 坐标（米）
        y: 当前 Y 坐标（米）
        yaw: 当前朝向角度（弧度）

    Returns:
        设置结果描述
    """
    try:
        navigator = Nav2ToolKit.get_navigator()
        initial_pose = create_pose(x, y, yaw)
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()

        navigator.setInitialPose(initial_pose)

        return f"已设置初始位姿: ({x:.1f}, {y:.1f}), 朝向 {yaw:.2f} 弧度"

    except Exception as e:
        return f"设置初始位姿时出错: {str(e)}"


@tool
def navigate_through_poses(waypoints_str: str) -> str:
    """
    让机器人依次经过多个路径点。

    当用户要求机器人巡逻、依次访问多个地点时使用此工具。

    Args:
        waypoints_str: 路径点字符串，格式为 "x1,y1;x2,y2;x3,y3"
                      例如 "0,0;1,1;2,0" 表示依次经过(0,0), (1,1), (2,0)

    Returns:
        导航结果描述

    Examples:
        navigate_through_poses("0,0;3,2;5,0") -> 依次经过三个点
    """
    try:
        # 解析路径点
        waypoints = []
        for point_str in waypoints_str.split(";"):
            coords = point_str.strip().split(",")
            if len(coords) >= 2:
                x = float(coords[0].strip())
                y = float(coords[1].strip())
                yaw = float(coords[2].strip()) if len(coords) > 2 else 0.0
                waypoints.append(create_pose(x, y, yaw))

        if not waypoints:
            return "无法解析路径点，请使用格式: x1,y1;x2,y2;x3,y3"

        navigator = Nav2ToolKit.get_navigator()

        # 设置时间戳
        for wp in waypoints:
            wp.header.stamp = navigator.get_clock().now().to_msg()

        # 执行路径点导航
        navigator.goThroughPoses(waypoints)

        # 等待完成
        while not navigator.isTaskComplete():
            time.sleep(0.5)

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            return f"已完成 {len(waypoints)} 个路径点的巡航"
        else:
            return f"巡航未完全完成，状态: {result}"

    except Exception as e:
        return f"路径点导航出错: {str(e)}"


# ==================== 导出所有工具 ====================

ALL_NAV_TOOLS = [
    navigate_to_coordinate,
    navigate_to_named_location,
    cancel_navigation,
    get_current_position,
    get_available_locations,
    set_initial_pose,
    navigate_through_poses,
]
