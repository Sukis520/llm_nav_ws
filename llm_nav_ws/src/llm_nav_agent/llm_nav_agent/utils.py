"""
工具函数模块
提供坐标转换、位置管理等辅助功能
"""

import os
import math
import yaml
from typing import Dict, Optional, Tuple, List
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped, Quaternion
from ament_index_python.packages import get_package_share_directory


@dataclass
class Location:
    """位置数据类"""

    name: str
    x: float
    y: float
    yaw: float = 0.0
    description: str = ""


def euler_to_quaternion(
    roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> Quaternion:
    """
    将欧拉角转换为四元数

    Args:
        roll: 横滚角（弧度）
        pitch: 俯仰角（弧度）
        yaw: 偏航角（弧度）

    Returns:
        geometry_msgs.msg.Quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """
    将四元数转换为欧拉角

    Args:
        q: geometry_msgs.msg.Quaternion

    Returns:
        (roll, pitch, yaw) 弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def create_pose(
    x: float, y: float, yaw: float = 0.0, frame_id: str = "map", stamp=None
) -> PoseStamped:
    """
    创建 ROS2 PoseStamped 消息

    Args:
        x: X坐标（米）
        y: Y坐标（米）
        yaw: 偏航角（弧度），默认0表示朝向X轴正方向
        frame_id: 坐标系，默认'map'
        stamp: 时间戳，默认None

    Returns:
        geometry_msgs.msg.PoseStamped
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id

    if stamp is not None:
        pose.header.stamp = stamp

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # 将yaw角转换为四元数
    pose.pose.orientation = euler_to_quaternion(yaw=yaw)

    return pose


def load_locations(yaml_path: str) -> Dict[str, Location]:
    """
    从 YAML 文件加载位置配置

    Args:
        yaml_path: YAML 文件路径

    Returns:
        位置字典 {名称: Location对象}
    """
    locations = {}

    if not os.path.exists(yaml_path):
        print(f"警告: 位置配置文件不存在: {yaml_path}")
        return locations

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if data and "locations" in data:
            for name, info in data["locations"].items():
                locations[name] = Location(
                    name=name,
                    x=float(info.get("x", 0.0)),
                    y=float(info.get("y", 0.0)),
                    yaw=float(info.get("yaw", 0.0)),
                    description=info.get("description", ""),
                )

        # 处理别名
        if data and "aliases" in data:
            for alias, target in data["aliases"].items():
                if target in locations:
                    # 创建别名位置（复制目标位置但使用别名作为名称）
                    orig = locations[target]
                    locations[alias] = Location(
                        name=alias,
                        x=orig.x,
                        y=orig.y,
                        yaw=orig.yaw,
                        description=f"别名 -> {target}",
                    )

    except Exception as e:
        print(f"加载位置配置出错: {e}")

    return locations


class LocationManager:
    """
    位置管理器
    管理预设位置的加载、查询和更新
    """

    _instance: Optional["LocationManager"] = None
    _locations: Dict[str, Location] = {}
    _initialized: bool = False

    def __new__(cls):
        """单例模式"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not self._initialized:
            self._locations = {}
            self._initialized = True

    def load_from_yaml(self, yaml_path: str) -> bool:
        """从 YAML 文件加载位置"""
        self._locations = load_locations(yaml_path)
        return len(self._locations) > 0

    def load_from_package(
        self,
        package_name: str = "llm_nav_agent",
        config_file: str = "config/locations.yaml",
    ) -> bool:
        """从 ROS2 包加载位置配置"""
        try:
            pkg_dir = get_package_share_directory(package_name)
            yaml_path = os.path.join(pkg_dir, config_file)
            return self.load_from_yaml(yaml_path)
        except Exception as e:
            print(f"从包加载位置配置失败: {e}")
            return False

    def get_location(self, name: str) -> Optional[Location]:
        """获取指定名称的位置"""
        return self._locations.get(name)

    def get_all_locations(self) -> Dict[str, Location]:
        """获取所有位置"""
        return self._locations.copy()

    def get_location_names(self) -> List[str]:
        """获取所有位置名称"""
        return list(self._locations.keys())

    def add_location(
        self, name: str, x: float, y: float, yaw: float = 0.0, description: str = ""
    ) -> None:
        """添加新位置"""
        self._locations[name] = Location(
            name=name, x=x, y=y, yaw=yaw, description=description
        )

    def remove_location(self, name: str) -> bool:
        """删除位置"""
        if name in self._locations:
            del self._locations[name]
            return True
        return False

    def has_location(self, name: str) -> bool:
        """检查位置是否存在"""
        return name in self._locations

    def find_similar_locations(self, query: str) -> List[str]:
        """
        查找相似的位置名称（简单的模糊匹配）
        """
        query_lower = query.lower()
        similar = []
        for name in self._locations.keys():
            if query_lower in name.lower() or name.lower() in query_lower:
                similar.append(name)
        return similar


# 默认位置配置（当无法加载 YAML 时使用）
DEFAULT_LOCATIONS = {
    "厨房": Location("厨房", 3.0, 2.0, 0.0, "厨房区域"),
    "客厅": Location("客厅", 0.0, 0.0, 0.0, "客厅中央"),
    "卧室": Location("卧室", -2.0, 3.0, 1.57, "主卧室"),
    "门口": Location("门口", 5.0, 0.0, 3.14, "入户门"),
    "充电桩": Location("充电桩", -1.0, -1.0, 0.0, "机器人充电位置"),
}


def get_default_location_manager() -> LocationManager:
    """
    获取默认的位置管理器实例
    会尝试从包配置加载，失败则使用默认配置
    """
    manager = LocationManager()

    # 尝试从包加载
    if not manager.load_from_package():
        # 使用默认配置
        for name, loc in DEFAULT_LOCATIONS.items():
            manager.add_location(name, loc.x, loc.y, loc.yaw, loc.description)

    return manager
