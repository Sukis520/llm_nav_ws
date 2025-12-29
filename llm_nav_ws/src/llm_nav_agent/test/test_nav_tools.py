#!/usr/bin/env python3
"""
Nav2 工具单元测试
测试导航工具的基本功能（不需要实际运行 Nav2）
"""

import unittest
import math
from unittest.mock import MagicMock, patch

import sys
import os

# 添加包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from llm_nav_agent.utils import (
    create_pose,
    euler_to_quaternion,
    quaternion_to_euler,
    Location,
    LocationManager,
    load_locations,
)


class TestUtils(unittest.TestCase):
    """测试工具函数"""

    def test_create_pose_basic(self):
        """测试基本的位姿创建"""
        pose = create_pose(1.0, 2.0)

        self.assertEqual(pose.pose.position.x, 1.0)
        self.assertEqual(pose.pose.position.y, 2.0)
        self.assertEqual(pose.pose.position.z, 0.0)
        self.assertEqual(pose.header.frame_id, "map")

    def test_create_pose_with_yaw(self):
        """测试带朝向的位姿创建"""
        yaw = math.pi / 2  # 90度
        pose = create_pose(0.0, 0.0, yaw)

        # 检查四元数是否正确
        # 对于纯yaw旋转: q.z = sin(yaw/2), q.w = cos(yaw/2)
        expected_z = math.sin(yaw / 2)
        expected_w = math.cos(yaw / 2)

        self.assertAlmostEqual(pose.pose.orientation.z, expected_z, places=5)
        self.assertAlmostEqual(pose.pose.orientation.w, expected_w, places=5)

    def test_create_pose_custom_frame(self):
        """测试自定义坐标系"""
        pose = create_pose(1.0, 1.0, frame_id="odom")
        self.assertEqual(pose.header.frame_id, "odom")

    def test_euler_to_quaternion_identity(self):
        """测试零角度转换"""
        q = euler_to_quaternion(0, 0, 0)

        self.assertAlmostEqual(q.x, 0.0, places=5)
        self.assertAlmostEqual(q.y, 0.0, places=5)
        self.assertAlmostEqual(q.z, 0.0, places=5)
        self.assertAlmostEqual(q.w, 1.0, places=5)

    def test_euler_quaternion_roundtrip(self):
        """测试欧拉角和四元数的往返转换"""
        original_yaw = 1.57  # 约90度

        q = euler_to_quaternion(0, 0, original_yaw)
        roll, pitch, yaw = quaternion_to_euler(q)

        self.assertAlmostEqual(yaw, original_yaw, places=4)
        self.assertAlmostEqual(roll, 0.0, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)


class TestLocation(unittest.TestCase):
    """测试 Location 数据类"""

    def test_location_creation(self):
        """测试位置对象创建"""
        loc = Location("测试位置", 1.0, 2.0, 0.5, "这是一个测试")

        self.assertEqual(loc.name, "测试位置")
        self.assertEqual(loc.x, 1.0)
        self.assertEqual(loc.y, 2.0)
        self.assertEqual(loc.yaw, 0.5)
        self.assertEqual(loc.description, "这是一个测试")

    def test_location_defaults(self):
        """测试位置对象默认值"""
        loc = Location("简单位置", 0.0, 0.0)

        self.assertEqual(loc.yaw, 0.0)
        self.assertEqual(loc.description, "")


class TestLocationManager(unittest.TestCase):
    """测试位置管理器"""

    def setUp(self):
        """每个测试前重置单例"""
        LocationManager._instance = None
        LocationManager._locations = {}
        LocationManager._initialized = False

    def test_singleton(self):
        """测试单例模式"""
        manager1 = LocationManager()
        manager2 = LocationManager()

        self.assertIs(manager1, manager2)

    def test_add_location(self):
        """测试添加位置"""
        manager = LocationManager()
        manager.add_location("测试", 1.0, 2.0, 0.0, "测试位置")

        self.assertTrue(manager.has_location("测试"))

        loc = manager.get_location("测试")
        self.assertIsNotNone(loc)
        self.assertEqual(loc.x, 1.0)
        self.assertEqual(loc.y, 2.0)

    def test_remove_location(self):
        """测试删除位置"""
        manager = LocationManager()
        manager.add_location("临时", 0.0, 0.0)

        self.assertTrue(manager.has_location("临时"))

        result = manager.remove_location("临时")
        self.assertTrue(result)
        self.assertFalse(manager.has_location("临时"))

    def test_get_location_names(self):
        """测试获取位置名称列表"""
        manager = LocationManager()
        manager.add_location("位置A", 0.0, 0.0)
        manager.add_location("位置B", 1.0, 1.0)

        names = manager.get_location_names()
        self.assertIn("位置A", names)
        self.assertIn("位置B", names)
        self.assertEqual(len(names), 2)

    def test_find_similar_locations(self):
        """测试模糊查找位置"""
        manager = LocationManager()
        manager.add_location("厨房", 0.0, 0.0)
        manager.add_location("厨房阳台", 1.0, 0.0)
        manager.add_location("客厅", 2.0, 0.0)

        similar = manager.find_similar_locations("厨")
        self.assertIn("厨房", similar)
        self.assertIn("厨房阳台", similar)
        self.assertNotIn("客厅", similar)

    def test_get_nonexistent_location(self):
        """测试获取不存在的位置"""
        manager = LocationManager()

        loc = manager.get_location("不存在的位置")
        self.assertIsNone(loc)


class TestLoadLocations(unittest.TestCase):
    """测试从文件加载位置"""

    def test_load_nonexistent_file(self):
        """测试加载不存在的文件"""
        locations = load_locations("/nonexistent/path.yaml")
        self.assertEqual(len(locations), 0)


if __name__ == "__main__":
    unittest.main()
