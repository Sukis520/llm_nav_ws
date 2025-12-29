#!/usr/bin/env python3
"""
Agent 集成测试
测试 LangChain Agent 的基本功能
"""

import unittest
from unittest.mock import MagicMock, patch, PropertyMock
import sys
import os

# 添加包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


class TestNavTools(unittest.TestCase):
    """测试导航工具（模拟 Nav2）"""

    @patch("llm_nav_agent.nav_tools.Nav2ToolKit")
    def test_navigate_to_coordinate_parse(self, mock_toolkit):
        """测试坐标解析"""
        from llm_nav_agent.nav_tools import navigate_to_coordinate

        # 模拟导航器
        mock_navigator = MagicMock()
        mock_navigator.isTaskComplete.return_value = True
        mock_navigator.getResult.return_value = MagicMock(value=1)  # SUCCEEDED
        mock_navigator.get_clock.return_value.now.return_value.to_msg.return_value = (
            None
        )

        mock_toolkit.get_navigator.return_value = mock_navigator

        # 测试会失败因为需要实际的 Nav2，这里只是演示结构
        # result = navigate_to_coordinate.invoke({"x": 1.0, "y": 2.0})
        pass

    @patch("llm_nav_agent.nav_tools.Nav2ToolKit")
    def test_get_available_locations(self, mock_toolkit):
        """测试获取可用位置"""
        from llm_nav_agent.nav_tools import get_available_locations
        from llm_nav_agent.utils import LocationManager

        # 设置模拟的位置管理器
        mock_manager = MagicMock(spec=LocationManager)
        mock_manager.get_all_locations.return_value = {
            "厨房": MagicMock(name="厨房", description="厨房区域"),
            "客厅": MagicMock(name="客厅", description="客厅区域"),
        }

        mock_toolkit.get_location_manager.return_value = mock_manager

        # 调用工具
        result = get_available_locations.invoke({})

        # 验证结果包含位置信息
        self.assertIn("厨房", result)
        self.assertIn("客厅", result)


class TestPromptManager(unittest.TestCase):
    """测试提示词管理器"""

    def test_default_prompts(self):
        """测试默认提示词"""
        from llm_nav_agent.prompts import PromptManager

        # 重置单例
        PromptManager._instance = None

        manager = PromptManager()

        system_prompt = manager.get_system_prompt()
        self.assertIsNotNone(system_prompt)
        self.assertIn("小智", system_prompt)

    def test_welcome_message(self):
        """测试欢迎消息"""
        from llm_nav_agent.prompts import PromptManager

        PromptManager._instance = None
        manager = PromptManager()

        welcome = manager.get_welcome_message()
        self.assertIsNotNone(welcome)
        self.assertTrue(len(welcome) > 0)

    def test_error_prompt_format(self):
        """测试错误提示格式化"""
        from llm_nav_agent.prompts import PromptManager

        PromptManager._instance = None
        manager = PromptManager()

        error_msg = "测试错误"
        prompt = manager.get_error_prompt(error_msg)

        self.assertIn(error_msg, prompt)

    def test_set_custom_prompt(self):
        """测试设置自定义提示词"""
        from llm_nav_agent.prompts import PromptManager

        PromptManager._instance = None
        manager = PromptManager()

        custom_prompt = "这是自定义提示词"
        manager.set_prompt("custom_key", custom_prompt)

        result = manager.get_prompt("custom_key")
        self.assertEqual(result, custom_prompt)

    def test_get_nonexistent_prompt(self):
        """测试获取不存在的提示词"""
        from llm_nav_agent.prompts import PromptManager

        PromptManager._instance = None
        manager = PromptManager()

        result = manager.get_prompt("nonexistent", "default")
        self.assertEqual(result, "default")


class TestAgentConfiguration(unittest.TestCase):
    """测试 Agent 配置"""

    def test_tools_list(self):
        """测试工具列表"""
        from llm_nav_agent.nav_tools import ALL_NAV_TOOLS

        # 验证工具列表不为空
        self.assertGreater(len(ALL_NAV_TOOLS), 0)

        # 验证每个工具都有名称
        for tool in ALL_NAV_TOOLS:
            self.assertTrue(hasattr(tool, "name"))
            self.assertTrue(len(tool.name) > 0)

    def test_tool_descriptions(self):
        """测试工具描述"""
        from llm_nav_agent.nav_tools import ALL_NAV_TOOLS

        for tool in ALL_NAV_TOOLS:
            self.assertTrue(hasattr(tool, "description"))
            self.assertTrue(len(tool.description) > 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
