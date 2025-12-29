"""
Pytest 配置和共享 fixtures
"""

import pytest
import sys
import os

# 添加包路径到 Python 路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


@pytest.fixture
def sample_locations():
    """提供示例位置数据"""
    return {
        "厨房": {"x": 3.0, "y": 2.0, "yaw": 0.0, "description": "厨房"},
        "客厅": {"x": 0.0, "y": 0.0, "yaw": 0.0, "description": "客厅"},
        "卧室": {"x": -2.0, "y": 3.0, "yaw": 1.57, "description": "卧室"},
    }


@pytest.fixture
def mock_navigator(mocker):
    """提供模拟的 Navigator"""
    mock_nav = mocker.MagicMock()
    mock_nav.isTaskComplete.return_value = True
    mock_nav.getResult.return_value = mocker.MagicMock(value=1)
    mock_nav.get_clock.return_value.now.return_value.to_msg.return_value = None
    return mock_nav


@pytest.fixture
def reset_singletons():
    """重置所有单例（测试隔离）"""
    from llm_nav_agent.utils import LocationManager
    from llm_nav_agent.prompts import PromptManager
    from llm_nav_agent.nav_tools import Nav2ToolKit

    # 重置前保存
    yield

    # 测试后重置
    LocationManager._instance = None
    LocationManager._locations = {}
    LocationManager._initialized = False

    PromptManager._instance = None

    Nav2ToolKit._navigator = None
    Nav2ToolKit._initialized = False
    Nav2ToolKit._location_manager = None
