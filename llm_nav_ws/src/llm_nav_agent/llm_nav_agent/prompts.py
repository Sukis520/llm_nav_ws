"""
提示词管理模块
管理 LLM Agent 使用的各种提示词模板
"""

import os
import yaml
from typing import Dict, Optional
from ament_index_python.packages import get_package_share_directory


# 默认系统提示词
DEFAULT_SYSTEM_PROMPT = """你是一个智能服务机器人助手，名叫"小智"。

## 你的能力
你可以帮助用户控制机器人在室内环境中移动：
1. 导航到指定坐标位置（使用 navigate_to_coordinate）
2. 导航到预设的命名地点（使用 navigate_to_named_location）
3. 取消正在进行的导航（使用 cancel_navigation）
4. 查询当前位置（使用 get_current_position）
5. 列出可用的预设位置（使用 get_available_locations）
6. 设置初始位姿（使用 set_initial_pose）
7. 多点巡航（使用 navigate_through_poses）

## 使用规则
- 当用户提到具体地点名称（如"厨房"、"客厅"）时，优先使用 navigate_to_named_location
- 当用户给出具体坐标（如"坐标3,2"、"位置(5, 3)"）时，使用 navigate_to_coordinate
- 如果用户说"停"、"取消"、"别动了"、"停下来"，使用 cancel_navigation
- 如果用户问"你在哪"、"当前位置"，使用 get_current_position
- 如果用户问"能去哪"、"有哪些地方"，使用 get_available_locations
- 如果用户意图不明确，请礼貌地询问确认

## 回复风格
- 使用友好、简洁的中文回复
- 执行操作后告知结果
- 遇到问题时提供有帮助的建议
- 使用适当的表情符号增加亲和力

## 注意事项
- 确保导航坐标在合理范围内
- 如果导航失败，建议用户检查是否有障碍物
- 在执行危险操作前确认用户意图
"""

DEFAULT_WELCOME_MESSAGE = """你好！我是小智，你的智能导航助手。🤖

我可以帮你：
• 去指定地点（如"去厨房"）
• 去指定坐标（如"去坐标3,2"）
• 查看可去的地方（如"你能去哪"）
• 停止移动（如"停下来"）

请告诉我你需要什么帮助？"""

DEFAULT_ERROR_PROMPT = (
    "抱歉，我遇到了一些问题：{error}。请稍后再试或换一种方式表达您的需求。"
)

DEFAULT_UNCLEAR_INTENT_PROMPT = "抱歉，我不太理解您的意思。您是想让我去某个地方吗？可以说'去厨房'或'去坐标3,2'这样的指令。"


class PromptManager:
    """
    提示词管理器
    负责加载和管理各种提示词模板
    """

    _instance: Optional["PromptManager"] = None

    def __new__(cls):
        """单例模式"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._prompts: Dict[str, str] = {
            "system_prompt": DEFAULT_SYSTEM_PROMPT,
            "welcome_message": DEFAULT_WELCOME_MESSAGE,
            "error_prompt": DEFAULT_ERROR_PROMPT,
            "unclear_intent_prompt": DEFAULT_UNCLEAR_INTENT_PROMPT,
        }
        self._initialized = True

    def load_from_yaml(self, yaml_path: str) -> bool:
        """
        从 YAML 文件加载提示词配置

        Args:
            yaml_path: YAML 文件路径

        Returns:
            是否加载成功
        """
        if not os.path.exists(yaml_path):
            print(f"提示词配置文件不存在: {yaml_path}")
            return False

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)

            if data:
                for key in self._prompts.keys():
                    if key in data:
                        self._prompts[key] = data[key]

            return True

        except Exception as e:
            print(f"加载提示词配置出错: {e}")
            return False

    def load_from_package(
        self,
        package_name: str = "llm_nav_agent",
        config_file: str = "config/prompts.yaml",
    ) -> bool:
        """
        从 ROS2 包加载提示词配置

        Args:
            package_name: 包名
            config_file: 配置文件相对路径

        Returns:
            是否加载成功
        """
        try:
            pkg_dir = get_package_share_directory(package_name)
            yaml_path = os.path.join(pkg_dir, config_file)
            return self.load_from_yaml(yaml_path)
        except Exception as e:
            print(f"从包加载提示词失败: {e}")
            return False

    def get_system_prompt(self) -> str:
        """获取系统提示词"""
        return self._prompts["system_prompt"]

    def get_welcome_message(self) -> str:
        """获取欢迎消息"""
        return self._prompts["welcome_message"]

    def get_error_prompt(self, error: str = "") -> str:
        """获取错误提示，支持格式化"""
        return self._prompts["error_prompt"].format(error=error)

    def get_unclear_intent_prompt(self) -> str:
        """获取意图不明确时的提示"""
        return self._prompts["unclear_intent_prompt"]

    def set_prompt(self, key: str, value: str) -> None:
        """设置指定提示词"""
        self._prompts[key] = value

    def get_prompt(self, key: str, default: str = "") -> str:
        """获取指定提示词"""
        return self._prompts.get(key, default)

    def get_all_prompts(self) -> Dict[str, str]:
        """获取所有提示词"""
        return self._prompts.copy()


def get_prompt_manager() -> PromptManager:
    """获取提示词管理器单例"""
    manager = PromptManager()
    # 尝试从包加载配置
    manager.load_from_package()
    return manager
