#!/usr/bin/env python3
"""
LLM Navigation Agent - ROS2 主节点
结合 LangChain + Nav2 实现自然语言控制机器人导航
"""

import os
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# LangChain 相关
from langchain_openai import ChatOpenAI
from langchain_community.chat_models import ChatOllama
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.messages import HumanMessage, AIMessage, BaseMessage

# 本地模块
from .nav_tools import ALL_NAV_TOOLS, Nav2ToolKit
from .prompts import get_prompt_manager, PromptManager
from .utils import LocationManager, get_default_location_manager


class LLMNavAgentNode(Node):
    """
    LLM 导航代理 ROS2 节点

    功能：
    - 接收自然语言命令
    - 使用 LLM 解析用户意图
    - 调用 Nav2 执行导航任务
    - 返回执行结果
    """

    def __init__(self):
        super().__init__("llm_nav_agent")

        # 使用可重入回调组，允许并发处理
        self.callback_group = ReentrantCallbackGroup()

        # ============ 声明参数 ============
        self._declare_parameters()

        # ============ 获取参数 ============
        self.use_local_llm = self.get_parameter("use_local_llm").value
        self.openai_model = self.get_parameter("openai_model").value
        self.openai_temperature = self.get_parameter("openai_temperature").value
        self.ollama_model = self.get_parameter("ollama_model").value
        self.ollama_base_url = self.get_parameter("ollama_base_url").value
        self.max_iterations = self.get_parameter("max_iterations").value
        self.verbose = self.get_parameter("verbose").value
        self.max_history_length = self.get_parameter("max_history_length").value
        self.command_topic = self.get_parameter("command_topic").value
        self.response_topic = self.get_parameter("response_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        # ============ 初始化组件 ============
        self.prompt_manager: PromptManager = get_prompt_manager()
        self.location_manager: LocationManager = get_default_location_manager()

        # 设置位置管理器到工具包
        Nav2ToolKit.set_location_manager(self.location_manager)

        # ============ 初始化 LLM ============
        self._init_llm()

        # ============ 初始化 Agent ============
        self._init_agent()

        # ============ 对话历史 ============
        self.chat_history: List[BaseMessage] = []
        self.history_lock = threading.Lock()

        # ============ ROS2 接口 ============
        # 订阅命令话题
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
            callback_group=self.callback_group,
        )

        # 发布响应话题
        self.response_pub = self.create_publisher(String, self.response_topic, 10)

        # 发布状态话题
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # ============ 等待 Nav2 就绪 ============
        self._publish_status("正在等待 Nav2 就绪...")
        self.get_logger().info("⏳ 等待 Nav2 导航栈就绪...")

        # 在后台线程中等待 Nav2
        self.nav2_ready = False
        self.nav2_init_thread = threading.Thread(target=self._wait_for_nav2)
        self.nav2_init_thread.start()

        self.get_logger().info("🚀 LLM Nav Agent 节点已启动！")
        self.get_logger().info(f"📡 监听命令话题: {self.command_topic}")
        self.get_logger().info(f"📡 发布响应话题: {self.response_topic}")

    def _declare_parameters(self):
        """声明所有 ROS2 参数"""
        # LLM 配置
        self.declare_parameter("use_local_llm", False)
        self.declare_parameter("openai_model", "gpt-4o")
        self.declare_parameter("openai_temperature", 0.0)
        self.declare_parameter("openai_timeout", 30.0)
        self.declare_parameter("ollama_model", "qwen2.5:7b")
        self.declare_parameter("ollama_base_url", "http://localhost:11434")

        # Agent 配置
        self.declare_parameter("max_iterations", 5)
        self.declare_parameter("verbose", True)
        self.declare_parameter("handle_parsing_errors", True)

        # 对话配置
        self.declare_parameter("max_history_length", 20)
        self.declare_parameter("enable_memory", True)

        # 导航配置
        self.declare_parameter("nav_timeout", 120.0)
        self.declare_parameter("locations_file", "config/locations.yaml")

        # 话题配置
        self.declare_parameter("command_topic", "/llm_agent/command")
        self.declare_parameter("response_topic", "/llm_agent/response")
        self.declare_parameter("status_topic", "/llm_agent/status")

    def _init_llm(self):
        """初始化大语言模型"""
        if self.use_local_llm:
            self.get_logger().info(f"🔧 使用本地 Ollama 模型: {self.ollama_model}")
            self.llm = ChatOllama(
                model=self.ollama_model,
                base_url=self.ollama_base_url,
                temperature=0,
            )
        else:
            self.get_logger().info(f"🌐 使用 OpenAI 模型: {self.openai_model}")
            # 检查 API Key
            api_key = os.environ.get("OPENAI_API_KEY")
            if not api_key:
                self.get_logger().warn("⚠️ 未设置 OPENAI_API_KEY 环境变量！")

            self.llm = ChatOpenAI(
                model=self.openai_model,
                temperature=self.openai_temperature,
            )

    def _init_agent(self):
        """初始化 LangChain Agent"""
        # 获取系统提示词
        system_prompt = self.prompt_manager.get_system_prompt()

        # 创建提示模板
        prompt = ChatPromptTemplate.from_messages(
            [
                ("system", system_prompt),
                MessagesPlaceholder(variable_name="chat_history"),
                ("human", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        )

        # 创建 Agent
        agent = create_tool_calling_agent(
            llm=self.llm, tools=ALL_NAV_TOOLS, prompt=prompt
        )

        # 创建 Agent 执行器
        self.agent_executor = AgentExecutor(
            agent=agent,
            tools=ALL_NAV_TOOLS,
            verbose=self.verbose,
            handle_parsing_errors=True,
            max_iterations=self.max_iterations,
            return_intermediate_steps=False,
        )

        self.get_logger().info("✅ LangChain Agent 初始化完成")

    def _wait_for_nav2(self):
        """等待 Nav2 就绪（在后台线程中运行）"""
        try:
            success = Nav2ToolKit.wait_for_nav2(timeout=60.0)
            if success:
                self.nav2_ready = True
                self.get_logger().info("✅ Nav2 导航栈已就绪！")
                self._publish_status("Nav2 就绪，可以接收命令")
            else:
                self.get_logger().error("❌ Nav2 初始化超时")
                self._publish_status("Nav2 初始化失败")
        except Exception as e:
            self.get_logger().error(f"❌ Nav2 初始化错误: {e}")
            self._publish_status(f"Nav2 错误: {e}")

    def _publish_status(self, status: str):
        """发布状态消息"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_response(self, response: str):
        """发布响应消息"""
        msg = String()
        msg.data = response
        self.response_pub.publish(msg)

    def _update_chat_history(self, user_input: str, response: str):
        """更新对话历史"""
        with self.history_lock:
            self.chat_history.append(HumanMessage(content=user_input))
            self.chat_history.append(AIMessage(content=response))

            # 限制历史长度
            if len(self.chat_history) > self.max_history_length:
                self.chat_history = self.chat_history[-self.max_history_length :]

    def _get_chat_history(self) -> List[BaseMessage]:
        """获取对话历史的副本"""
        with self.history_lock:
            return self.chat_history.copy()

    def clear_chat_history(self):
        """清除对话历史"""
        with self.history_lock:
            self.chat_history.clear()
        self.get_logger().info("🗑️ 对话历史已清除")

    def command_callback(self, msg: String):
        """
        处理用户命令的回调函数

        Args:
            msg: 包含用户命令的 ROS2 消息
        """
        user_input = msg.data.strip()

        if not user_input:
            return

        self.get_logger().info(f"📩 收到指令: {user_input}")
        self._publish_status("正在处理命令...")

        # 处理特殊命令
        if user_input.lower() in ["clear", "reset", "清除历史", "重置"]:
            self.clear_chat_history()
            response = "已清除对话历史，我们可以重新开始。"
            self._publish_response(response)
            return

        if user_input.lower() in ["help", "帮助", "?"]:
            response = self.prompt_manager.get_welcome_message()
            self._publish_response(response)
            return

        # 检查 Nav2 是否就绪
        if not self.nav2_ready:
            response = "⏳ 导航系统正在初始化中，请稍候..."
            self._publish_response(response)
            return

        try:
            # 调用 Agent 处理用户输入
            result = self.agent_executor.invoke(
                {"input": user_input, "chat_history": self._get_chat_history()}
            )

            response = result.get("output", "抱歉，我无法处理这个请求。")

            # 更新对话历史
            self._update_chat_history(user_input, response)

            self.get_logger().info(f"🤖 回复: {response}")
            self._publish_status("命令处理完成")

        except Exception as e:
            error_msg = str(e)
            self.get_logger().error(f"❌ 处理命令时出错: {error_msg}")
            response = self.prompt_manager.get_error_prompt(error_msg)
            self._publish_status(f"错误: {error_msg}")

        # 发布响应
        self._publish_response(response)

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("🛑 正在关闭 LLM Nav Agent...")

        # 等待 Nav2 初始化线程完成
        if hasattr(self, "nav2_init_thread") and self.nav2_init_thread.is_alive():
            self.nav2_init_thread.join(timeout=2.0)

        # 关闭 Nav2
        try:
            Nav2ToolKit.shutdown()
        except Exception as e:
            self.get_logger().warn(f"关闭 Nav2 时出错: {e}")

        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    node = LLMNavAgentNode()

    # 使用多线程执行器以支持并发回调
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("⌨️ 收到键盘中断信号")
    except Exception as e:
        node.get_logger().error(f"❌ 发生错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
