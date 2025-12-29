#!/usr/bin/env python3
"""
命令行测试脚本
用于快速测试 LLM Agent 节点
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class TestAgentClient(Node):
    """测试客户端节点"""

    def __init__(self):
        super().__init__("test_agent_client")

        # 发布命令
        self.command_pub = self.create_publisher(String, "/llm_agent/command", 10)

        # 订阅响应
        self.response_sub = self.create_subscription(
            String, "/llm_agent/response", self.response_callback, 10
        )

        # 订阅状态
        self.status_sub = self.create_subscription(
            String, "/llm_agent/status", self.status_callback, 10
        )

        self.waiting_for_response = False
        self.get_logger().info("✅ 测试客户端已启动")

    def response_callback(self, msg: String):
        """处理 Agent 响应"""
        print(f"\n🤖 \033[92mAgent 回复:\033[0m {msg.data}\n")
        self.waiting_for_response = False

    def status_callback(self, msg: String):
        """处理状态更新"""
        print(f"📊 \033[94m状态:\033[0m {msg.data}")

    def send_command(self, command: str):
        """发送命令"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.waiting_for_response = True
        self.get_logger().info(f"📤 已发送: {command}")


def print_help():
    """打印帮助信息"""
    print(
        """
╔══════════════════════════════════════════════════════════════╗
║           LLM Nav Agent 测试工具                              ║
╠══════════════════════════════════════════════════════════════╣
║ 命令示例:                                                     ║
║   • 去厨房                                                    ║
║   • 移动到坐标 3, 2                                           ║
║   • 你能去哪些地方？                                          ║
║   • 停下来                                                    ║
║   • 你在哪里？                                                ║
║                                                              ║
║ 特殊命令:                                                     ║
║   • help    - 显示此帮助信息                                  ║
║   • clear   - 清除对话历史                                    ║
║   • quit/q  - 退出程序                                        ║
╚══════════════════════════════════════════════════════════════╝
"""
    )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    client = TestAgentClient()

    # 创建执行器并在后台线程运行
    executor = MultiThreadedExecutor()
    executor.add_node(client)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print_help()
    print("输入命令与机器人对话，输入 'quit' 或 'q' 退出\n")

    try:
        while rclpy.ok():
            try:
                user_input = input("👤 \033[93m你:\033[0m ").strip()
            except EOFError:
                break

            if not user_input:
                continue

            # 处理本地命令
            if user_input.lower() in ["quit", "exit", "q"]:
                print("👋 再见！")
                break

            if user_input.lower() == "help":
                print_help()
                continue

            # 发送命令到 Agent
            client.send_command(user_input)

            # 等待响应（最多等待 30 秒）
            import time

            timeout = 30.0
            start = time.time()
            while client.waiting_for_response and (time.time() - start) < timeout:
                time.sleep(0.1)

            if client.waiting_for_response:
                print("⏰ 等待响应超时\n")
                client.waiting_for_response = False

    except KeyboardInterrupt:
        print("\n👋 收到中断信号，退出...")

    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
