#!/usr/bin/env python3
"""
交互式聊天界面
提供更友好的命令行交互体验
"""

import sys
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


# ANSI 颜色代码
class Colors:
    HEADER = "\033[95m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class ChatInterface(Node):
    """交互式聊天界面节点"""

    def __init__(self):
        super().__init__("chat_interface")

        # ROS2 接口
        self.command_pub = self.create_publisher(String, "/llm_agent/command", 10)
        self.response_sub = self.create_subscription(
            String, "/llm_agent/response", self.response_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, "/llm_agent/status", self.status_callback, 10
        )

        # 状态变量
        self.current_response: Optional[str] = None
        self.current_status: Optional[str] = None
        self.response_event = threading.Event()

        self.get_logger().info("聊天界面已启动")

    def response_callback(self, msg: String):
        """响应回调"""
        self.current_response = msg.data
        self.response_event.set()

    def status_callback(self, msg: String):
        """状态回调"""
        self.current_status = msg.data
        # 在同一行显示状态更新
        sys.stdout.write(f"\r{Colors.CYAN}[状态] {msg.data}{Colors.ENDC}          ")
        sys.stdout.flush()

    def send_and_wait(self, command: str, timeout: float = 60.0) -> Optional[str]:
        """
        发送命令并等待响应

        Args:
            command: 用户命令
            timeout: 超时时间（秒）

        Returns:
            Agent 的响应，超时返回 None
        """
        self.response_event.clear()
        self.current_response = None

        # 发送命令
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        # 等待响应
        if self.response_event.wait(timeout=timeout):
            return self.current_response
        else:
            return None

    def print_banner(self):
        """打印欢迎横幅"""
        banner = f"""
{Colors.HEADER}╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║   {Colors.BOLD}🤖 LLM Nav Agent 交互式聊天界面{Colors.HEADER}                            ║
║                                                                  ║
║   通过自然语言控制机器人导航                                     ║
║                                                                  ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║   {Colors.GREEN}命令示例:{Colors.HEADER}                                                   ║
║     • "去厨房"          - 导航到预设位置                         ║
║     • "去坐标 3, 2"     - 导航到指定坐标                         ║
║     • "停下来"          - 取消当前导航                           ║
║     • "你在哪"          - 查询当前位置                           ║
║     • "能去哪些地方"    - 列出可用位置                           ║
║                                                                  ║
║   {Colors.YELLOW}特殊命令:{Colors.HEADER}                                                   ║
║     • help / 帮助       - 显示帮助信息                           ║
║     • clear / 清除      - 清除对话历史                           ║
║     • status / 状态     - 显示系统状态                           ║
║     • quit / q          - 退出程序                               ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝{Colors.ENDC}
"""
        print(banner)

    def print_help(self):
        """打印帮助信息"""
        help_text = f"""
{Colors.GREEN}=== 帮助信息 ==={Colors.ENDC}

{Colors.BOLD}导航命令:{Colors.ENDC}
  • 去[地点名]        例如: "去厨房", "去客厅"
  • 去坐标 x, y       例如: "去坐标 3, 2", "移动到 (5, 3)"
  • 停/取消/别动了    取消当前导航任务

{Colors.BOLD}查询命令:{Colors.ENDC}
  • 你在哪/当前位置   查询机器人位置
  • 能去哪/有哪些地方 列出所有可用位置

{Colors.BOLD}系统命令:{Colors.ENDC}
  • help    显示此帮助
  • clear   清除对话历史
  • status  显示系统状态
  • quit    退出程序
"""
        print(help_text)


def run_chat_loop(node: ChatInterface):
    """运行聊天循环"""
    node.print_banner()

    print(f"\n{Colors.GREEN}系统已就绪，请输入您的指令...{Colors.ENDC}\n")

    while rclpy.ok():
        try:
            # 获取用户输入
            user_input = input(f"{Colors.YELLOW}👤 你: {Colors.ENDC}").strip()

            if not user_input:
                continue

            # 处理本地命令
            lower_input = user_input.lower()

            if lower_input in ["quit", "exit", "q", "退出"]:
                print(f"\n{Colors.GREEN}👋 再见！感谢使用 LLM Nav Agent{Colors.ENDC}\n")
                break

            if lower_input in ["help", "帮助", "?"]:
                node.print_help()
                continue

            if lower_input in ["clear", "清除", "清除历史"]:
                # 发送清除命令给 Agent
                node.send_and_wait("clear", timeout=5.0)
                print(f"{Colors.GREEN}✓ 对话历史已清除{Colors.ENDC}\n")
                continue

            if lower_input in ["status", "状态"]:
                status = node.current_status or "未知"
                print(f"{Colors.CYAN}📊 当前状态: {status}{Colors.ENDC}\n")
                continue

            # 发送命令到 Agent
            print(f"{Colors.CYAN}⏳ 正在处理...{Colors.ENDC}")

            response = node.send_and_wait(user_input, timeout=120.0)

            # 清除状态行并打印响应
            sys.stdout.write("\r" + " " * 50 + "\r")

            if response:
                print(f"\n{Colors.GREEN}🤖 小智: {Colors.ENDC}{response}\n")
            else:
                print(f"\n{Colors.RED}⏰ 响应超时，请重试{Colors.ENDC}\n")

        except EOFError:
            break
        except KeyboardInterrupt:
            print(f"\n\n{Colors.YELLOW}收到中断信号...{Colors.ENDC}")
            break


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    node = ChatInterface()

    # 在后台线程运行 ROS2 spin
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 等待一下让节点完全初始化
    time.sleep(0.5)

    try:
        run_chat_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
