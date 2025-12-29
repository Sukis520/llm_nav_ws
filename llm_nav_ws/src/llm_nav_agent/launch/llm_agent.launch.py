"""
LLM Agent 主启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory("llm_nav_agent")

    # 配置文件路径
    default_params_file = os.path.join(pkg_dir, "config", "llm_params.yaml")

    # ============ 声明启动参数 ============
    declare_use_local_llm = DeclareLaunchArgument(
        "use_local_llm",
        default_value="false",
        description="Use local Ollama instead of OpenAI API",
    )

    declare_openai_model = DeclareLaunchArgument(
        "openai_model",
        default_value="gpt-4o",
        description="OpenAI model name (gpt-4o, gpt-4-turbo, gpt-3.5-turbo)",
    )

    declare_ollama_model = DeclareLaunchArgument(
        "ollama_model", default_value="qwen2.5:7b", description="Ollama model name"
    )

    declare_ollama_url = DeclareLaunchArgument(
        "ollama_base_url",
        default_value="http://localhost:11434",
        description="Ollama server URL",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file",
    )

    declare_verbose = DeclareLaunchArgument(
        "verbose", default_value="true", description="Enable verbose output"
    )

    # ============ 日志信息 ============
    log_info = LogInfo(
        msg=[
            "Launching LLM Nav Agent with params: ",
            LaunchConfiguration("params_file"),
        ]
    )

    # ============ LLM Agent 节点 ============
    llm_agent_node = Node(
        package="llm_nav_agent",
        executable="llm_agent_node",
        name="llm_nav_agent",
        output="screen",
        emulate_tty=True,  # 保持颜色输出
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_local_llm": LaunchConfiguration("use_local_llm"),
                "openai_model": LaunchConfiguration("openai_model"),
                "ollama_model": LaunchConfiguration("ollama_model"),
                "ollama_base_url": LaunchConfiguration("ollama_base_url"),
                "verbose": LaunchConfiguration("verbose"),
            },
        ],
        # 重映射（如果需要）
        remappings=[
            # ('/llm_agent/command', '/voice_command'),
        ],
    )

    return LaunchDescription(
        [
            # 参数声明
            declare_use_local_llm,
            declare_openai_model,
            declare_ollama_model,
            declare_ollama_url,
            declare_params_file,
            declare_verbose,
            # 日志
            log_info,
            # 节点
            llm_agent_node,
        ]
    )
