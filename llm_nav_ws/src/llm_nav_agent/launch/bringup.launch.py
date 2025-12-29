"""
完整系统启动文件
包含 Nav2 + LLM Agent + 可选的 Gazebo 仿真
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # ============ 包路径 ============
    llm_pkg_dir = get_package_share_directory("llm_nav_agent")

    try:
        nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    except:
        nav2_bringup_dir = ""

    # ============ 参数配置 ============
    default_params_file = os.path.join(llm_pkg_dir, "config", "llm_params.yaml")

    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file (leave empty for SLAM)",
    )

    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=(
            os.path.join(nav2_bringup_dir, "params", "nav2_params.yaml")
            if nav2_bringup_dir
            else ""
        ),
        description="Full path to Nav2 parameters file",
    )

    declare_llm_params = DeclareLaunchArgument(
        "llm_params_file",
        default_value=default_params_file,
        description="Full path to LLM agent parameters file",
    )

    declare_use_local_llm = DeclareLaunchArgument(
        "use_local_llm", default_value="false", description="Use local Ollama model"
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically start Nav2 stack"
    )

    # ============ 获取参数值 ============
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    llm_params_file = LaunchConfiguration("llm_params_file")
    use_local_llm = LaunchConfiguration("use_local_llm")
    autostart = LaunchConfiguration("autostart")

    # ============ 设置全局参数 ============
    set_use_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    # ============ Nav2 启动（如果包存在）============
    nav2_bringup = []
    if nav2_bringup_dir:
        nav2_bringup = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml_file,
                    "params_file": nav2_params_file,
                    "autostart": autostart,
                }.items(),
            )
        ]

    # ============ LLM Agent 启动 ============
    llm_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(llm_pkg_dir, "launch", "llm_agent.launch.py")
        ),
        launch_arguments={
            "params_file": llm_params_file,
            "use_local_llm": use_local_llm,
        }.items(),
    )

    # 延迟启动 LLM Agent（等待 Nav2 就绪）
    delayed_llm_agent = TimerAction(period=5.0, actions=[llm_agent_launch])

    # ============ 组装启动描述 ============
    return LaunchDescription(
        [
            # 参数声明
            declare_use_sim_time,
            declare_map_yaml,
            declare_nav2_params,
            declare_llm_params,
            declare_use_local_llm,
            declare_autostart,
            # 全局参数
            set_use_sim_time,
            # Nav2
            *nav2_bringup,
            # LLM Agent
            delayed_llm_agent,
        ]
    )
