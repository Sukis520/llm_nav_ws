"""
仿真环境启动文件
启动 Gazebo + Nav2 + LLM Agent
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============ 包路径 ============
    llm_pkg_dir = get_package_share_directory("llm_nav_agent")

    # Nav2 相关包（需要安装）
    try:
        nav2_bringup_dir = get_package_share_directory("nav2_bringup")
        turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
        has_turtlebot = True
    except:
        has_turtlebot = False
        nav2_bringup_dir = ""
        turtlebot3_gazebo_dir = ""

    # ============ 参数 ============
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    # ============ 创建启动描述 ============
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time)

    if has_turtlebot:
        # ========== TurtleBot3 Gazebo 仿真 ==========
        # 设置 TurtleBot3 模型
        os.environ["TURTLEBOT3_MODEL"] = "waffle"

        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    turtlebot3_gazebo_dir, "launch", "turtlebot3_world.launch.py"
                )
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
        ld.add_action(gazebo_launch)

        # ========== Nav2 导航栈 ==========
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        )
        # 延迟启动 Nav2，等待 Gazebo 完全加载
        delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch])
        ld.add_action(delayed_nav2)

    # ========== LLM Agent 节点 ==========
    llm_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(llm_pkg_dir, "launch", "llm_agent.launch.py")
        ),
    )
    # 延迟启动 LLM Agent，等待 Nav2 就绪
    delayed_llm_agent = TimerAction(
        period=10.0 if has_turtlebot else 0.0, actions=[llm_agent_launch]
    )
    ld.add_action(delayed_llm_agent)

    return ld
