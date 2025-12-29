import os
from glob import glob
from setuptools import setup, find_packages

package_name = "llm_nav_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # 包索引标记（必需）
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package.xml（必需）
        ("share/" + package_name, ["package.xml"]),
        # Launch 文件
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # 配置文件
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Shell 脚本
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="LLM-powered navigation agent for ROS2 with Nav2 integration",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 主节点
            "llm_agent_node = llm_nav_agent.llm_agent_node:main",
            # 测试工具
            "test_agent = llm_nav_agent.test_agent:main",
            # 交互式聊天界面
            "chat_interface = llm_nav_agent.chat_interface:main",
        ],
    },
)
