# LLM Nav Agent

基于大语言模型（LLM）的 ROS2 导航代理，使用自然语言控制机器人导航。

## 概述

LLM Nav Agent 将 LangChain 与 ROS2 Nav2 结合，允许用户通过自然语言指令控制机器人移动。

### 主要特性

- **自然语言理解**：支持中文自然语言指令
- **多模型支持**：支持 OpenAI API 和本地 Ollama 模型
- **Nav2 集成**：无缝对接 ROS2 Nav2 导航栈
- **预设位置**：支持命名位置的快速导航
- **对话记忆**：保持上下文的多轮对话

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Nav2 导航栈

## 安装

### 1. 安装依赖

```bash
# ROS2 依赖
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-simple-commander

# Python 依赖
pip install langchain langchain-openai langchain-community pyyaml

