#!/bin/bash
# ============================================
# 运行测试脚本
# ============================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$(dirname "$PKG_DIR")")"

echo "============================================"
echo "运行 LLM Nav Agent 测试"
echo "============================================"

cd "$WS_DIR"

# 确保已构建
if [ ! -d "install/llm_nav_agent" ]; then
    echo "包未构建，正在构建..."
    colcon build --packages-select llm_nav_agent
fi

# Source 环境
source install/setup.bash

# 运行测试
echo ""
echo "运行单元测试..."
colcon test --packages-select llm_nav_agent

echo ""
echo "测试结果:"
colcon test-result --verbose

echo ""
echo "============================================"
echo "测试完成"
echo "============================================"
