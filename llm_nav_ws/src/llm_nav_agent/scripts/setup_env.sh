#!/bin/bash
# ============================================
# LLM Nav Agent 环境配置脚本
# ============================================

set -e

echo "============================================"
echo "LLM Nav Agent 环境配置"
echo "============================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查 ROS2 环境
check_ros2() {
    echo -e "\n${YELLOW}[1/5] 检查 ROS2 环境...${NC}"
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}错误: 未检测到 ROS2 环境${NC}"
        echo "请先 source ROS2 环境: source /opt/ros/humble/setup.bash"
        exit 1
    else
        echo -e "${GREEN}✓ ROS2 $ROS_DISTRO 已就绪${NC}"
    fi
}

# 安装 ROS2 依赖
install_ros2_deps() {
    echo -e "\n${YELLOW}[2/5] 安装 ROS2 依赖...${NC}"
    sudo apt update
    sudo apt install -y \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-simple-commander \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs
    echo -e "${GREEN}✓ ROS2 依赖安装完成${NC}"
}

# 安装 Python 依赖
install_python_deps() {
    echo -e "\n${YELLOW}[3/5] 安装 Python 依赖...${NC}"
    pip3 install --upgrade pip
    pip3 install \
        langchain \
        langchain-openai \
        langchain-community \
        pyyaml \
        pydantic
    echo -e "${GREEN}✓ Python 依赖安装完成${NC}"
}

# 配置 OpenAI API Key
setup_api_key() {
    echo -e "\n${YELLOW}[4/5] 配置 API Key...${NC}"
    
    if [ -z "$OPENAI_API_KEY" ]; then
        echo "未检测到 OPENAI_API_KEY 环境变量"
        read -p "是否现在配置 OpenAI API Key? (y/n): " choice
        
        if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
            read -p "请输入您的 OpenAI API Key: " api_key
            
            if [ -n "$api_key" ]; then
                echo "export OPENAI_API_KEY=\"$api_key\"" >> ~/.bashrc
                export OPENAI_API_KEY="$api_key"
                echo -e "${GREEN}✓ API Key 已配置${NC}"
            fi
        else
            echo -e "${YELLOW}跳过 API Key 配置（您可以使用本地 Ollama 模型）${NC}"
        fi
    else
        echo -e "${GREEN}✓ OPENAI_API_KEY 已存在${NC}"
    fi
}

# 安装 Ollama（可选）
install_ollama() {
    echo -e "\n${YELLOW}[5/5] 安装 Ollama（可选）...${NC}"
    
    if command -v ollama &> /dev/null; then
        echo -e "${GREEN}✓ Ollama 已安装${NC}"
    else
        read -p "是否安装 Ollama 本地模型? (y/n): " choice
        
        if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
            echo "正在安装 Ollama..."
            curl -fsSL https://ollama.com/install.sh | sh
            
            echo "正在下载 qwen2.5:7b 模型..."
            ollama pull qwen2.5:7b
            
            echo -e "${GREEN}✓ Ollama 安装完成${NC}"
        else
            echo -e "${YELLOW}跳过 Ollama 安装${NC}"
        fi
    fi
}

# 构建包
build_package() {
    echo -e "\n${YELLOW}构建 ROS2 包...${NC}"
    
    # 获取脚本所在目录
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PKG_DIR="$(dirname "$SCRIPT_DIR")"
    WS_DIR="$(dirname "$(dirname "$PKG_DIR")")"
    
    cd "$WS_DIR"
    
    colcon build --packages-select llm_nav_agent
    
    echo -e "${GREEN}✓ 构建完成${NC}"
    echo ""
    echo "请运行以下命令使环境生效:"
    echo "  source $WS_DIR/install/setup.bash"
}

# 主函数
main() {
    check_ros2
    
    read -p "是否安装所有依赖? (y/n): " install_choice
    
    if [ "$install_choice" = "y" ] || [ "$install_choice" = "Y" ]; then
        install_ros2_deps
        install_python_deps
        setup_api_key
        install_ollama
    fi
    
    read -p "是否构建包? (y/n): " build_choice
    
    if [ "$build_choice" = "y" ] || [ "$build_choice" = "Y" ]; then
        build_package
    fi
    
    echo ""
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}配置完成！${NC}"
    echo -e "${GREEN}============================================${NC}"
    echo ""
    echo "启动命令:"
    echo "  # 使用 OpenAI"
    echo "  ros2 launch llm_nav_agent llm_agent.launch.py"
    echo ""
    echo "  # 使用本地 Ollama"
    echo "  ros2 launch llm_nav_agent llm_agent.launch.py use_local_llm:=true"
    echo ""
}

main
