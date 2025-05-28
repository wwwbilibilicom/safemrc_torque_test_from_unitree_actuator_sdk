#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 安装Python依赖
pip3 install pandas matplotlib numpy

# 设置Python脚本的执行权限（使用绝对路径）
chmod +x "${SCRIPT_DIR}/plot_data.py"

echo "Dependencies installed and permissions set successfully!" 