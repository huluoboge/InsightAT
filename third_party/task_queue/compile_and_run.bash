#!/bin/bash

echo "=== 开始测试异步队列计算模型 ==="

# 检查必要的文件是否存在
required_files=("task_queue.hpp" "task_queue_demo.cpp" "task_queue.py" "task_queue_demo.py")
for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        echo "错误: 缺少文件 $file"
        exit 1
    fi
done

echo "1. 测试C++实现..."
echo "编译task_queue_demo.cpp..."

# 使用g++编译，需要C++11和pthread支持
g++ -std=c++11 -pthread task_queue_demo.cpp -o task_queue_demo 2>&1
if [ $? -ne 0 ]; then
    echo "C++编译失败!"
    exit 1
fi

echo "运行C++演示程序..."
./task_queue_demo
cpp_exit_code=$?

if [ $cpp_exit_code -eq 0 ]; then
    echo "C++演示程序运行成功 (退出码: $cpp_exit_code)"
else
    echo "C++演示程序运行失败 (退出码: $cpp_exit_code)"
fi

echo ""
echo "2. 测试Python实现..."
echo "运行task_queue_demo.py..."

# 检查Python3是否可用
if command -v python3 &> /dev/null; then
    python3 task_queue_demo.py
    py_exit_code=$?
elif command -v python &> /dev/null; then
    python task_queue_demo.py
    py_exit_code=$?
else
    echo "错误: 未找到Python解释器"
    exit 1
fi

if [ $py_exit_code -eq 0 ]; then
    echo "Python演示程序运行成功 (退出码: $py_exit_code)"
else
    echo "Python演示程序运行失败 (退出码: $py_exit_code)"
fi

echo ""
echo "=== 测试完成 ==="
echo "C++测试: $( [ $cpp_exit_code -eq 0 ] && echo '通过' || echo '失败' )"
echo "Python测试: $( [ $py_exit_code -eq 0 ] && echo '通过' || echo '失败' )"

# 清理临时文件
if [ -f "task_queue_demo" ]; then
    rm task_queue_demo
fi

if [ $cpp_exit_code -eq 0 ] && [ $py_exit_code -eq 0 ]; then
    exit 0
else
    exit 1
fi