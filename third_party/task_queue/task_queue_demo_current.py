#!/usr/bin/env python3
"""
演示StageCurrent的使用 - 在当前线程中执行任务
类似于C++的main3()示例
"""

import task_queue as tq
import threading
import time

def demo1_basic_current_thread():
    """演示1: 基本的CurrentThread使用"""
    print("=== 演示1: 基本的CurrentThread使用 ===")
    
    N = 10
    data = list(range(N))
    
    # 创建一个在当前线程执行的stage
    current_stage = tq.StageCurrent("CurrentStage", 1, 8, 
        lambda i: print(f"当前线程处理: {i} -> {data[i]}")
    )
    
    current_stage.setTaskCount(N)
    
    # 在后台线程中push任务
    def producer():
        print("后台线程开始push任务...")
        for i in range(N):
            current_stage.push(i)
        print("后台线程完成push")
    
    producer_thread = threading.Thread(target=producer)
    producer_thread.start()
    
    # 在主线程中执行任务
    print("主线程开始执行任务...")
    current_stage.run()  # 阻塞直到所有任务完成
    print("主线程完成所有任务\n")


def demo2_mixed_pipeline():
    """演示2: 混合流水线 - ThreadPool + CurrentThread"""
    print("=== 演示2: 混合流水线（多线程处理 + 当前线程渲染）===")
    
    N = 10
    data = list(range(N))
    
    # Stage 1: I/O阶段（多线程）
    io_stage = tq.Stage("IO", 2, 4, lambda i: (
        time.sleep(0.01),  # 模拟I/O延迟
        data.__setitem__(i, data[i] * 2),
        print(f"[IO线程] 加载数据: {i}")
    ))
    
    # Stage 2: CPU处理阶段（多线程）
    process_stage = tq.Stage("Process", 2, 4, lambda i: (
        data.__setitem__(i, data[i] + 10),
        print(f"[CPU线程] 处理数据: {i} -> {data[i]}")
    ))
    
    # Stage 3: GPU/渲染阶段（当前线程 - 模拟CUDA必须在主线程）
    render_stage = tq.StageCurrent("Render", 1, 8, lambda i: (
        print(f"[主线程-GPU] 渲染: {i} -> {data[i] * 100}")
    ))
    
    # 链接流水线
    tq.chain(io_stage, process_stage)
    tq.chain(process_stage, render_stage)
    
    io_stage.setTaskCount(N)
    process_stage.setTaskCount(N)
    render_stage.setTaskCount(N)
    
    # 在后台线程中push任务
    def producer():
        print("生产者线程开始push任务...")
        for i in range(N):
            io_stage.push(i)
        print("生产者线程完成push")
    
    producer_thread = threading.Thread(target=producer)
    producer_thread.start()
    
    # 主线程可以做其他工作
    print("主线程可以同时处理其他任务...")
    for i in range(3):
        print(f"主线程正在处理其他工作 {i+1}/3")
        time.sleep(0.05)
    print("主线程完成其他工作")
    
    # 在主线程中运行渲染阶段（必须在主线程）
    print("\n主线程开始执行渲染任务...")
    render_stage.run()  # 阻塞直到所有渲染任务完成
    
    # 等待其他阶段完成
    io_stage.wait()
    process_stage.wait()
    
    print("所有阶段完成！\n")


def demo3_cuda_simulation():
    """演示3: 模拟CUDA工作流 - CPU预处理 + GPU计算（主线程）"""
    print("=== 演示3: 模拟CUDA工作流 ===")
    
    N = 8
    data = list(range(N))
    results = [0] * N
    
    # CPU预处理阶段（多线程）
    cpu_stage = tq.Stage("CPU-Preprocess", 2, 4, lambda i: (
        time.sleep(0.02),  # 模拟CPU计算
        data.__setitem__(i, data[i] ** 2),
        print(f"[CPU] 预处理: {i} -> {data[i]}")
    ))
    
    # GPU计算阶段（必须在主线程，因为CUDA上下文绑定）
    gpu_stage = tq.StageCurrent("GPU-Compute", 1, 8, lambda i: (
        time.sleep(0.01),  # 模拟GPU计算
        results.__setitem__(i, data[i] * 1000),
        print(f"[GPU-主线程] CUDA计算: {i} -> {results[i]}")
    ))
    
    # CPU后处理阶段（多线程）
    postprocess_stage = tq.Stage("CPU-Postprocess", 2, 4, lambda i: (
        print(f"[CPU] 后处理: {i} -> 最终结果: {results[i]}")
    ))
    
    # 链接流水线
    tq.chain(cpu_stage, gpu_stage)
    tq.chain(gpu_stage, postprocess_stage)
    
    cpu_stage.setTaskCount(N)
    gpu_stage.setTaskCount(N)
    postprocess_stage.setTaskCount(N)
    
    # 在后台线程中push任务
    def producer():
        for i in range(N):
            cpu_stage.push(i)
    
    producer_thread = threading.Thread(target=producer)
    producer_thread.start()
    
    print("主线程准备执行GPU任务...")
    print("（在真实CUDA程序中，这里会初始化CUDA上下文）\n")
    
    # 在主线程中运行GPU阶段
    gpu_stage.run()  # 阻塞直到所有GPU任务完成
    
    # 等待其他阶段完成
    cpu_stage.wait()
    postprocess_stage.wait()
    
    print("\n最终结果:", results)
    print("CUDA工作流完成！\n")


def demo4_gui_simulation():
    """演示4: 模拟GUI渲染 - 后台处理 + 主线程渲染"""
    print("=== 演示4: 模拟GUI渲染（后台处理 + 主线程更新UI）===")
    
    N = 5
    images = [f"image_{i}.png" for i in range(N)]
    processed_images = [None] * N
    
    # 后台图像处理（多线程）
    process_stage = tq.Stage("ImageProcess", 2, 3, lambda i: (
        time.sleep(0.1),  # 模拟图像处理
        processed_images.__setitem__(i, f"processed_{images[i]}"),
        print(f"[后台线程] 处理图像: {images[i]}")
    ))
    
    # UI更新（必须在主线程 - Tkinter/PyQt要求）
    ui_stage = tq.StageCurrent("UI-Update", 1, 5, lambda i: (
        print(f"[主线程-GUI] 更新UI显示: {processed_images[i]}")
    ))
    
    # 链接
    tq.chain(process_stage, ui_stage)
    
    process_stage.setTaskCount(N)
    ui_stage.setTaskCount(N)
    
    # 模拟用户操作触发处理
    def user_action():
        print("用户触发批量图像处理...")
        for i in range(N):
            process_stage.push(i)
            time.sleep(0.05)  # 模拟用户逐个选择图像
    
    user_thread = threading.Thread(target=user_action)
    user_thread.start()
    
    print("主线程（GUI事件循环）等待UI更新任务...\n")
    
    # 主线程运行UI更新
    ui_stage.run()  # 阻塞直到所有UI更新完成
    
    process_stage.wait()
    
    print("GUI更新完成！\n")


if __name__ == "__main__":
    print("=" * 60)
    print("Python StageCurrent 演示程序")
    print("展示如何在主线程中执行特定任务（CUDA/GUI/等）")
    print("=" * 60)
    print()
    
    demo1_basic_current_thread()
    time.sleep(0.5)
    
    demo2_mixed_pipeline()
    time.sleep(0.5)
    
    demo3_cuda_simulation()
    time.sleep(0.5)
    
    demo4_gui_simulation()
    
    print("=" * 60)
    print("所有演示完成！")
    print("=" * 60)
