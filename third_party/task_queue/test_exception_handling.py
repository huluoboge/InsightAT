#!/usr/bin/env python3
"""
测试异常处理机制
验证：
1. 任务抛异常不会导致死锁
2. 异常会被正确收集和报告
3. Pipeline级别的异常管理
4. chain顺序无关性
"""

import task_queue as tq
import threading
import time

def test1_basic_exception_handling():
    """测试1: 基本异常处理 - 单个stage"""
    print("=" * 60)
    print("测试1: 基本异常处理")
    print("=" * 60)
    
    data = list(range(10))
    
    def failing_task(i):
        if i == 5:
            raise ValueError(f"Task {i} intentionally failed!")
        data[i] *= 2
    
    stage = tq.Stage("TestStage", 2, 8, failing_task)
    stage.setTaskCount(10)
    
    # 推送任务
    for i in range(10):
        stage.push(i)
    
    # 等待完成
    try:
        stage.wait()
        print("❌ 测试失败：应该抛出异常")
    except RuntimeError as e:
        print("✅ 测试通过：捕获到异常")
        print(f"异常信息:\n{e}")
        print(f"成功处理的任务: {[i for i in range(10) if i != 5 and data[i] != i]}")
    
    print()


def test2_pipeline_exception_propagation():
    """测试2: Pipeline异常传播 - 多个stage"""
    print("=" * 60)
    print("测试2: Pipeline异常传播")
    print("=" * 60)
    
    data = list(range(10))
    
    def stage_a_func(i):
        data[i] *= 2
    
    def stage_b_func(i):
        if i == 3:
            raise ValueError(f"Stage B: Task {i} failed!")
        data[i] += 10
    
    def stage_c_func(i):
        if i == 7:
            raise ValueError(f"Stage C: Task {i} failed!")
        print(f"Task {i}: {data[i]}")
    
    stage_a = tq.Stage("A", 2, 8, stage_a_func)
    stage_b = tq.Stage("B", 2, 8, stage_b_func)
    stage_c = tq.Stage("C", 2, 8, stage_c_func)
    
    tq.chain(stage_a, stage_b)
    tq.chain(stage_b, stage_c)
    
    stage_a.setTaskCount(10)
    stage_b.setTaskCount(10)
    stage_c.setTaskCount(10)
    
    # 在后台线程推送任务
    def producer():
        for i in range(10):
            stage_a.push(i)
    
    threading.Thread(target=producer).start()
    
    # 只等待最后一个stage
    try:
        stage_c.wait()
        print("❌ 测试失败：应该抛出异常")
    except RuntimeError as e:
        print("✅ 测试通过：捕获到pipeline异常")
        print(f"异常信息:\n{e}")
        
        # 验证所有stage共享同一个pipeline
        print(f"\n验证pipeline共享:")
        print(f"  stage_a.pipeline == stage_b.pipeline: {stage_a.pipeline is stage_b.pipeline}")
        print(f"  stage_b.pipeline == stage_c.pipeline: {stage_b.pipeline is stage_c.pipeline}")
    
    print()


def test3_no_deadlock_on_exception():
    """测试3: 异常不会导致死锁"""
    print("=" * 60)
    print("测试3: 异常不会导致死锁")
    print("=" * 60)
    
    data = list(range(20))
    
    def always_fail(i):
        raise RuntimeError(f"Task {i} always fails!")
    
    stage = tq.Stage("AlwaysFail", 2, 8, always_fail)
    stage.setTaskCount(20)
    
    start_time = time.time()
    
    for i in range(20):
        stage.push(i)
    
    try:
        stage.wait()
    except RuntimeError as e:
        elapsed = time.time() - start_time
        print(f"✅ 测试通过：在 {elapsed:.2f} 秒内完成（没有死锁）")
        print(f"收集到 {len(stage.pipeline.exceptions)} 个异常")
    
    print()


def test4_chain_order_independence():
    """测试4: chain顺序无关性"""
    print("=" * 60)
    print("测试4: chain顺序无关性")
    print("=" * 60)
    
    data = list(range(5))
    
    stage_a = tq.Stage("A", 2, 4, lambda i: data.__setitem__(i, data[i] * 2))
    stage_b = tq.Stage("B", 2, 4, lambda i: data.__setitem__(i, data[i] + 1))
    stage_c = tq.Stage("C", 2, 4, lambda i: None)
    
    # 乱序chain
    print("乱序链接: B→C, A→B")
    tq.chain(stage_b, stage_c)
    tq.chain(stage_a, stage_b)
    
    # 验证pipeline共享
    print(f"✅ Pipeline共享验证:")
    print(f"  stage_a.pipeline is stage_b.pipeline: {stage_a.pipeline is stage_b.pipeline}")
    print(f"  stage_b.pipeline is stage_c.pipeline: {stage_b.pipeline is stage_c.pipeline}")
    print(f"  stage_a.pipeline is stage_c.pipeline: {stage_a.pipeline is stage_c.pipeline}")
    
    # 必须push任务并wait，否则线程池会一直等待
    stage_a.setTaskCount(5)
    stage_b.setTaskCount(5)
    stage_c.setTaskCount(5)
    
    for i in range(5):
        stage_a.push(i)
    
    stage_c.wait()
    
    print()


def test5_mixed_pipeline_exception():
    """测试5: 混合pipeline（Stage + StageCurrent）异常处理"""
    print("=" * 60)
    print("测试5: 混合pipeline异常处理")
    print("=" * 60)
    
    data = list(range(8))
    
    def cpu_func(i):
        if i == 2:
            raise ValueError(f"CPU stage: Task {i} failed!")
        data[i] *= 2
    
    def gpu_func(i):
        if i == 5:
            raise ValueError(f"GPU stage: Task {i} failed!")
        print(f"GPU处理: {i} -> {data[i]}")
    
    cpu_stage = tq.Stage("CPU", 2, 4, cpu_func)
    gpu_stage = tq.StageCurrent("GPU", 1, 8, gpu_func)
    
    tq.chain(cpu_stage, gpu_stage)
    
    cpu_stage.setTaskCount(8)
    gpu_stage.setTaskCount(8)
    
    # 后台推送
    def producer():
        for i in range(8):
            cpu_stage.push(i)
    
    threading.Thread(target=producer).start()
    
    # 主线程运行GPU stage
    try:
        gpu_stage.run()
        print("❌ 测试失败：应该抛出异常")
    except RuntimeError as e:
        print("✅ 测试通过：混合pipeline异常处理正确")
        print(f"异常信息:\n{e}")
    
    print()


def test6_partial_failure():
    """测试6: 部分任务失败，其他任务继续执行"""
    print("=" * 60)
    print("测试6: 部分任务失败")
    print("=" * 60)
    
    results = []
    
    def partial_fail(i):
        if i % 3 == 0:
            raise ValueError(f"Task {i} failed (divisible by 3)")
        results.append(i)
    
    stage = tq.Stage("PartialFail", 2, 8, partial_fail)
    stage.setTaskCount(10)
    
    for i in range(10):
        stage.push(i)
    
    try:
        stage.wait()
    except RuntimeError as e:
        print(f"✅ 测试通过：部分任务失败")
        print(f"成功的任务: {sorted(results)}")
        print(f"失败的任务数: {len(stage.pipeline.exceptions)}")
        print(f"预期失败: [0, 3, 6, 9]")
        print(f"实际失败: {[idx for _, idx, _ in stage.pipeline.exceptions]}")
    
    print()


def test7_exception_details():
    """测试7: 异常详细信息"""
    print("=" * 60)
    print("测试7: 异常详细信息")
    print("=" * 60)
    
    def detailed_fail(i):
        if i == 0:
            raise ValueError("ValueError at task 0")
        elif i == 1:
            raise TypeError("TypeError at task 1")
        elif i == 2:
            raise RuntimeError("RuntimeError at task 2")
    
    stage = tq.Stage("DetailedFail", 2, 8, detailed_fail)
    stage.setTaskCount(3)
    
    for i in range(3):
        stage.push(i)
    
    try:
        stage.wait()
    except RuntimeError as e:
        print("✅ 测试通过：异常详细信息")
        print(f"\n异常摘要:\n{e}")
        
        print(f"\n详细异常列表:")
        for stage_name, idx, exc in stage.pipeline.exceptions:
            print(f"  - Stage: {stage_name}, Task: {idx}, Type: {type(exc).__name__}, Msg: {exc}")
    
    print()


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("异常处理测试套件")
    print("=" * 60 + "\n")
    
    test1_basic_exception_handling()
    time.sleep(0.2)
    
    test2_pipeline_exception_propagation()
    time.sleep(0.2)
    
    test3_no_deadlock_on_exception()
    time.sleep(0.2)
    
    test4_chain_order_independence()
    time.sleep(0.2)
    
    test5_mixed_pipeline_exception()
    time.sleep(0.2)
    
    test6_partial_failure()
    time.sleep(0.2)
    
    test7_exception_details()
    
    print("=" * 60)
    print("所有测试完成！")
    print("=" * 60)
