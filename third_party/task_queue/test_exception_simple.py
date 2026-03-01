#!/usr/bin/env python3
"""
简化的异常处理测试
"""

import task_queue as tq
import threading
import time

print("=" * 60)
print("异常处理测试")
print("=" * 60)

# 测试1: 单个stage异常
print("\n测试1: 单个stage异常处理")
data = list(range(10))

def failing_task(i):
    if i == 5:
        raise ValueError(f"Task {i} failed!")
    data[i] *= 2

stage = tq.Stage("Test", 2, 8, failing_task)
stage.setTaskCount(10)

for i in range(10):
    stage.push(i)

try:
    stage.wait()
    print("❌ 失败：应该抛出异常")
except RuntimeError as e:
    print("✅ 成功：捕获到异常")
    print(f"   异常数量: {len(stage.pipeline.exceptions)}")

# 测试2: Pipeline异常传播
print("\n测试2: Pipeline异常传播")
data = list(range(10))

stage_a = tq.Stage("A", 2, 8, lambda i: data.__setitem__(i, data[i] * 2))
stage_b = tq.Stage("B", 2, 8, lambda i: 1/0 if i == 3 else data.__setitem__(i, data[i] + 10))
stage_c = tq.Stage("C", 2, 8, lambda i: None)

tq.chain(stage_a, stage_b)
tq.chain(stage_b, stage_c)

stage_a.setTaskCount(10)
stage_b.setTaskCount(10)
stage_c.setTaskCount(10)

def producer():
    for i in range(10):
        stage_a.push(i)

t = threading.Thread(target=producer)
t.daemon = True  # 设置为守护线程
t.start()

try:
    stage_c.wait()
    print("❌ 失败：应该抛出异常")
except RuntimeError as e:
    print("✅ 成功：Pipeline异常传播正确")
    print(f"   Pipeline共享: {stage_a.pipeline is stage_c.pipeline}")
    print(f"   异常数量: {len(stage_c.pipeline.exceptions)}")

# 测试3: 混合pipeline
print("\n测试3: 混合pipeline (Stage + StageCurrent)")
data = list(range(8))

cpu_stage = tq.Stage("CPU", 2, 4, lambda i: data.__setitem__(i, data[i] * 2) if i != 2 else 1/0)
gpu_stage = tq.StageCurrent("GPU", 1, 8, lambda i: None if i != 5 else 1/0)

tq.chain(cpu_stage, gpu_stage)

cpu_stage.setTaskCount(8)
gpu_stage.setTaskCount(8)

def producer2():
    for i in range(8):
        cpu_stage.push(i)

t = threading.Thread(target=producer2)
t.daemon = True  # 设置为守护线程
t.start()

try:
    gpu_stage.run()
    print("❌ 失败：应该抛出异常")
except RuntimeError as e:
    print("✅ 成功：混合pipeline异常处理正确")
    print(f"   异常数量: {len(gpu_stage.pipeline.exceptions)}")

# 测试4: chain顺序无关
print("\n测试4: chain顺序无关性")
data = list(range(5))

stage_a = tq.Stage("A", 2, 4, lambda i: data.__setitem__(i, data[i] * 2))
stage_b = tq.Stage("B", 2, 4, lambda i: data.__setitem__(i, data[i] + 1))
stage_c = tq.Stage("C", 2, 4, lambda i: None)

# 乱序chain
tq.chain(stage_b, stage_c)
tq.chain(stage_a, stage_b)

# 验证pipeline共享
if stage_a.pipeline is stage_b.pipeline is stage_c.pipeline:
    print("✅ 成功：Pipeline正确共享（乱序chain）")
else:
    print("❌ 失败：Pipeline未正确共享")

# 必须push任务并wait，否则线程池会一直等待
stage_a.setTaskCount(5)
stage_b.setTaskCount(5)
stage_c.setTaskCount(5)

for i in range(5):
    stage_a.push(i)

stage_c.wait()

print("\n" + "=" * 60)
print("所有测试完成！")
print("=" * 60)
