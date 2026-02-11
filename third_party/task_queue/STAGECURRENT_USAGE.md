# StageCurrent 使用指南

## 概述

`StageCurrent` 是一个特殊的流水线阶段，它在**当前线程**中执行任务，而不是在独立的工作线程池中执行。这对于需要特定线程上下文的场景非常重要。

## 为什么需要 StageCurrent？

### 1. CUDA 程序
CUDA 上下文通常绑定到特定线程。如果在不同线程中调用 CUDA API，可能会导致错误或性能问题。

```python
# ❌ 错误：在多线程中使用CUDA
stage = tq.Stage("GPU", 2, 8, lambda i: cuda_kernel(data[i]))  # 可能出错！

# ✅ 正确：在主线程中使用CUDA
stage = tq.StageCurrent("GPU", 1, 8, lambda i: cuda_kernel(data[i]))
stage.run()  # 在主线程执行
```

### 2. GUI 应用
大多数 GUI 框架（Tkinter、PyQt、wxPython）要求 UI 更新必须在主线程中进行。

```python
# ❌ 错误：在工作线程中更新UI
ui_stage = tq.Stage("UI", 2, 8, lambda i: update_ui(data[i]))  # 可能崩溃！

# ✅ 正确：在主线程中更新UI
ui_stage = tq.StageCurrent("UI", 1, 8, lambda i: update_ui(data[i]))
ui_stage.run()  # 在主线程执行
```

### 3. 线程局部存储
某些库使用线程局部存储（Thread-Local Storage），需要在特定线程中访问。

## 使用方法

### Python 示例

```python
import task_queue as tq
import threading

N = 10
data = list(range(N))

# 创建多线程处理阶段
cpu_stage = tq.Stage("CPU", 2, 4, lambda i: data.__setitem__(i, data[i] * 2))

# 创建当前线程执行阶段
gpu_stage = tq.StageCurrent("GPU", 1, 8, lambda i: print(f"GPU: {data[i]}"))

# 链接流水线
tq.chain(cpu_stage, gpu_stage)

# 设置任务数量
cpu_stage.setTaskCount(N)
gpu_stage.setTaskCount(N)

# 在后台线程中推送任务
def producer():
    for i in range(N):
        cpu_stage.push(i)

threading.Thread(target=producer).start()

# 在主线程中运行GPU阶段
gpu_stage.run()  # 阻塞直到所有任务完成

# 等待CPU阶段完成
cpu_stage.wait()
```

### C++ 示例

```cpp
#include "task_queue.hpp"
#include <thread>

int N = 10;
std::vector<int> data(N);

// 创建多线程处理阶段
Stage cpuStage("CPU", 2, 4, [&](int i) {
    data[i] *= 2;
});

// 创建当前线程执行阶段
StageCurrent gpuStage("GPU", 1, 8, [&](int i) {
    printf("GPU: %d\n", data[i]);
});

// 链接流水线
chain(cpuStage, gpuStage);

// 设置任务数量
cpuStage.setTaskCount(N);
gpuStage.setTaskCount(N);

// 在后台线程中推送任务
std::thread producer([&]() {
    for (int i = 0; i < N; ++i) {
        cpuStage.push(i);
    }
});

// 在主线程中运行GPU阶段
gpuStage.run();  // 阻塞直到所有任务完成

// 等待
producer.join();
cpuStage.wait();
```

## API 对比

### Stage vs StageCurrent

| 特性 | Stage | StageCurrent |
|------|-------|--------------|
| 执行方式 | 独立线程池 | 当前线程 |
| 并发执行 | ✅ 支持 | ❌ 串行执行 |
| 线程安全 | ✅ 自动处理 | ⚠️ 需要注意 |
| 适用场景 | CPU/IO密集型 | CUDA/GUI/TLS |
| 启动方式 | 自动启动 | 需调用 `run()` |

### 构造函数

```python
# Stage - 多线程执行
Stage(name, num_workers, capacity, func)
#            ^^^^^^^^^^^
#            实际使用的线程数

# StageCurrent - 当前线程执行
StageCurrent(name, dummy_param, capacity, func)
#                  ^^^^^^^^^^^
#                  无意义，仅为保持接口一致
```

### 关键方法

```python
# Stage
stage = tq.Stage("A", 2, 8, func)
stage.push(index)    # 推送任务到队列
stage.wait()         # 等待所有任务完成（自动执行）

# StageCurrent
stage = tq.StageCurrent("B", 1, 8, func)
stage.push(index)    # 推送任务到队列
stage.run()          # ⚠️ 必须手动调用！在当前线程执行
```

## 常见使用模式

### 模式 1: I/O → CPU → GPU

```python
# I/O 加载（多线程）
io_stage = tq.Stage("IO", 4, 8, load_data)

# CPU 预处理（多线程）
cpu_stage = tq.Stage("CPU", 8, 8, preprocess)

# GPU 计算（主线程）
gpu_stage = tq.StageCurrent("GPU", 1, 8, gpu_compute)

tq.chain(io_stage, cpu_stage)
tq.chain(cpu_stage, gpu_stage)

# 启动
threading.Thread(target=lambda: [io_stage.push(i) for i in range(N)]).start()
gpu_stage.run()  # 主线程执行GPU
```

### 模式 2: 后台处理 → UI 更新

```python
# 后台图像处理（多线程）
process_stage = tq.Stage("Process", 4, 8, process_image)

# UI 更新（主线程）
ui_stage = tq.StageCurrent("UI", 1, 8, update_ui)

tq.chain(process_stage, ui_stage)

# 启动
threading.Thread(target=lambda: [process_stage.push(i) for i in range(N)]).start()
ui_stage.run()  # 主线程更新UI
```

### 模式 3: 纯当前线程执行

```python
# 如果只需要在当前线程执行，不需要多线程
stage = tq.StageCurrent("Current", 1, 8, func)
stage.setTaskCount(N)

# 在后台线程推送任务
threading.Thread(target=lambda: [stage.push(i) for i in range(N)]).start()

# 在当前线程执行
stage.run()
```

## 注意事项

### 1. 必须调用 `run()`

```python
# ❌ 错误：忘记调用 run()
stage = tq.StageCurrent("GPU", 1, 8, func)
stage.push(0)
# 任务永远不会执行！

# ✅ 正确
stage = tq.StageCurrent("GPU", 1, 8, func)
stage.push(0)
stage.run()  # 必须调用
```

### 2. `run()` 会阻塞

```python
stage = tq.StageCurrent("GPU", 1, 8, func)
stage.setTaskCount(10)

# 在后台推送任务
threading.Thread(target=lambda: [stage.push(i) for i in range(10)]).start()

print("开始执行...")
stage.run()  # 阻塞直到所有10个任务完成
print("执行完成！")
```

### 3. 串行执行

```python
# StageCurrent 在当前线程中串行执行任务
stage = tq.StageCurrent("GPU", 1, 8, lambda i: time.sleep(1))
stage.setTaskCount(10)

# 这将花费 10 秒（串行执行）
stage.run()
```

### 4. 队列容量仍然有效

```python
# 即使是 StageCurrent，队列容量仍然控制背压
stage = tq.StageCurrent("GPU", 1, 2, func)  # 容量=2

# 如果推送速度 > 执行速度，push() 会阻塞
for i in range(100):
    stage.push(i)  # 当队列满时会阻塞
```

## 性能考虑

### 优点
- ✅ 避免线程切换开销
- ✅ 保证线程上下文（CUDA/GUI）
- ✅ 简化调试（单线程执行）

### 缺点
- ❌ 无法并行执行
- ❌ 可能成为性能瓶颈
- ❌ 需要手动管理执行

### 优化建议

1. **将耗时操作放在多线程阶段**
```python
# ✅ 好：耗时的预处理在多线程
cpu_stage = tq.Stage("CPU", 8, 8, expensive_preprocess)
gpu_stage = tq.StageCurrent("GPU", 1, 8, quick_gpu_call)
```

2. **使用合适的队列容量**
```python
# 如果GPU很快，使用小容量避免内存占用
gpu_stage = tq.StageCurrent("GPU", 1, 4, fast_gpu)

# 如果GPU较慢，使用大容量避免CPU等待
gpu_stage = tq.StageCurrent("GPU", 1, 16, slow_gpu)
```

3. **批处理优化**
```python
# ❌ 差：每次处理一个
gpu_stage = tq.StageCurrent("GPU", 1, 8, lambda i: gpu_process_one(data[i]))

# ✅ 好：批量处理
batch = []
def batch_process(i):
    batch.append(data[i])
    if len(batch) >= 32:
        gpu_process_batch(batch)
        batch.clear()

gpu_stage = tq.StageCurrent("GPU", 1, 8, batch_process)
```

## 完整示例

查看以下文件获取完整示例：
- **Python**: `task_queue_demo_current.py`
- **C++**: `task_queue_demo.cpp` (main3 函数)

运行演示：
```bash
# Python
python3 task_queue_demo_current.py

# C++
./task_queue_demo
```

## 总结

`StageCurrent` 是处理需要特定线程上下文场景的强大工具。记住：

1. 用于 CUDA、GUI、线程局部存储等场景
2. 必须手动调用 `run()` 方法
3. 任务在当前线程中串行执行
4. 可以与普通 `Stage` 混合使用构建复杂流水线

正确使用 `StageCurrent` 可以让你的异步流水线既高效又安全！
