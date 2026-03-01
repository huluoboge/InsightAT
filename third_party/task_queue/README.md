# TaskQueue - 异步任务队列计算模型

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![C++11](https://img.shields.io/badge/C%2B%2B-11-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B11)
[![Python](https://img.shields.io/badge/Python-3.6+-blue.svg)](https://www.python.org/)

TaskQueue是一个高效的多语言并发任务队列系统，支持C++和Python双语言实现。主要用于构建数据处理流水线，支持多线程并行计算和任务同步。

## 特性

- 🚀 **高性能**: 基于线程池的生产者-消费者模型
- 🔄 **流水线处理**: 支持多阶段数据流式处理
- 🛡️ **线程安全**: 完整的互斥锁和条件变量保护
- 📏 **容量控制**: 支持有界队列防止内存溢出
- 🔗 **易于扩展**: 模板化设计，支持自定义队列类型
- 🌍 **双语言**: C++和Python接口一致
- ⚡ **异步执行**: 非阻塞任务提交和同步等待完成

## 架构组件

### 核心类

- **TaskQueue**: 基本无界任务队列
- **BoundedTaskQueue**: 有界任务队列，支持容量限制
- **ThreadPool**: 多线程任务执行器
- **Stage**: 流水线处理阶段（多线程执行）
- **StageCurrent**: 在当前线程执行的流水线阶段（适用于CUDA/GUI等场景）
- **chain()**: 阶段链接函数

### 架构图

```
数据流: Input → Stage A → Stage B → Stage C → Output
线程池:   [线程1, 线程2] [线程1, 线程2] [线程1, 线程2]
队列:     有界队列(容量8) 有界队列(容量8) 有界队列(容量4)
```

## 快速开始

### 构建和运行

#### 方式1: 使用编译脚本（推荐）

```bash
# 自动编译C++并运行测试
./compile_and_run.bash
```

#### 方式2: 手动编译

```bash
# C++
g++ -std=c++11 -pthread task_queue_demo.cpp -o task_queue_demo
./task_queue_demo

# Python
python3 task_queue_demo.py
```

#### 方式3: 使用CMake

```bash
mkdir build && cd build
cmake ..
make
./task_queue_demo
```

### Python使用示例

```python
import task_queue as tq

# 创建三阶段流水线
def stage_a(i): data[i] *= 2
def stage_b(i): data[i] += 1
def stage_c(i): print(f"result: {i}, {data[i]}")

stage_a = tq.Stage("A", 2, 8, stage_a)  # 2线程，队列容量8
stage_b = tq.Stage("B", 2, 8, stage_b)
stage_c = tq.Stage("C", 2, 4, stage_c)

# 链接阶段
tq.chain(stage_a, stage_b)
tq.chain(stage_b, stage_c)

# 设置任务数量并启动
stage_a.setTaskCount(100)
stage_b.setTaskCount(100)
stage_c.setTaskCount(100)

for i in range(100):
    stage_a.push(i)

stage_c.wait()  # 等待所有任务完成
```

### C++使用示例

```cpp
#include "task_queue.hpp"

// 创建三阶段流水线
Stage stageA("A", 2, 8, [](int i) { data[i] *= 2; });
Stage stageB("B", 2, 8, [](int i) { data[i] += 1; });
Stage stageC("C", 2, 4, [](int i) { printf("%d,%d\n", i, data[i]); });

// 链接阶段
chain(stageA, stageB);
chain(stageB, stageC);

// 设置任务数量并启动
stageA.setTaskCount(100);
stageB.setTaskCount(100);
stageC.setTaskCount(100);

for (int i = 0; i < 100; ++i) {
    stageA.push(i);
}

stageC.wait();
```

### 混合流水线示例（多线程 + 当前线程）

**C++版本**:
```cpp
// CPU处理阶段（多线程）
Stage processStage("Process", 2, 4, [&](int i) {
    data[i] += 10;
});

// GPU/渲染阶段（当前线程 - CUDA上下文绑定）
StageCurrent renderStage("Render", 1, 8, [&](int i) {
    printf("GPU渲染: %d -> %d\n", i, data[i] * 100);
});

chain(processStage, renderStage);

// 在后台线程push任务
std::thread producer([&]() {
    for (int i = 0; i < N; ++i) {
        processStage.push(i);
    }
});

// 在主线程运行渲染阶段
renderStage.run();  // 阻塞直到完成
producer.join();
```

**Python版本**:
```python
import task_queue as tq

# CPU处理阶段（多线程）
process_stage = tq.Stage("Process", 2, 4, lambda i: data.__setitem__(i, data[i] + 10))

# GPU/渲染阶段（当前线程）
render_stage = tq.StageCurrent("Render", 1, 8, lambda i: print(f"GPU渲染: {i} -> {data[i] * 100}"))

tq.chain(process_stage, render_stage)

# 在后台线程push任务
def producer():
    for i in range(N):
        process_stage.push(i)

threading.Thread(target=producer).start()

# 在主线程运行渲染阶段
render_stage.run()  # 阻塞直到完成
```

## API 文档

### TaskQueue

```cpp
class TaskQueue {
public:
    void pushTask(std::function<void()> task);  // 添加任务
    std::function<void()> popTask();            // 获取任务（阻塞）
    bool empty();                               // 检查是否为空
};
```

### BoundedTaskQueue

```cpp
class BoundedTaskQueue {
public:
    BoundedTaskQueue(size_t capacity = 20);     // 构造函数
    void setCapacity(size_t capacity);           // 设置容量
    void pushTask(std::function<void()> task);  // 添加任务（阻塞）
    std::function<void()> popTask();            // 获取任务（阻塞）
    bool empty();                               // 检查是否为空
};
```

### ThreadPoolEx

```cpp
class ThreadPoolEx {
public:
    ThreadPoolEx(size_t numThreads);            // 构造函数
    void setTaskCount(int n);                   // 设置任务总数
    void pushTask(std::function<void()> task);  // 添加任务
    void wait();                                // 等待所有任务完成
};
```

### Stage

```cpp
class Stage {
public:
    Stage(const std::string& name, int num_workers, int capacity,
          std::function<void(int)> func);       // 构造函数
    void setTaskCount(int n);                   // 设置任务总数
    void push(int index);                       // 推送索引到流水线
    void wait();                                // 等待完成
};
```

### StageCurrent

```cpp
class StageCurrent {
public:
    StageCurrent(const std::string& name, int dummy_param, int capacity,
                 std::function<void(int)> func);  // 构造函数
    void setTaskCount(int n);                     // 设置任务总数
    void push(int index);                         // 推送索引到流水线
    void run();                                   // 在当前线程运行任务
};
```

**使用场景**:
- **CUDA程序**: CUDA上下文通常绑定到特定线程
- **GUI应用**: Tkinter/PyQt等要求UI更新在主线程
- **线程局部存储**: 需要特定线程上下文的操作

### chain 函数

```cpp
void chain(Stage& a, Stage& b);  // 将阶段a链接到阶段b
```

## 性能调优

### 队列容量选择

- **小容量队列** (4-8): 减少内存使用，增加线程竞争
- **大容量队列** (16-32): 减少线程竞争，增加内存使用
- **推荐**: 根据数据处理速度和内存限制选择

### 线程数量选择

- **CPU密集型**: 等于CPU核心数
- **I/O密集型**: CPU核心数的2-4倍
- **混合型**: CPU核心数的1-2倍

### 示例配置

```cpp
// 高并发数据处理
Stage stage("Processor", 8, 32, processFunc);

// 内存敏感应用
Stage stage("MemorySaver", 2, 4, processFunc);
```

## 构建要求

### C++版本

- **编译器**: GCC 4.8+ 或 Clang 3.5+ 或 MSVC 2015+
- **标准**: C++11 或更高
- **依赖**: POSIX线程库 (pthread)

### Python版本

- **版本**: Python 3.6+
- **依赖**: 仅标准库 (threading, queue)

## 项目结构

```
task_queue/
├── task_queue.hpp              # C++头文件
├── task_queue.py               # Python实现
├── task_queue_demo.cpp         # C++演示程序
├── task_queue_demo.py          # Python演示程序
├── task_queue_demo_current.py  # Python StageCurrent演示
├── compile_and_run.bash        # 编译运行脚本
├── CMakeLists.txt              # CMake构建配置
├── README.md                   # 项目文档
└── .gitignore                  # Git忽略文件
```

## 测试

运行完整测试套件：

```bash
# 运行C++和Python基本演示
./compile_and_run.bash

# 运行Python StageCurrent演示
python3 task_queue_demo_current.py
```

这将测试C++和Python实现的正确性，包括混合流水线（多线程 + 当前线程）。

## 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 贡献

欢迎提交问题和改进建议！

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

## 致谢

- 感谢C++标准库提供的基础并发原语
- 感谢Python threading和queue模块的优秀实现