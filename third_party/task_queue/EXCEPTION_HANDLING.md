# 异常处理机制说明

## 问题背景

在原始设计中存在严重的线程安全问题：

### Python vs C++ 异常行为差异

**C++**:
- 线程中未捕获的异常会调用 `std::terminate()`
- 整个进程终止，不会出现死锁

**Python**:
- ❌ 线程中未捕获的异常**不会**终止主线程
- 异常只打印到stderr，线程静默退出
- **任务计数器无法正确递减** → 导致死锁！

### 原始代码的问题

```python
def _worker(self):
    while True:
        task = self.task_queue.popTask()
        if self.stop:
            break
        task()  # ❌ 如果这里抛异常，taskFinished()不会被调用！
        self.taskFinished()
```

**死锁场景**：
```python
# 假设有10个任务，第5个任务抛异常
stage.setTaskCount(10)
for i in range(10):
    stage.push(i)

stage.wait()  # ❌ 永远阻塞！计数器从10只减到9
```

## 解决方案

### 1. 双层异常处理

#### 第一层：Worker线程级别
```python
def _worker(self):
    while True:
        task = self.task_queue.popTask()
        if self.stop:
            break
        
        try:
            task()
        except Exception as e:
            print(f"[ERROR] Task execution failed: {e}", file=sys.stderr)
            traceback.print_exc()
        finally:
            self.taskFinished()  # ✅ 保证总是被调用
```

**作用**：防止死锁，确保计数器正确递减

#### 第二层：Stage任务级别
```python
def _run(self, index):
    try:
        self.func(index)
    except Exception as e:
        # 记录异常到pipeline
        self.pipeline.add_exception(self.name, index, e)
    finally:
        # 无论成功还是失败，都传播到下游
        if self.next:
            self.next.push(index)
```

**作用**：
- 收集异常信息
- 确保下游stage的计数器也能正确递减
- 避免级联失败

### 2. Pipeline级别的异常管理

```python
class Pipeline:
    """管理整个流水线的异常"""
    def __init__(self):
        self.exceptions = []  # (stage_name, index, exception)
        self.exception_lock = threading.Lock()
    
    def add_exception(self, stage_name, index, exception):
        with self.exception_lock:
            self.exceptions.append((stage_name, index, exception))
```

**优点**：
- ✅ 所有stage共享同一个Pipeline对象
- ✅ 只需等待最后一个stage
- ✅ 能看到整个pipeline的所有异常

### 3. 自动Pipeline共享

```python
def chain(stage_a, stage_b):
    stage_a.next = stage_b
    
    # 让stage_b及其下游都使用stage_a的pipeline
    if stage_b.pipeline is not stage_a.pipeline:
        current = stage_b
        while current is not None:
            current.pipeline = stage_a.pipeline
            current = current.next
    
    return stage_b
```

**优点**：
- ✅ 支持任意顺序调用chain
- ✅ 自动传播pipeline到所有下游stage

## 使用示例

### 基本用法

```python
import task_queue as tq

data = list(range(10))

def process(i):
    if i == 5:
        raise ValueError(f"Task {i} failed!")
    data[i] *= 2

stage = tq.Stage("Process", 2, 8, process)
stage.setTaskCount(10)

for i in range(10):
    stage.push(i)

try:
    stage.wait()
except RuntimeError as e:
    print(f"有任务失败: {e}")
    # 输出：1 task(s) failed in pipeline:
    #   - Stage 'Process', task 5: ValueError: Task 5 failed!
```

### Pipeline用法

```python
stage_a = tq.Stage("A", 2, 8, func_a)
stage_b = tq.Stage("B", 2, 8, func_b)  # 可能抛异常
stage_c = tq.Stage("C", 2, 8, func_c)

tq.chain(stage_a, stage_b)
tq.chain(stage_b, stage_c)

# 设置任务数
stage_a.setTaskCount(100)
stage_b.setTaskCount(100)
stage_c.setTaskCount(100)

# 推送任务
for i in range(100):
    stage_a.push(i)

# ✅ 只需等待最后一个stage
try:
    stage_c.wait()
except RuntimeError as e:
    print(f"Pipeline失败: {e}")
    # 会显示所有stage的所有异常
```

### 混合Pipeline

```python
cpu_stage = tq.Stage("CPU", 2, 4, cpu_func)
gpu_stage = tq.StageCurrent("GPU", 1, 8, gpu_func)

tq.chain(cpu_stage, gpu_stage)

cpu_stage.setTaskCount(N)
gpu_stage.setTaskCount(N)

# 后台推送
threading.Thread(target=lambda: [cpu_stage.push(i) for i in range(N)]).start()

# 主线程运行GPU stage
try:
    gpu_stage.run()
except RuntimeError as e:
    print(f"Pipeline失败: {e}")
```

## 关键特性

### 1. 不会死锁
- ✅ 即使所有任务都失败，也能正常完成
- ✅ 计数器保证正确递减

### 2. 异常信息完整
- ✅ 记录哪个stage、哪个任务失败
- ✅ 保留异常类型和消息
- ✅ 显示前5个异常（避免输出过多）

### 3. 部分失败继续执行
- ✅ 一个任务失败不影响其他任务
- ✅ 失败的任务也会传播到下游（让计数器正确）

### 4. 用户友好
- ✅ 只需等待最后一个stage
- ✅ 异常信息清晰易懂
- ✅ 支持任意顺序chain

## 测试验证

运行测试：
```bash
# 简化测试
python3 test_exception_simple.py

# 完整测试
python3 test_exception_handling.py
```

所有测试都验证了：
- ✅ 单个stage异常处理
- ✅ Pipeline异常传播
- ✅ 不会死锁
- ✅ chain顺序无关
- ✅ 混合pipeline异常处理
- ✅ 部分任务失败
- ✅ 异常详细信息

## 性能影响

异常处理机制的性能开销：
- **正常情况**：几乎无开销（只是try-finally）
- **异常情况**：
  - 异常收集：O(1)，只是append到列表
  - 异常打印：会输出到stderr
  - 任务传播：正常流程，无额外开销

## 总结

通过双层异常处理 + Pipeline级别管理，我们实现了：

1. **安全性**：不会死锁，计数器保证正确
2. **可见性**：用户能清楚知道哪里出错
3. **易用性**：只需等待最后一个stage
4. **灵活性**：支持任意顺序chain
5. **兼容性**：Stage和StageCurrent都支持

这个设计让Python的异常处理行为更接近C++，同时提供了更好的用户体验！
