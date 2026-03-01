import threading
import queue
import sys
import traceback
from typing import Callable, Optional, List, Tuple

class Pipeline:
    """管理整个流水线的异常"""
    def __init__(self):
        self.exceptions: List[Tuple[str, int, Exception]] = []
        self.exception_lock = threading.Lock()
    
    def add_exception(self, stage_name: str, index: int, exception: Exception):
        """添加异常到pipeline"""
        with self.exception_lock:
            self.exceptions.append((stage_name, index, exception))
    
    def has_exceptions(self) -> bool:
        """检查是否有异常"""
        with self.exception_lock:
            return len(self.exceptions) > 0
    
    def get_exception_summary(self) -> str:
        """获取异常摘要"""
        with self.exception_lock:
            if not self.exceptions:
                return "No exceptions"
            
            msg = f"{len(self.exceptions)} task(s) failed in pipeline:\n"
            for stage, idx, exc in self.exceptions[:5]:  # 只显示前5个
                msg += f"  - Stage '{stage}', task {idx}: {type(exc).__name__}: {exc}\n"
            
            if len(self.exceptions) > 5:
                msg += f"  ... and {len(self.exceptions) - 5} more errors\n"
            
            return msg

class TaskQueue:
    def __init__(self):
        self.tasks = queue.Queue()

    def pushTask(self, task: Callable):
        self.tasks.put(task)

    def popTask(self) -> Callable:
        return self.tasks.get()

    def empty(self) -> bool:
        return self.tasks.empty()

class BoundedTaskQueue:
    def __init__(self, capacity: int = 20):
        self.capacity = capacity
        self.tasks = queue.Queue(maxsize=capacity)

    def setCapacity(self, capacity: int):
        if not self.tasks.empty():
            raise RuntimeError("setCapacity must be called before pushing tasks")
        self.capacity = capacity
        self.tasks = queue.Queue(maxsize=capacity)
    def pushTask(self, task: Callable):
        self.tasks.put(task)   # 阻塞直到有空间（等价 cv_producer）

    def popTask(self) -> Callable:
        return self.tasks.get()

    def empty(self) -> bool:
        return self.tasks.empty()

class ThreadPool:
    def __init__(self, num_threads, task_queue,
                 task_counter_ref, done_cv: threading.Condition):
        self.task_queue = task_queue
        self.task_counter_ref = task_counter_ref
        self.done_cv = done_cv
        self.stop = False
        self.workers = []

        for _ in range(num_threads):
            t = threading.Thread(target=self._worker)
            t.start()
            self.workers.append(t)

    def _worker(self):
        while True:
            task = self.task_queue.popTask()
            if self.stop:
                break
            
            # ✅ 异常处理：确保taskFinished总是被调用
            try:
                task()
            except Exception as e:
                # 打印异常信息（任务内部应该已经处理了）
                print(f"[ERROR] Task execution failed: {e}", file=sys.stderr)
                traceback.print_exc()
            finally:
                self.taskFinished()

    def taskFinished(self):
        with self.done_cv:
            self.task_counter_ref[0] -= 1
            if self.task_counter_ref[0] == 0:
                self.stopAll()
                self.done_cv.notify_all()

    def stopAll(self):
        if not self.stop:
            self.stop = True
            for _ in self.workers:
                self.task_queue.pushTask(lambda: None)

    def join(self):
        for t in self.workers:
            t.join()


class ThreadPoolEx:
    def __init__(self, num_threads, queue_cls=TaskQueue):
        self.taskQueue = queue_cls()
        self.task_counter = [0]
        self.done_cv = threading.Condition()
        self.threadPool = ThreadPool(
            num_threads,
            self.taskQueue,
            self.task_counter,
            self.done_cv
        )

    def setTaskCount(self, n: int):
        self.task_counter[0] = n

    def pushTask(self, task: Callable):
        self.taskQueue.pushTask(task)

    def wait(self):
        with self.done_cv:
            self.done_cv.wait_for(lambda: self.task_counter[0] == 0)
        self.threadPool.join()

class CurrentThread:
    def __init__(self, task_queue, task_counter_ref, done_cv):
        self.task_queue = task_queue
        self.task_counter_ref = task_counter_ref
        self.done_cv = done_cv
        self.stop = False

    def run(self):
        while True:
            task = self.task_queue.popTask()
            if self.stop:
                break
            
            # ✅ 异常处理：确保taskFinished总是被调用
            try:
                task()
            except Exception as e:
                # 打印异常信息（任务内部应该已经处理了）
                print(f"[ERROR] Task execution failed: {e}", file=sys.stderr)
                traceback.print_exc()
            finally:
                self.taskFinished()

    def taskFinished(self):
        with self.done_cv:
            self.task_counter_ref[0] -= 1
            if self.task_counter_ref[0] == 0:
                self.stopAll()
                self.done_cv.notify_all()

    def stopAll(self):
        if not self.stop:
            self.stop = True
            self.task_queue.pushTask(lambda: None)

class CurrentThreadEx:
    def __init__(self, dummy_param=None):  # 为了保证调用方式和ThreadPoolEx一致
        self.taskQueue = BoundedTaskQueue()
        self.task_counter = [0]
        self.done_cv = threading.Condition()
        self.currentThread = CurrentThread(
            self.taskQueue,
            self.task_counter,
            self.done_cv
        )

    def setTaskCount(self, n: int):
        self.task_counter[0] = n

    def pushTask(self, task: Callable):
        self.taskQueue.pushTask(task)

    def run(self):
        self.currentThread.run()

class Stage:
    def __init__(self, name, num_workers, capacity, func, pipeline: Optional[Pipeline] = None):
        self.name = name
        self.func = func
        self.queue = BoundedTaskQueue(capacity)
        self.pool = ThreadPoolEx(num_workers, lambda: self.queue)
        self.next = None   # 由 chain 设置
        self._pipeline = pipeline  # Pipeline对象，用于异常管理

    @property
    def pipeline(self) -> Pipeline:
        """获取pipeline，如果没有就创建"""
        if self._pipeline is None:
            self._pipeline = Pipeline()
        return self._pipeline
    
    @pipeline.setter
    def pipeline(self, value: Pipeline):
        self._pipeline = value

    def setTaskCount(self, n):
        self.pool.setTaskCount(n)

    def push(self, index):
        self.queue.pushTask(lambda i=index: self._run(i))

    def _run(self, index):
        try:
            self.func(index)
        except Exception as e:
            # 记录异常到pipeline
            self.pipeline.add_exception(self.name, index, e)
            # 即使失败，也要传播到下游（让下游的计数器正确递减）
        finally:
            # 无论成功还是失败，都传播到下游
            if self.next:
                self.next.push(index)

    def wait(self):
        self.pool.wait()
        
        # ✅ 检查整个pipeline的异常
        if self.pipeline.has_exceptions():
            summary = self.pipeline.get_exception_summary()
            # 获取第一个异常作为cause
            first_exception = self.pipeline.exceptions[0][2]
            raise RuntimeError(summary) from first_exception

class StageCurrent:
    """
    在当前线程中执行任务的Stage，类似于C++的StageCurrent
    适用于需要在特定线程（如主线程）中执行的任务，例如：
    - GUI渲染（Tkinter、PyQt等）
    - CUDA操作（如果需要特定线程上下文）
    - 需要特定线程局部存储的操作
    """
    def __init__(self, name, dummy_param, capacity, func, pipeline: Optional[Pipeline] = None):
        """
        name: stage名称
        dummy_param: 为了保持与Stage接口一致（无实际意义）
        capacity: 队列容量
        func: 处理函数 func(index)
        pipeline: Pipeline对象，用于异常管理
        """
        self.name = name
        self.func = func
        self.executor = CurrentThreadEx(dummy_param)
        self.executor.taskQueue.setCapacity(capacity)
        self.next = None
        self._pipeline = pipeline

    @property
    def pipeline(self) -> Pipeline:
        """获取pipeline，如果没有就创建"""
        if self._pipeline is None:
            self._pipeline = Pipeline()
        return self._pipeline
    
    @pipeline.setter
    def pipeline(self, value: Pipeline):
        self._pipeline = value

    def setTaskCount(self, n):
        self.executor.setTaskCount(n)

    def push(self, index):
        self.executor.pushTask(lambda i=index: self._run(i))

    def _run(self, index):
        try:
            self.func(index)
        except Exception as e:
            # 记录异常到pipeline
            self.pipeline.add_exception(self.name, index, e)
            # 即使失败，也要传播到下游（让下游的计数器正确递减）
        finally:
            # 无论成功还是失败，都传播到下游
            if self.next:
                self.next.push(index)

    def run(self):
        """在当前线程中运行任务队列，阻塞直到所有任务完成"""
        self.executor.run()
        
        # ✅ 检查整个pipeline的异常
        if self.pipeline.has_exceptions():
            summary = self.pipeline.get_exception_summary()
            first_exception = self.pipeline.exceptions[0][2]
            raise RuntimeError(summary) from first_exception

def chain(stage_a, stage_b):
    """
    链接两个stage，自动共享pipeline
    支持任意顺序调用，会自动传播pipeline
    """
    stage_a.next = stage_b
    
    # ✅ 让stage_b使用stage_a的pipeline
    if stage_b.pipeline is not stage_a.pipeline:
        old_pipeline = stage_b.pipeline
        
        # 将stage_b及其下游的所有stage都改用stage_a的pipeline
        current = stage_b
        while current is not None:
            current.pipeline = stage_a.pipeline
            current = current.next
    
    return stage_b

