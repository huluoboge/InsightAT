#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

// 任务队列
class TaskQueue {
public:
    // 向队列添加任务
    void pushTask(std::function<void()> task)
    {
        std::unique_lock<std::mutex> lock(mtx);
        tasks.push(task);
        cv.notify_one(); // 通知一个等待的线程
    }

    // 从队列中取出任务
    std::function<void()> popTask()
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !tasks.empty(); }); // 等待直到队列有任务
        auto task = tasks.front();
        tasks.pop();
        return task;
    }

    bool empty()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return tasks.empty();
    }

private:
    std::queue<std::function<void()>> tasks;
    std::mutex mtx;
    std::condition_variable cv;
};

// 有界任务队列，用于在I/O和处理任务之间传递数据
class BoundedTaskQueue {
public:
    BoundedTaskQueue(size_t capacity = 20)
        : capacity(capacity)
    {
    }

    void setCapacity(size_t capacity)
    {
        this->capacity = capacity;
    }
    // 向队列中添加任务
    void pushTask(std::function<void()> task)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv_producer.wait(lock, [this] { return tasks.size() < capacity; }); // 等待缓冲区有空位
        tasks.push(task);
        cv_consumer.notify_one(); // 通知消费者有新的任务
    }

    // 从队列中取出任务
    std::function<void()> popTask()
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv_consumer.wait(lock, [this] { return !tasks.empty(); }); // 等待队列中有任务
        auto task = tasks.front();
        tasks.pop();
        cv_producer.notify_one(); // 通知生产者可以继续生产
        return task;
    }

    bool empty()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return tasks.empty();
    }

protected:
    std::queue<std::function<void()>> tasks;
    std::mutex mtx;
    std::condition_variable cv_producer, cv_consumer;
    size_t capacity; // 队列的最大容量
};

// 线程池

template <typename TaskQueueT>
class ThreadPool {
public:
    ThreadPool(size_t numThreads,
        TaskQueueT& _taskQueue,
        std::atomic<int>& taskCounter,
        std::condition_variable& doneCV,
        std::mutex& doneMtx)
        : taskQueue(_taskQueue)
        , stop(false)
        , taskCounter(taskCounter)
        , doneCV(doneCV)
        , doneMtx(doneMtx)
    {
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    auto task = taskQueue.popTask();
                    if (stop)
                        break;
                    task(); // 执行任务
                    taskFinished();
                }
            });
        }
    }
    ~ThreadPool()
    {
        stopAll();
        for (auto& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void stopAll()
    {
        if (!stop) {
            stop = true;
            for (size_t i = 0; i < workers.size(); ++i) {
                taskQueue.pushTask([] { }); // 发送空任务让线程退出
            }
        }
    }

    void taskFinished()
    {
        std::unique_lock<std::mutex> lock(doneMtx);
        --taskCounter; // 减少任务计数器
        if (taskCounter == 0) {
            stopAll();
            doneCV.notify_all(); // 当所有任务完成时，通知主线程
        }
    }

private:
    std::vector<std::thread> workers;
    TaskQueueT& taskQueue;
    bool stop;
    std::atomic<int>& taskCounter; // 任务计数器，追踪未完成任务
    std::condition_variable& doneCV; // 用于通知任务完成
    std::mutex& doneMtx; // 用于任务计数器的互斥锁
};

template <typename TaskQueueT>
class ThreadPoolEx {
public:
    using ThreadPoolPtr = std::shared_ptr<ThreadPool<TaskQueueT>>;

    TaskQueueT taskQueue;
    ThreadPoolEx(size_t numThreads)
    {
        threadPool = std::make_shared<ThreadPool<TaskQueueT>>(numThreads, taskQueue, taskCounter, doneCV, doneMtx);
    }

    void setTaskCount(int n)
    {
        taskCounter = n;
    }

    void pushTask(std::function<void()> task)
    {
        taskQueue.pushTask(task);
    }

    void wait()
    {
        std::unique_lock<std::mutex> lock(doneMtx);
        doneCV.wait(lock, [&] { return taskCounter == 0; });
    }
    // void restart()
    // {
    //     threadPool->restart();
    // }

private:
    ThreadPoolPtr threadPool;
    std::atomic<int> taskCounter;
    std::condition_variable doneCV;
    std::mutex doneMtx;
};

template <typename TaskQueueT>
class CurrentThread {
public:
    CurrentThread(
        TaskQueueT& _taskQueue,
        std::atomic<int>& taskCounter,
        std::condition_variable& doneCV,
        std::mutex& doneMtx)
        : taskQueue(_taskQueue)
        , stop(false)
        , taskCounter(taskCounter)
        , doneCV(doneCV)
        , doneMtx(doneMtx)
    {
    }

    void run()
    {
        while (true) {
            auto task = taskQueue.popTask();
            if (stop)
                break;
            task(); // 执行任务
            taskFinished();
        }
    }
    ~CurrentThread()
    {
        stopAll();
    }

    void stopAll()
    {
        if (!stop) {
            stop = true;
            taskQueue.pushTask([] { }); // 发送空任务让线程退出
        }
    }

    void taskFinished()
    {
        std::unique_lock<std::mutex> lock(doneMtx);
        --taskCounter; // 减少任务计数器
        if (taskCounter == 0) {
            stopAll();
            doneCV.notify_all(); // 当所有任务完成时，通知主线程
        }
    }

private:
    // std::vector<std::thread> workers;
    TaskQueueT& taskQueue;
    bool stop;
    std::atomic<int>& taskCounter; // 任务计数器，追踪未完成任务
    std::condition_variable& doneCV; // 用于通知任务完成
    std::mutex& doneMtx; // 用于任务计数器的互斥锁
};

template <typename TaskQueueT>
class CurrentThreadEx {
public:
    using CurrentThreadPtr = std::shared_ptr<CurrentThread<TaskQueueT>>;

    TaskQueueT taskQueue;
    CurrentThreadEx(int) // 为了保证调用方式和ThreadPoolEx一致，这里并没有意义
    {
        currentThread = std::make_shared<CurrentThread<TaskQueueT>>(taskQueue, taskCounter, doneCV, doneMtx);
    }

    void setTaskCount(int n)
    {
        taskCounter = n;
    }

    void pushTask(std::function<void()> task)
    {
        taskQueue.pushTask(task);
    }

    void run()
    {
        currentThread->run();
    }

private:
    CurrentThreadPtr currentThread;
    std::atomic<int> taskCounter;
    std::condition_variable doneCV;
    std::mutex doneMtx;
};

// 基类，用于StageT链接
class StageBase {
public:
    virtual ~StageBase() = default;
    virtual void push(int index) = 0;
};

// 泛型Stage类，支持不同的执行器类型
template <typename ExecutorT>
class StageT : public StageBase {
public:
    using Func = std::function<void(int)>;

    // 构造函数 - 对于ThreadPoolEx需要num_workers，对于CurrentThreadEx不需要
    StageT(const std::string& name, int threads, int capacity, Func func)
        : name_(name)
        , executor_(threads)
        , func_(std::move(func))
    {
        executor_.taskQueue.setCapacity(capacity);
    }

    void setTaskCount(int n)
    {
        executor_.setTaskCount(n);
    }

    void push(int index) override
    {
        executor_.pushTask([this, index]() {
            run(index);
        });
    }

    void wait()
    {
        executor_.wait();
    }

    // 对于CurrentThreadEx，需要手动调用run
    void run()
    {
        executor_.run();
    }

    // 公共方法用于链接
    void setNext(StageBase* next)
    {
        next_ = next;
    }

private:
    void run(int index)
    {
        func_(index);
        if (next_) {
            next_->push(index);
        }
    }

private:
    std::string name_;
    ExecutorT executor_;
    Func func_;
    StageBase* next_ = nullptr;

    // 为了让所有StageT实例都可以访问next_
    template <typename AnyExecutorT>
    friend class StageT;
};

using Stage = StageT<ThreadPoolEx<BoundedTaskQueue>>;
using StageCurrent = StageT<CurrentThreadEx<BoundedTaskQueue>>;

// 通用的chain函数，支持不同类型的StageT
template <typename Stage1, typename Stage2>
void chain(Stage1& a, Stage2& b)
{
    a.setNext(&b);
}
