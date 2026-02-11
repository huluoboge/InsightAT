#include "task_queue.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <numeric>
#include <stdio.h>
#include <thread>

int main1()
{
    int N = 100;

    ThreadPoolEx<BoundedTaskQueue> a(2), b(2), c(2);
    a.taskQueue.setCapacity(8);
    b.taskQueue.setCapacity(8);
    c.taskQueue.setCapacity(8);
    a.setTaskCount(N);
    b.setTaskCount(N);
    c.setTaskCount(N);

    std::vector<int> datas(N, 0);
    std::iota(datas.begin(), datas.end(), 0);
    std::future<void> future = std::async(std::launch::async, [&]() {
        for (int i = 0; i < N; ++i) {
            a.pushTask([&, i]() -> void {
                datas[i] *= 2;
                b.pushTask([&, i]() -> void {
                    datas[i] += 1;
                    c.pushTask([&, i]() -> void {
                        printf("%d,%d\n", i, datas[i]);
                    });
                });
            });
        }
    });

    c.wait();

    return 0;
}

int main2()
{
    int N = 100;

    std::vector<int> datas(N, 0);
    std::iota(datas.begin(), datas.end(), 0);

    Stage stageA("A", 2, 8, [&](int i) {
        datas[i] *= 2;
    });

    Stage stageB("B", 2, 8, [&](int i) {
        datas[i] += 1;
    });

    Stage stageC("C", 2, 4, [&](int i) {
        printf("%d,%d\n", i, datas[i]);
    });
    chain(stageA, stageB);
    chain(stageB, stageC);
    stageA.setTaskCount(N);
    stageB.setTaskCount(N);
    stageC.setTaskCount(N);

    // 在另一个线程中push任务，主线程可以继续做其他事情
    std::future<void> future = std::async(std::launch::async, [&]() {
        for (int i = 0; i < N; ++i) {
            stageA.push(i);
        }
    });

    // 主线程可以同时做其他工作
    printf("主线程可以同时处理其他任务...\n");
    for (int i = 0; i < 5; ++i) {
        printf("主线程正在处理其他工作 %d/5\n", i + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    printf("主线程完成其他工作\n");

    // 等待流水线处理完成
    printf("等待流水线处理完成...\n");
    stageC.wait();
    printf("所有任务处理完成\n");

    return 0;
}

int main3()
{
    int N = 10; // 使用更少的任务来演示

    std::vector<int> datas(N, 0);
    std::iota(datas.begin(), datas.end(), 0);

    // 使用泛型Stage，支持CurrentThreadEx（在当前线程执行）
    // 这对于渲染线程等需要在主线程工作的场景很有用
    StageT<CurrentThreadEx<BoundedTaskQueue>> renderStage("Render", 2, 8,
        [&](int i) {
            printf("渲染线程处理: %d -> %d\n", i, datas[i]);
            datas[i] *= 100; // 模拟渲染工作
        });

    StageT<ThreadPoolEx<BoundedTaskQueue>> processStage("Process", 2, 4, [&](int i) {
        datas[i] += 10; // CPU密集型处理
    });

    // 链接：先处理，再渲染
    processStage.setNext(&renderStage);

    processStage.setTaskCount(N);
    renderStage.setTaskCount(N);

    printf("开始混合流水线：多线程处理 + 当前线程渲染\n");
    std::future<void> future = std::async(std::launch::async, [&]() {
        for (int i = 0; i < N; ++i) {
            processStage.push(i);
        }
    });

    // 手动调用渲染阶段的run（因为它是CurrentThreadEx）
    renderStage.run();
    // 等待处理阶段完成
    processStage.wait();
    printf("处理阶段完成，现在在当前线程执行渲染...\n");

    printf("所有阶段完成！\n");
    return 0;
}

int main()
{
    printf("=== 演示1: 手动线程池 ===\n");
    main1();
    printf("\n=== 演示2: 流水线（生产者线程） ===\n");
    main2();
    printf("\n=== 演示3: 混合流水线（ThreadPool + CurrentThread） ===\n");
    main3();
    return 0;
}
