import task_queue as tq
import threading
import time

N = 100
# data = [1]*N
data = list(range(N))
def stage_a_fn(i):
    data[i]*=2

def stage_b_fn(i):
    data[i]+=1

def stage_c_fn(i):
    print("final:",i, data[i])

stage_a = tq.Stage("A", 2, 8, stage_a_fn)
stage_b = tq.Stage("B", 2, 8, stage_b_fn)
stage_c = tq.Stage("C", 2, 8, stage_c_fn)

tq.chain(stage_a, stage_b)
tq.chain(stage_b, stage_c)

stage_a.setTaskCount(N)
stage_b.setTaskCount(N)
stage_c.setTaskCount(N)

# 在另一个线程中push任务，主线程可以继续做其他事情
def start_worker():
    print("开始push任务...")
    for i in range(N):
        stage_a.push(i)
        # 模拟一些处理时间
    print("完成push任务")

producer_thread = threading.Thread(target=start_worker)
producer_thread.start()

# 主线程可以同时做其他工作
print("主线程可以同时处理其他任务...")
for i in range(5):
    print(f"主线程正在处理其他工作 {i+1}/5")
    time.sleep(0.05)
print("主线程完成其他工作")

# 等待生产者线程完成
producer_thread.join()

# 等待流水线处理完成
print("等待流水线处理完成...")
stage_c.pool.wait()
print("所有任务处理完成")
