import time
import numpy as np
from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber
from booster_interface.msg import LowCmd, MotorCmd
import threading
import queue
import rclpy
import math
import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory

# ===== 配置 =====
JOINT_DIM = 14  # 关节维度

def joint_state_thread(joint_queue, exit_event):
    """线程：采集关节状态并推送到队列"""
    ChannelFactory.Instance().Init(0)

    class JointStateHandler:
        def __init__(self, queue_):
            self.queue = queue_
            self.joint_q = np.zeros(JOINT_DIM, dtype=np.float32)
            self.frame_count = 0
            self.last_time = time.time()

        def __call__(self, low_state_msg):
            # 将 2~15 号电机的 q 拷到数组里
            for i, motor in enumerate(low_state_msg.motor_state_serial[2:2+JOINT_DIM]):
                self.joint_q[i] = motor.q

            # 推送到队列（丢弃溢出）
            try:
                self.queue.put(self.joint_q.copy(), timeout=0.01)
            except queue.Full:
                pass

            # 帧率统计（每 2 秒）
            self.frame_count += 1
            now = time.time()
            elapsed = now - self.last_time
            if elapsed >= 2.0:
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                print(f"[JointStateHandler] 2秒平均帧率: {fps:.2f} Hz")
                self.frame_count = 0
                self.last_time = now

    handler = JointStateHandler(joint_queue)
    channel_subscriber = B1LowStateSubscriber(handler)
    channel_subscriber.InitChannel()
    try:
        while not exit_event.is_set():
            time.sleep(0.001)
    finally:
        try:
            channel_subscriber.CloseChannel()
        except Exception:
            pass

def ros2_pub_thread(exit_event):
    """线程：ROS2 Float32 发布"""
    rclpy.init()
    from rclpy.node import Node

    class SimplePubNode(Node):
        def __init__(self):
            super().__init__('simple_pub_node')
            self.publisher = self.create_publisher(LowCmd, 'test_control', 10)
            self.frame_count = 0
            self.last_time = time.time()

        def pub_loop(self):
            rate = 1.0 / 500  # 500Hz
            while not exit_event.is_set():
                now = time.time()
                # 发布电机控制指令
                value = math.sin(now) + 0.5 * math.cos(2 * now) + np.random.normal(0, 0.01)
                msg = LowCmd()
                mc = MotorCmd()
                mc.q = value
                msg.motor_cmd.append(mc)
                self.publisher.publish(msg)
                self.frame_count += 1

                elapsed = now - self.last_time
                if elapsed >= 2.0:
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    print(f"[SimplePubNode] 2秒平均发布帧率: {fps:.2f} Hz, 最近一次发布值: {value:.4f}")
                    self.frame_count = 0
                    self.last_time = now

                time.sleep(rate)

    node = SimplePubNode()
    try:
        node.pub_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

# ===== mock计算过程读取最新关节反馈 =====
def cpu_intensive_process(exit_event, shm_name):
    """进程：高 CPU 占用计算，从共享内存读取最新关节 q"""
    print("[CPUProc] 启动高CPU占用进程...")
    # 连接到已有的共享内存
    shm = SharedMemory(name=shm_name)
    try:
        joint_view = np.ndarray((JOINT_DIM,), dtype=np.float32, buffer=shm.buf)
        count = 0
        total = 0.0
        last_time = time.time()

        while not exit_event.is_set():
            # 读取最新关节（无锁快照；如需更严谨可做双缓冲或 seqlock）
            q = joint_view.copy()
            total += float(np.sum(np.sin(q) * np.cos(q)))
            count += 1

            # 统计输出
            now = time.time()
            if now - last_time >= 2.0:
                print(f"[CPUProc] 2秒累计浮点运算: {count} 次, 当前total={total:.2f}")
                count = 0
                last_time = now

            # 适当让出时间片（防止把核吃满影响其它线程）
            # 注：如果你希望“尽可能高”的吞吐，可以去掉这句
            # time.sleep(0.0005)
    finally:
        shm.close()
        print("[CPUProc] 退出")

if __name__ == "__main__":
    # 进程事件：用于线程与进程共同退出
    exit_event = mp.Event()

    # 线程间队列：从关节订阅线程到主线程
    joint_queue = queue.Queue(maxsize=10)

    # 创建共享内存，用于“主线程写最新关节 q”，CPU 进程读取
    shm = SharedMemory(create=True, size=JOINT_DIM * 4)
    joint_buf = np.ndarray((JOINT_DIM,), dtype=np.float32, buffer=shm.buf)
    joint_buf[:] = 0.0

    # 启动订阅线程与发布线程（直接把 mp.Event 当普通 Event 用在线程里也 OK）
    t_sub = threading.Thread(target=joint_state_thread, args=(joint_queue, exit_event), daemon=True)
    t_pub = threading.Thread(target=ros2_pub_thread, args=(exit_event,), daemon=True)
    t_sub.start()
    t_pub.start()

    # 启动 CPU 进程
    cpu_proc = mp.Process(target=cpu_intensive_process, args=(exit_event, shm.name), daemon=True)
    cpu_proc.start()

    try:
        while True:
            try:
                qv = joint_queue.get(timeout=1.0)
                # 将最新关节写入共享内存（一次性切片拷贝）
                joint_buf[:] = qv
            except Exception:
                print("未收到关节数据")
    except KeyboardInterrupt:
        print("收到中断信号，退出中...")
        exit_event.set()
        # 等待线程与进程退出
        t_sub.join(timeout=2.0)
        t_pub.join(timeout=2.0)
        cpu_proc.join(timeout=2.0)
    finally:
        # 释放共享内存（只有创建者需要 unlink）
        try:
            shm.close()
            shm.unlink()
        except Exception:
            pass
        print("清理完成，已退出。")
