接下来最重要的三个问题
在你开始集成之前，应该能回答：

Q1：为什么不能直接在 controller.update() 中调用 IPOPT？

因为求解时间可能超过控制周期，导致控制线程错过截止时间。

Q2：为什么需要两个 SPSC Queue？

因为状态与力矩命令沿相反方向传输，每个方向有独立的生产者与消费者。

Q3：为什么不能直接执行队列中的每一个 TorqueCommand？

因为旧命令可能对应过去的机器人状态。控制器需要检查求解结果的时间戳、参考版本、有效性以及力矩限制。
你现在的具体任务
先运行 test_realtime_exchange，确保数据交换模块正确。

完成后，我们进入下一课：
Lesson 8 — Integrate RealtimeExchange into Fr3NmpcController
届时我们会具体修改：

controller.hpp → 声明交换模块与线程状态。

controller.cpp → 实现状态发布、协调线程、结果读取和验证。

SolverWorker → 保留原有后台优化职责。
