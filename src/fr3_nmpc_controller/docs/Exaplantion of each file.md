troller.hpp 将增加控制状态、接口索引、求解结果有效期和数据交接成员。

controller.cpp 将实现接口检查、控制循环、有效力矩输出和故障处理。

optimizer.cpp 暂时不修改，因为你的离线及只读测试已经成功。

fr3_sim_bringup 只负责配置和启动仿真，不应该包含 NMPC 数学算法
