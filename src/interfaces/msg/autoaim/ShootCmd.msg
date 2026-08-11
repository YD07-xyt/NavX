# 自瞄云台控制和发弹指令

float32 yaw
float32 pitch
# 期望的云台系相对于电控世界系的欧拉角，单位rad。yaw在-pi~pi，pitch在-pi/2~pi/2。
# 即期望云台指向：电控世界系先绕着z轴旋转yaw对应的角度，再绕着旋转后的y轴旋转pitch对应的角度。

uint8 shoot_mode
# 0: 不发弹, 1: 单发, 2: 连发