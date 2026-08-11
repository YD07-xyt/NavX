# 当前底盘状态

float32 velocity # 底盘x轴方向的速度（m/s）
float32 omega # 底盘绕yaw轴旋转角速度（rad/s）
float32 leg_h # 两腿等效腿长的平均值（m）
float32 leg_psi # 两腿等效摆角的平均值（rad），向下为0，后摆为正

uint8 leg_mode # 串联腿模式
# 0:Dead            ///< 死亡模式
# 1:Recovery        ///< 恢复模式
# 2:Flight          ///< 飞行模式
# 3:Jump,           ///< 跳跃模式
# 4:Mature,         ///< 成熟模式
# 5:Step,           ///< 上台阶模式
# 6:Abnormal        ///< 处于异常状态

int16 remaining_energy_supercap # 当前超级电容电量中能使用的部分（J），即总电容×(当前电压²-最小保证电压²)/2
int16 remaining_energy_buffercap # 当前缓冲电容电量中能使用的部分（J），即总电容×(当前电压²-最小保证电压²)/2
int16 curr_chassis_pwr # 当前底盘功率（W）
int16 rfr_pwr_limit # 裁判系统电管的最大底盘取电功率（W）