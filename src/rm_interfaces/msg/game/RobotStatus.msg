# 机器人性能体系数据 (裁判系统串口协议 V1.7.0 0x0201)
uint8 robot_id                          # 本机器人 ID
uint8 robot_level                       # 机器人等级
uint16 current_hp                       # 机器人当前血量
uint16 maximum_hp                       # 机器人血量上限
uint16 shooter_barrel_cooling_value     # 机器人枪口热量每秒冷却值
uint16 shooter_barrel_heat_limit        # 机器人枪口热量上限
uint16 projectile_allowance_17mm        # 17mm 弹丸剩余发射次数