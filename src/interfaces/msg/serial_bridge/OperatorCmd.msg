# 云台手下发数据

float32 target_position_x # 目标位置 x 轴坐标，单位 m。当发送目标机器人 ID 时，该值为 0。
float32 target_position_y # 目标位置 y 轴坐标，单位 m。当发送目标机器人 ID 时，该值为 0。
uint8 cmd_keyboard # 云台手按下的键盘按键通用键值。无按键按下，则为 0。
uint8 target_robot_id # 对方机器人 ID。当发送坐标数据时，该值为 0。