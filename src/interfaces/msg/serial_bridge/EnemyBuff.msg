# 解析出信息波时发送，没解出来不发送。无敌状态视为防御增益为100%。

uint8 enemy_1_defense_buff # 对方1号英雄机器人防御增益（百分比，值为 50 表示 50% 防御增益，下同）
uint8 enemy_1_defense_debuff # 对方1号英雄机器人负防御增益（即易伤）（百分比，值为 30 表示 -30% 防御增益，下同）

uint8 enemy_2_defense_buff # 对方2号工程机器人防御增益
uint8 enemy_2_defense_debuff # 对方2号工程机器人负防御增益

uint8 enemy_3_defense_buff # 对方3号步兵机器人防御增益
uint8 enemy_3_defense_debuff # 对方3号步兵机器人负防御增益

uint8 enemy_4_defense_buff # 对方4号步兵机器人防御增益
uint8 enemy_4_defense_debuff # 对方4号步兵机器人负防御增益

uint8 enemy_sentry_defense_buff # 对方7号哨兵机器人防御增益
uint8 enemy_sentry_defense_debuff # 对方7号哨兵机器人负防御增益

uint8 enemy_1_status # 对方1号英雄机器人主要状态（0 为存活，1 为战亡，2 为无敌但不虚弱，3 为无敌且虚弱，下同）
uint8 enemy_2_status # 对方2号工程机器人主要状态
uint8 enemy_3_status # 对方3号步兵机器人主要状态
uint8 enemy_4_status # 对方4号步兵机器人主要状态
uint8 enemy_sentry_status # 对方7号哨兵机器人主要状态