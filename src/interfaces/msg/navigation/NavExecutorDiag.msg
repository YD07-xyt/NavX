# nav_executor 单个 control_tick 的完整诊断快照。
# header.stamp 是本周期开始时间；所有 map 坐标均使用 header.frame_id。
# cycle_result != CYCLE_EXECUTED 时，控制门槛、输入快照和当前任务状态仍有效；
# route/executor/MPC 区仅在对应阶段实际执行后有效。

std_msgs/Header header
uint64 control_cycle

# ==================== 控制门槛与输入拒绝 ====================

uint8 CYCLE_EXECUTED=0
uint8 CYCLE_WAITING_GLOBAL_COST_MAP=1
uint8 CYCLE_WAITING_DIRECTION_MAP=2
uint8 CYCLE_WAITING_STEP_MASK=3
uint8 CYCLE_INVALID_CHASSIS_STATE=4
uint8 CYCLE_NO_NEW_CHASSIS_STATE=5
uint8 CYCLE_CHASSIS_POSE_UNAVAILABLE=6
uint8 CYCLE_ROUTE_CONTEXT_UNAVAILABLE_BEFORE_TASK=7
uint8 CYCLE_ROUTE_CONTEXT_UNAVAILABLE_AFTER_TASK=8
uint8 cycle_result

# PathExecutor 当前运动状态；早退周期也保留 FSM 上一状态。
# motion_state: 0 DEAD, 1 IDLE, 2 FOLLOW, 3 SPIN, 4 PREPARE_SPIN,
# 5 HAZARD_RECOVERY, 6 STUCK_REVERSE, 7 FIXED, 8 STEPPING。
uint8 motion_state
# STEPPING 内部阶段：0 NONE, 1 PREPARING, 2 ARMED, 3 COMMITTED。
uint8 step_phase

# 最近一次被 node 丢弃的输入消息及累计次数；该状态跨周期保留。
uint8 INPUT_REJECTION_NONE=0
uint8 INPUT_REJECTION_CHASSIS_INVALID=1
uint8 INPUT_REJECTION_LOCAL_COST_MAP_BEFORE_GLOBAL_MAP=2
uint8 INPUT_REJECTION_DIRECTION_MAP_BEFORE_GLOBAL_MAP=3
uint8 INPUT_REJECTION_LOCAL_COST_MAP_EMPTY=4
uint8 INPUT_REJECTION_LOCAL_COST_MAP_SIZE_MISMATCH=5
uint8 INPUT_REJECTION_PREDICTION_MAP_SIZE_MISMATCH=6
uint8 INPUT_REJECTION_PREDICTION_DT_INVALID=7
uint8 INPUT_REJECTION_SPIN_MODE_INVALID=8
uint8 INPUT_REJECTION_IDLE_MODE_INVALID=9
uint8 INPUT_REJECTION_DIRECTION_MAP_SIZE_MISMATCH=10
uint8 last_input_rejection_reason
uint64 input_rejection_count

# ==================== 控制输入快照 ====================

uint64 chassis_state_sequence
bool chassis_state_valid
float64 chassis_velocity                 # 实测线速度 (m/s)
float64 chassis_angular_velocity         # 实测角速度 (rad/s)
float64 chassis_leg_height               # 实测等效腿长 (m)
float64 chassis_leg_psi                  # 实测等效摆角 (rad)
uint8 chassis_leg_mode
uint8 comp_stage

uint8 CHASSIS_CONTROL_STOPPED=0
uint8 CHASSIS_CONTROL_BLOCKED=1
uint8 CHASSIS_CONTROL_NORMAL=2
uint8 chassis_control_state

float64 remaining_energy_supercap        # 滤波后的可用超级电容能量 (J)
float64 remaining_energy_buffercap       # 滤波后的可用缓冲电容能量 (J)
float64 rfr_power_limit                  # 裁判系统允许底盘功率 (W)
bool high_performance_available

bool spin_requested
bool spin_high_priority
bool spin_fast

bool current_cost_map_available
uint32 prediction_map_count
float64 prediction_dt

# ==================== 任务、规划与路径跟踪 ====================

# goal_id 为 0 表示当前无导航目标；goal_* 仅在 goal_id != 0 时有效。
uint64 goal_id
float64 goal_x
float64 goal_y
bool goal_fixed

# active_path_goal_id 为 0 表示当前没有活动路径。
uint64 active_path_goal_id
bool has_hold_goal
float64 hold_goal_x                         # 仅在 has_hold_goal=true 时有效
float64 hold_goal_y
uint64 plan_generation
bool needs_plan

uint8 PLANNER_IDLE=0
uint8 PLANNER_PLANNING=1
uint8 PLANNER_COOLDOWN=2
uint8 planner_state
float64 planner_cooldown_remaining          # (s)，非 COOLDOWN 时为 0

uint8 PLANNER_RESULT_NONE=0
uint8 PLANNER_RESULT_PATH_ACCEPTED=1
uint8 PLANNER_RESULT_COMPLETE=2
uint8 PLANNER_RESULT_FIXED_GOAL=3
uint8 PLANNER_RESULT_FAILED=4
uint8 planner_last_result
string planner_last_failure_reason          # 最近一次有效规划失败；非失败时为空

# 当前 goal 生命周期内最近一次路径失效原因；新 goal 到达时重置。
uint8 REPLAN_NONE=0
uint8 REPLAN_ROUTE_TRACKING_LOST=1
uint8 REPLAN_PROJECTION_OUT_OF_MAP=2
uint8 REPLAN_PROJECTION_COST_EXCEEDED=3
uint8 REPLAN_STEP_BLOCKED_CURRENT=4
uint8 REPLAN_STEP_BLOCKED_PREDICTED=5
uint8 REPLAN_MPC_LETHAL=6
uint8 REPLAN_EXECUTOR_REPLAN_EVENT=7
uint8 REPLAN_PERFORMANCE_DEGRADED=8
uint8 REPLAN_PERFORMANCE_RECOVERED=9
uint8 REPLAN_HIGH_PRIORITY_SPIN_PREEMPTION=10
uint8 last_replan_reason
uint64 replan_count

# 这是本周期 Executor 实际使用的 route estimate。
uint8 ROUTE_NONE=0
uint8 ROUTE_TRACKED=1
uint8 ROUTE_LOST=2
uint8 route_status
float64 route_progress                        # (m)，route_status != ROUTE_NONE 时有效
float64 route_path_speed                      # (m/s)
float64 route_remaining_length                # (m)
float64 route_tracking_error                  # (m)

# ==================== 最终输出 ====================

uint8 COMMAND_NOT_EVALUATED=0
uint8 COMMAND_PUBLISHED=1
uint8 COMMAND_HELD_PREVIOUS=2
uint8 COMMAND_INVALID=3
uint8 command_status
float64 command_velocity                      # 最终下发线速度 (m/s)
float64 command_angular_velocity              # 最终下发角速度 (rad/s)
uint8 command_mode                            # 最终下发底盘模式
uint8 command_step_distance                   # 最终下发台阶距离 (cm)

# ==================== LPV Observer ====================

# 本周期 observer 更新事件。Observer 在非 NORMAL 控制阶段会保持 RESET。
# 0: NONE, 1: RESET, 2: INITIALIZED, 3: CORRECTED, 4: MODEL_STRESS
uint8 observer_event

# 最近一次 reset 的独立原因；该字段在重新初始化后仍保留。
# 0: NONE, 1: EXPLICIT_REQUEST, 2: NONFINITE_INPUT, 3: MODEL_DEGENERATE, 4: NONFINITE_PREDICTION
# 5: VELOCITY_INNOVATION, 6: STATE_SEQUENCE_GAP, 7: CONTROL_UPDATE_GAP, 8: COMMAND_RESYNCHRONIZED
# 9: CONTROL_UNAVAILABLE, 10: CONTROL_OUTPUT_INVALID, 11: MEASUREMENT_HISTORY_DISCONTINUITY
uint8 observer_last_reset_reason

bool observer_initialized
bool observer_validated
bool observer_prediction_available
bool observer_auxiliary_prediction_available # e_w/e_psi 可用；仅用于诊断
bool observer_velocity_correction_clipped

uint64 observer_state_sequence
uint64 observer_reset_count
uint64 observer_active_run_length
uint64 observer_revalidation_latency_updates

float64 observer_hidden_state
float64 observer_predicted_hidden_state
float64 observer_predicted_velocity
float64 observer_velocity_innovation
float64 observer_predicted_angular_velocity
float64 observer_angular_velocity_innovation
float64 observer_predicted_leg_psi
float64 observer_leg_psi_innovation
float64 observer_input_command_velocity
float64 observer_input_command_angular_velocity

# ==================== MPC 求解状态 ====================

# false 时，本消息其余 mpc_* 和 ancillary_* 字段无效或为空。
bool mpc_attempted

# MPC 求解模式
# 0: NONE, 1: FOLLOW, 2: STOP, 3: HOLD
uint8 mpc_mode

# 仅在 mpc_attempted=true 时有效。失败时 mpc_error 给出原因，预测数组为空。
bool mpc_succeeded
string mpc_error
float64 mpc_solve_time_ms                     # 本次调用的完整 MPC 求解耗时 (ms)

# ==================== Ancillary feedback ====================

bool ancillary_enabled        # 配置是否开启 ancillary feedback
bool ancillary_active         # 本周期是否实际启用，要求观测器已经通过验证
bool ancillary_reanchored     # 名义纵向状态是否在本周期重锚到实测状态
bool ancillary_tube_feasible  # 首拍修正是否满足命令 tube 约束，仅 active=true 时有效

# ==================== MPC 当前状态与首拍控制 ====================

float64 previous_command_velocity             # 求解前的线速度命令 (m/s)
float64 previous_command_angular_velocity     # 求解前的角速度命令 (rad/s)

# ancillary 修正前的 FDDP 名义首拍命令及其变化率。
float64 mpc_nominal_command_velocity                  # (m/s)
float64 mpc_nominal_command_angular_velocity          # (rad/s)
float64 mpc_nominal_command_velocity_rate             # (m/s^2)
float64 mpc_nominal_command_angular_velocity_rate     # (rad/s^2)

# ancillary 修正后的首拍变化率；修正后的命令就是上面的 command_*。
float64 mpc_applied_command_velocity_rate             # (m/s^2)
float64 mpc_applied_command_angular_velocity_rate     # (rad/s^2)

# ==================== FOLLOW 参考序列 ====================

# 以下数组仅对成功的 FOLLOW 求解有效，长度恒为 MPC_HORIZON。
# 每个下标 k 对应 running-cost stage k。
float64[] mpc_reference_path_progress             # stage k 的累计弧长 s_k (m)
float64[] mpc_reference_path_speed                # MPC 优化的 PATH_SPEED_CMD，即 ds/dt (m/s)
float64[] trajectory_nominal_velocity             # MINCO 在 s_k 处的名义带符号线速度 (m/s)
float64[] trajectory_nominal_angular_velocity     # MINCO 在 s_k 处的名义角速度 (rad/s)
float64[] mpc_reference_velocity                  # MPC 代价函数实际跟踪的线速度 (m/s)
float64[] mpc_reference_angular_velocity          # MPC 代价函数实际跟踪的角速度 (rad/s)

# ==================== MPC 状态预测 ====================

# 以下数组对成功的 FOLLOW、STOP、HOLD 求解有效，长度恒为 MPC_HORIZON+1。
# 下标 0 是 rollout 初始状态。
float64[] mpc_nominal_prediction_velocity          # FDDP 名义模型预测线速度 (m/s)
float64[] mpc_nominal_prediction_angular_velocity  # FDDP 名义模型预测角速度 (rad/s)
float64[] mpc_applied_prediction_velocity          # ancillary 应用后预测线速度 (m/s)
float64[] mpc_applied_prediction_angular_velocity  # ancillary 应用后预测角速度 (rad/s)

# ==================== FDDP 终止情况 ====================

# 不参与验收判定；实时 MPC 在 max_iters 内通常不会真正收敛，这些字段只用于
# 离线判断求解质量与终止原因。仅在 mpc_succeeded=true 时有效。
int32 mpc_solver_iters        # 实际执行的迭代数
bool mpc_solver_converged     # 是否达到投影梯度与 gap 容差
float64 mpc_solver_cost       # 最终 rollout 代价

# ==================== 被拒绝的 FOLLOW rollout ====================

# FOLLOW rollout 命中致命障碍时该 rollout 一次也不下发，本消息其余字段描述的是
# 实际发布的 STOP 求解。以下字段保留被拒绝的那条 rollout，用于离线定位。
bool rejected_follow_rollout                      # 本周期是否拒绝了一条 FOLLOW rollout
int32 rejected_lethal_state_index                 # 首个致命状态在 horizon 中的下标
float64 rejected_lethal_position_x                # 首个致命状态位置 (m, map)
float64 rejected_lethal_position_y                # 首个致命状态位置 (m, map)
float64 rejected_lethal_sampled_cost              # 该位置的静态插值代价 (0~255)
int32 rejected_consecutive_count                  # 连续拒绝帧数
bool rejected_replan_requested                    # 是否已达到阈值并上报 MPC_LETHAL
# 两个数组长度恒相等，成功拒绝时至多为 MPC_HORIZON+1。
float64[] rejected_prediction_x                   # 被拒绝 rollout 的位置序列 (m, map)
float64[] rejected_prediction_y                   # 被拒绝 rollout 的位置序列 (m, map)
