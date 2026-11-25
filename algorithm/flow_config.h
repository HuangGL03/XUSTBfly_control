#pragma once

/* QGC 速度环调试：1 发送期望与实测速度，0 恢复上位机 yaw 遥测。 */
#define MAVLINK_QGC_FLOW_DEBUG_ENABLE 1
#define MAVLINK_QGC_FLOW_DEBUG_PERIOD_MS 100U

/* B=1、C=0 光流控速模式调参。高度向上为正，水平速度沿机体前、右为正。
 * 定高先看第 1、2 组；第 5、6 组用于核对安装方向和观测有效性。 */

/* 1. 定高与升降控制 */
#define FLOW_RUN_HEIGHT_ENABLE    1      /* 0：沿用普通 RUN 手动油门，仅调 X/Y；1：启用光流定高 */
/* 单高度环：P 作用于高度误差，I 按秒积分，D 使用滤波后的升降速度。
 * 积分增量限速，且输出限幅或限速时只允许积分减轻受限程度。 */
#define FLOW_RUN_HEIGHT_KP      68.0f   /* 推力指令/m */
#define FLOW_RUN_HEIGHT_KI       3.0f   /* 推力指令/(m*s)，实飞调试初值 */
#define FLOW_RUN_HEIGHT_I_SLEW   3.5f   /* 高度积分项每秒最大变化量，单位为推力指令 */
#define FLOW_RUN_HEIGHT_KD      20.0f   /* 推力指令/(m/s) */
#define FLOW_RUN_TARGET_HEIGHT_M  1.0f   /* 光流定高的旋转中心离地目标，单位 m */
#define FLOW_RUN_THRUST_MAX     340.0f   /* 垂直控制的基础推力上限，单位为混控前的推力指令 */
#define FLOW_RUN_THRUST_SLEW    250.0f   /* 基础推力每秒最大变化量，同时限制上升和下降 */
#define FLOW_RUN_MANUAL_SCALE     3.40f  /* 测距失效时，每 1 个油门杆量对应的手动推力指令 */

/* 2. 高度差分测速与控制周期 */
#define FLOW_RUN_VZ_SAMPLE_MIN_MS    15U   /* 两次参与高度差分的最短间隔，单位 ms */
#define FLOW_RUN_VZ_FILTER_TAU_S     0.08f /* 升降速度一阶低通时间常数，单位 s；增大后更平滑、滞后更大 */
#define FLOW_RUN_DT_MAX_MS           50U   /* 垂直控制单步 dt 上限，单位 ms，用于高度 PID 积分和推力限速 */

/* 3. 光流水平控制 */
#define FLOW_RUN_XY_VELOCITY_ENABLE 0     /* 0：直接使用目标姿态；1：使用水平速度环 */
#define FLOW_PC_TARGET_TIMEOUT_MS 1500U   /* 上位机 ID 248 指令有效期，超时给定零 */
#define FLOW_RUN_ATTITUDE_MAX_DEG 30.0f    /* 直接姿态控制时的单轴目标角限幅 */
#define FLOW_RUN_XY_MAX_MPS       3.0f   /* 水平满杆的单轴期望速度，单位 m/s */
#define FLOW_RUN_VX_FEEDBACK_SCALE 3.5f   /* 前向滤波速度进入速度 PID 前的倍率 */
#define FLOW_RUN_VY_FEEDBACK_SCALE 4.0f   /* 右向滤波速度进入速度 PID 前的倍率 */
/* 两轴各有独立状态与参数。Ki、Kd 沿用本工程 PID_calc 的每控制步约定。 */
#define FLOW_RUN_VX_KP    4.6f   /* 前向速度误差 -> 俯仰角，度/(m/s) */
#define FLOW_RUN_VX_KI    0.0f
#define FLOW_RUN_VX_KD    1.0f
#define FLOW_RUN_VY_KP    5.6f   /* 右向速度误差 -> 横滚角，取反后送姿态环 */
#define FLOW_RUN_VY_KI    0.0f
#define FLOW_RUN_VY_KD    1.0f
#define FLOW_RUN_XY_IOUT_MAX_DEG 12.0f   /* 仅限制 X/Y 积分项幅值，单位度 */

/* 4. 本模式的水平和偏航摇杆 */
#define FLOW_RUN_STICK_DEADBAND   5      /* 俯仰、横滚、偏航杆的中位死区半宽，杆量范围 -100~100 */
#define FLOW_RUN_YAW_SCALE        0.004f /* 每个控制周期、每单位偏航杆量对应的航向目标变化量，单位度 */

/* 5. 光流安装方向与比例校正；0 取原始 X 分量，1 取原始 Y 分量 */
#define FLOW_BODY_X_AXIS 0  /* 用于机体 X 角轴的原始光流分量索引，0 为 X，1 为 Y */
#define FLOW_BODY_Y_AXIS 1  /* 用于机体 Y 角轴的原始光流分量索引 */
#define FLOW_BODY_X_SCALE (1.0f)  /* X 角轴的符号与比例校正，负号表示反向 */
#define FLOW_BODY_Y_SCALE (1.0f)  /* Y 角轴的符号与比例校正 */
#define FLOW_SENSOR_OFFSET_Z_M 0.055f /* 旋转中心到测距探头沿机体 +Z（向下）的距离 */

/* 6. 观测有效性门限 */
#define FLOW_TIMEOUT_MS 100U  /* 观测有效期，超过该时间后清除对应的快照有效位 */
#define FLOW_MIN_DISTANCE_M 0.08f  /* 有效斜距下限，单位 m */
#define FLOW_MAX_DISTANCE_M 4.0f  /* 有效斜距上限，单位 m */
#define FLOW_MIN_INTEGRATION_S 0.005f  /* 有效光流积分时间下限，单位 s */
#define FLOW_MAX_INTEGRATION_S 0.05f  /* 有效光流积分时间上限，单位 s */
#define FLOW_MIN_TILT_COS 0.5f /* 合成倾角上限 60°，cos(60°) = 0.5 */
#define FLOW_MAX_SPEED_MPS 10.0f  /* 速度估计的单轴绝对值上限，超过即判为无效，单位 m/s */
#define FLOW_IMU_TIMEOUT_MS 10U  /* 姿态快照允许的最大年龄，单位 ms */

/* 7. 光流速度与高度的一阶低通。飞控闭环使用滤波结果。
 * 公式：输出 += alpha * (本次测量 - 上次输出)，alpha = dt / (tau + dt)。
 * tau 单位为秒，设为 0 可关闭对应滤波；首帧及断流恢复首帧直接使用测量值。
 * 调试时先保持 PID 不变，对比 aircraft_control.flow_data 中的：
 *   vx_raw_mps / vy_raw_mps 与 vx_body_mps / vy_body_mps（滤波前 / 后速度）；
 *   height_raw_m 与 height_m（倾角补偿后、滤波前 / 后高度）。
 * 数值抖动明显时可小步增加 tau；回中制动变迟或高度响应拖延时应减小。
 * 更大的 tau 会增加反馈延迟，不能用持续加大滤波来消除控制振荡。
 * 高度滤波与第 2 组升降速度滤波会叠加延迟，后续启用定高时需一起观察。
 * 原始光流角位移、质量及原始斜距仍保留，便于检查传感器本身的数据。 */
#define FLOW_XY_FILTER_TAU_S     0.07f /* 水平速度滤波时间常数；20 ms 帧间隔时 alpha=0.5 */
#define FLOW_HEIGHT_FILTER_TAU_S 0.05f /* 高度滤波时间常数；20 ms 更新间隔时 alpha=0.4 */
