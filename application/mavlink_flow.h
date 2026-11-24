/**
  ********************************************************
  * @file       mavlink_flow.h
  * @brief      光流+测距 MAVLink 端口(USART2 / MAVLINK_COMM_1)的数据与派发。
  *             接收 MAVLink v1/v2 OPTICAL_FLOW_RAD(106) 与 DISTANCE_SENSOR(132),
  *             输入周期由模块实际输出决定。
  ********************************************************
  */
#ifndef __MAVLINK_FLOW_H__
#define __MAVLINK_FLOW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "common/mavlink.h"

/* 最近一次完成的飞控周期。PID 未参与时对应输出为 0；CCR 是写入定时器的计数值。 */
typedef struct
{
    uint32_t update_ms;
    uint8_t flow_mode_active;
    uint8_t xy_pid_active;
    uint8_t height_pid_active;
    float vx_set_mps, vy_set_mps;             /* 速度环实际使用的期望速度 */
    float vx_unclamped_deg, vy_unclamped_deg; /* Pout + Iout + Dout，输出限幅前 */
    float vx_pid_out_deg, vy_pid_out_deg;     /* 送往姿态环的速度 PID 输出，Y 轴尚未取反 */
    float pitch_target_deg, roll_target_deg;  /* 姿态环实际接收的目标角 */
    float pitch_rate_out, roll_rate_out;      /* 角速度内环输出，进入舵机/电机混控 */
    float height_pid_out;                     /* 高度 PID 原始推力输出，推力限速前 */
    uint16_t motor_left_ccr, motor_right_ccr; /* TIM3 CH4 / CH3 */
    uint16_t servo_left_ccr, servo_right_ccr; /* TIM5 CH2 / CH1 */
} OpticalFlow_Control_t;

typedef struct
{
    /* 来自 OPTICAL_FLOW_RAD(106) */
    float    flow_x_rad;        /* integrated_x:X 累计光流角位移(弧度) */
    float    flow_y_rad;        /* integrated_y:Y 累计光流角位移(弧度) */
    float    integration_time_s;/* 积分时间(秒),由 integration_time_us 换算 */
    float    ground_distance_m; /* 106 distance:沿下视光轴的原始斜距(米) */
    uint8_t  flow_quality;      /* 模块原始质量 0~255，仅供观察，0 表示模块未确认有效光流 */

    /* 来自 DISTANCE_SENSOR(132) */
    uint16_t range_cm;          /* 132 current_distance:原始斜距(厘米) */

    /* 链路健康 */
    uint32_t last_update_ms;    /* 最近一次收到任意一帧的 HAL_GetTick() */
    uint8_t  online;            /* 快照中为 1 表示近期收到过光流或测距报文 */

    uint32_t flow_update_ms;    /* 两种报文独立计时，测距不能掩盖光流断流 */
    uint32_t range_update_ms;  /* 最近一次收到测距报文的系统时间戳，单位 ms */
    uint32_t flow_frame_count;  /* 累计收到的光流报文数 */
    uint32_t range_frame_count;  /* 累计收到的测距报文数 */
    float vx_raw_mps;          /* 完成旋转和倾斜补偿、滤波前的水平前向速度，单位 m/s */
    float vy_raw_mps;          /* 完成旋转和倾斜补偿、滤波前的水平右向速度，单位 m/s */
    float vx_body_mps;         /* 低通后的水平前向速度，沿当前航向，供速度环和遥测使用，单位 m/s */
    float vy_body_mps;         /* 低通后的水平右向速度，单位 m/s */
    float height_raw_m;        /* 滤波前的旋转中心离地高度，单位 m */
    float height_m;            /* 滤波后的旋转中心离地高度，向上为正，供控制和遥测使用，单位 m */
    uint32_t height_update_ms;  /* 最近一次用斜距更新高度估计的时间戳，单位 ms */
    uint8_t velocity_valid;  /* 表示本帧已算出两轴速度且未超时，不代表模块质量合格 */
    uint8_t height_valid;  /* 为 1 时可使用本快照内的离地高度，与速度有效位独立 */
    OpticalFlow_Control_t control; /* 由飞控任务在执行器输出之后发布 */
} OpticalFlow_Data_t;

/**
  * @brief  兼容旧接口；不保证跨任务快照一致性，也不主动检查超时。
  *         新代码使用 mavlink_flow_get_snapshot。
  */
const OpticalFlow_Data_t *get_optical_flow_point(void);

/* 任务上下文原子复制，按 now_ms 清除超时有效位并将无效结果置 NaN。
 * 返回是否至少收到过一条支持的消息；数值是否可用应检查各自 valid。 */
bool mavlink_flow_get_snapshot(OpticalFlow_Data_t *out, uint32_t now_ms);

/**
  * @brief  COMM_1(USART2)整帧派发:OPTICAL_FLOW_RAD / DISTANCE_SENSOR。
  *         作为 MavlinkPort_t.on_message 回调注册。
  */
void mavlink_flow_dispatch(const mavlink_message_t *msg);

/* 仅由飞控任务调用；与串口接收任务分别更新控制数据和观测数据。 */
void mavlink_flow_publish_control(const OpticalFlow_Control_t *control);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_FLOW_H__ */
