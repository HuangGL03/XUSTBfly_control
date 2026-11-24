/**
  ********************************************************
  * @file       mavlink_flow.c
  * @brief      光流/测距端口(USART2 / COMM_1)派发逻辑。
  *             用官方 decode 解析,v1/v2 帧同步/载荷零截断/CRC 由库自动处理。
  * @note       忽略 DISTANCE_SENSOR 的 horizontal_fov / vertical_fov(协议文档
  *             说明:发送端未初始化的脏栈数据,不可使用)。
  ********************************************************
  */
#include "mavlink_flow.h"
#include "main.h"   /* HAL_GetTick */
#include "INS_task.h"
#include "flow_config.h"
#include "optical_flow.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

static OpticalFlow_Data_t flow_data = {
    .vx_raw_mps = NAN, .vy_raw_mps = NAN,
    .vx_body_mps = NAN, .vy_body_mps = NAN,
    .height_raw_m = NAN, .height_m = NAN
};

/* 两轴滤波状态只由光流接收任务更新，读取快照不会推进滤波器。 */
static struct {
    float vx_mps, vy_mps;  /* 上一帧的滤波输出，单位 m/s */
    uint32_t update_ms;    /* 上一次有效光流帧的接收时间，单位 ms */
    bool initialized;     /* 首帧、无效帧或断流后需重新初始化 */
} velocity_filter;

/* 106 和 132 报文更新同一个高度滤波器，按实际时间间隔分配权重。 */
static struct {
    float height_m;       /* 上一次滤波后的离地高度，单位 m */
    uint32_t update_ms;   /* 上一次有效高度更新的系统时间，单位 ms */
    bool initialized;    /* 无效观测或断流后重新跟随测量初始化 */
} height_filter;

/* 在旋转补偿之后滤波，保持原始角位移与本帧积分时间的对应关系。 */
static void filter_velocity(OpticalFlow_Data_t *next, const float velocity[2], uint32_t now)
{
    next->vx_raw_mps = velocity[0];
    next->vy_raw_mps = velocity[1];
    if (!next->velocity_valid || !isfinite(velocity[0]) || !isfinite(velocity[1])) {
        velocity_filter.initialized = false;
        next->vx_body_mps = next->vy_body_mps = NAN;
        return;
    }

    uint32_t elapsed_ms = (uint32_t)(now - velocity_filter.update_ms);
    if (!velocity_filter.initialized || elapsed_ms > FLOW_TIMEOUT_MS ||
        FLOW_XY_FILTER_TAU_S <= 0.0f) {
        /* 首帧直接跟随测量，避免从零缓慢爬升；恢复时不混入断流前的旧速度。 */
        velocity_filter.vx_mps = velocity[0];
        velocity_filter.vy_mps = velocity[1];
    } else {
        /* 按新帧间隔更新；同一毫秒内解析的缓存帧使用各自积分时间。 */
        float dt_s = elapsed_ms != 0U ? elapsed_ms * 0.001f : next->integration_time_s;
        float alpha = dt_s / (FLOW_XY_FILTER_TAU_S + dt_s); /* 当前测量的权重 */
        velocity_filter.vx_mps += alpha * (velocity[0] - velocity_filter.vx_mps);
        velocity_filter.vy_mps += alpha * (velocity[1] - velocity_filter.vy_mps);
    }
    velocity_filter.initialized = true;
    velocity_filter.update_ms = now;
    next->vx_body_mps = velocity_filter.vx_mps;
    next->vy_body_mps = velocity_filter.vy_mps;
}

void mavlink_flow_publish_control(const OpticalFlow_Control_t *control)
{
    taskENTER_CRITICAL();
    flow_data.control = *control;
    taskEXIT_CRITICAL();
}

const OpticalFlow_Data_t *get_optical_flow_point(void)
{
    return &flow_data;
}

bool mavlink_flow_get_snapshot(OpticalFlow_Data_t *out, uint32_t now)
{
    if (out == NULL) return false;
    taskENTER_CRITICAL();
    *out = flow_data;
    taskEXIT_CRITICAL();
    out->online = out->online && (uint32_t)(now - out->last_update_ms) <= FLOW_TIMEOUT_MS;
    if ((uint32_t)(now - out->flow_update_ms) > FLOW_TIMEOUT_MS) {
        out->vx_raw_mps = out->vy_raw_mps = NAN;
        out->vx_body_mps = out->vy_body_mps = NAN;
        out->velocity_valid = 0;
    }
    out->height_valid = out->height_valid &&
        (uint32_t)(now - out->height_update_ms) <= FLOW_TIMEOUT_MS;
    if (!out->height_valid) out->height_raw_m = out->height_m = NAN;
    return out->flow_frame_count != 0U || out->range_frame_count != 0U;
}

static void update_height(OpticalFlow_Data_t *next, float distance,
                          const FlowAttitude_t *imu, bool attitude_valid, uint32_t now)
{
    next->height_raw_m = NAN;
    next->height_update_ms = now;
    next->height_valid = attitude_valid && optical_flow_height(distance,
        imu->roll_rad,
        imu->pitch_rad, &next->height_raw_m);
    if (!next->height_valid) {
        height_filter.initialized = false;
        next->height_m = NAN;
        return;
    }

    uint32_t elapsed_ms = (uint32_t)(now - height_filter.update_ms);
    if (!height_filter.initialized || elapsed_ms > FLOW_TIMEOUT_MS ||
        FLOW_HEIGHT_FILTER_TAU_S <= 0.0f) {
        height_filter.height_m = next->height_raw_m;
    } else {
        /* 两种报文可能报告同一测距样本；同毫秒报文权重为零，避免重复推进。 */
        float dt_s = elapsed_ms * 0.001f; /* 两次有效高度更新的实际间隔，单位 s */
        float alpha = dt_s / (FLOW_HEIGHT_FILTER_TAU_S + dt_s);
        height_filter.height_m += alpha * (next->height_raw_m - height_filter.height_m);
    }
    height_filter.initialized = true;
    height_filter.update_ms = now;
    next->height_m = height_filter.height_m;
}

/* 只用 106 帧自身的旋转中心高度估计升降速度，避免与 132 帧交错差分。
 * 假设观测地面局部平坦；首次有效帧及断流恢复时以零升降速度起步。 */
static float flow_climb_rate(const OpticalFlow_Data_t *sample, uint32_t now)
{
    static bool initialized;
    static float previous_height_m, climb_mps;
    static uint32_t previous_ms;
    uint32_t elapsed_ms = now - previous_ms;
    if (!sample->height_valid) {
        initialized = false;
        return 0.0f;
    }
    if (!initialized || elapsed_ms > FLOW_TIMEOUT_MS) {
        climb_mps = 0.0f;
    } else if (elapsed_ms >= FLOW_RUN_VZ_SAMPLE_MIN_MS) {
        float dt = elapsed_ms * 0.001f;
        float measured = (sample->height_raw_m - previous_height_m) / dt;
        climb_mps += dt / (FLOW_RUN_VZ_FILTER_TAU_S + dt) * (measured - climb_mps);
    } else {
        return climb_mps;
    }
    initialized = true;
    previous_height_m = sample->height_raw_m;
    previous_ms = now;
    return climb_mps;
}

/* 光流/ToF 采集处理
 * 整帧解码后填局部 next，再原子发布原始观测及估计；供遥测和飞控任务使用。
 * 光流帧自身的 distance 与该帧配套；不借用可能陈旧的 DISTANCE_SENSOR 距离。
 * 旋转补偿使用接收时 INS 角速度近似，不读取发送端为 NaN 的 gyro 字段。 */
void mavlink_flow_dispatch(const mavlink_message_t *msg)
{
    if (msg == NULL || (msg->msgid != MAVLINK_MSG_ID_OPTICAL_FLOW_RAD &&
                        msg->msgid != MAVLINK_MSG_ID_DISTANCE_SENSOR)) return;
    FlowAttitude_t imu;
    bool attitude_valid = ins_get_flow_attitude(&imu);
    /* 先复制后取当前时间，避免被高优先级 INS 抢占后快照时间晚于 now。 */
    uint32_t now = HAL_GetTick();
    attitude_valid = attitude_valid &&
        (uint32_t)(now - imu.update_ms) <= FLOW_IMU_TIMEOUT_MS;
    OpticalFlow_Data_t next;
    (void)mavlink_flow_get_snapshot(&next, now);
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:   /* 106 */
    {
        mavlink_optical_flow_rad_t flow;
        mavlink_msg_optical_flow_rad_decode(msg, &flow);
        next.flow_x_rad         = flow.integrated_x;
        next.flow_y_rad         = flow.integrated_y;
        next.integration_time_s = (float)flow.integration_time_us / 1.0e6f;
        next.ground_distance_m  = flow.distance;
        next.flow_quality       = flow.quality;
        next.flow_update_ms = now;
        next.flow_frame_count++;
        update_height(&next, flow.distance, &imu, attitude_valid, now);
        float integrated[2] = {flow.integrated_x, flow.integrated_y};
        float velocity[2] = {NAN, NAN};
        float climb_mps = flow_climb_rate(&next, now);
        next.velocity_valid = next.height_valid &&
            optical_flow_velocity(integrated, next.integration_time_s, flow.distance,
                                  imu.gyro_rads, imu.roll_rad, imu.pitch_rad, climb_mps, velocity);
        filter_velocity(&next, velocity, now);
        break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:    /* 132 */
    {
        mavlink_distance_sensor_t dist;
        mavlink_msg_distance_sensor_decode(msg, &dist);
        next.range_cm = dist.current_distance;  /* 单位:厘米 */
        next.range_update_ms = now;
        next.range_frame_count++;
        /* 协议中的 max_distance=4000 与“4米”矛盾，不将其作为可信量程。
         * 实际工作范围由 FLOW_MIN/MAX_DISTANCE_M 控制，向下安装已由用户确认。 */
        update_height(&next, dist.current_distance * 0.01f, &imu, attitude_valid, now);
        break;
    }

    default:
        break;
    }
    next.last_update_ms = now;
    next.online = 1U;
    taskENTER_CRITICAL();
    /* 解码期间飞控可能已发布新输出，接收任务只更新传感器数据。 */
    next.control = flow_data.control;
    flow_data = next;
    taskEXIT_CRITICAL();
}
