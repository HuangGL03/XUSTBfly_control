#include "optical_flow.h"
#include "flow_config.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

static bool valid_distance(float distance)
{
    return isfinite(distance) && distance >= FLOW_MIN_DISTANCE_M && distance <= FLOW_MAX_DISTANCE_M;
}

static bool valid_tilt(float tilt_cos)
{
    /* 允许临界角换算为单精度弧度时产生的舍入误差。 */
    return tilt_cos >= FLOW_MIN_TILT_COS - FLT_EPSILON;
}

bool optical_flow_height(float distance, float roll, float pitch, float *height)
{
    if (height == NULL) return false;
    *height = NAN;
    if (!valid_distance(distance) || !isfinite(roll) || !isfinite(pitch)) return false;
    float tilt_cos = cosf(roll) * cosf(pitch);
    if (!valid_tilt(tilt_cos)) return false;
    *height = (distance + FLOW_SENSOR_OFFSET_Z_M) * tilt_cos;
    return true;
}

bool optical_flow_velocity(const float integrated[2], float dt, float distance,
                           const float gyro[2], float roll, float pitch, float climb_mps,
                           float velocity[2])
{
    if (velocity == NULL) return false;
    velocity[0] = velocity[1] = NAN;
    if (integrated == NULL || gyro == NULL || !valid_distance(distance) ||
        !isfinite(dt) || dt < FLOW_MIN_INTEGRATION_S || dt > FLOW_MAX_INTEGRATION_S ||
        !isfinite(integrated[0]) || !isfinite(integrated[1]) ||
        !isfinite(gyro[0]) || !isfinite(gyro[1]) || !isfinite(climb_mps) ||
        !isfinite(roll) || !isfinite(pitch)) return false;

    float cr = cosf(roll), cp = cosf(pitch);
    if (cr <= 0.0f || cp <= 0.0f || !valid_tilt(cr * cp)) return false;

    /* MAVLink OPTICAL_FLOW_RAD 的角轴约定：前移 +flow_y，右移 -flow_x。
     * 先映射/标定原始光流，再减 gyro；发送端 gyro 为 NaN，使用本机 INS。 */
    float flow_x = integrated[FLOW_BODY_X_AXIS] * FLOW_BODY_X_SCALE / dt - gyro[0];
    float flow_y = integrated[FLOW_BODY_Y_AXIS] * FLOW_BODY_Y_SCALE / dt - gyro[1];
    /* 从探头速度中扣除 omega × r。工程角速度左滚、前倾为正，
     * 转到前右下机体系后 X/Y 均取反，杆臂 r 沿 +Z。 */
    float body_x = flow_y * distance + gyro[1] * FLOW_SENSOR_OFFSET_Z_M;
    float body_y = -flow_x * distance - gyro[0] * FLOW_SENSOR_OFFSET_Z_M;
    /* 以航向为水平前方，右方与之正交。工程姿态前倾、左滚为正，爬升为正。
     * 先扣除升降在机体测量轴上的投影，再解出两个水平分量。 */
    float sp = sinf(pitch), sr = sinf(roll);
    float vx = (body_x + sp * climb_mps) / cp;
    float vy = (body_y - sr * (sp * vx + cp * climb_mps)) / cr;
    if (!isfinite(vx) || !isfinite(vy) ||
        fabsf(vx) > FLOW_MAX_SPEED_MPS || fabsf(vy) > FLOW_MAX_SPEED_MPS) return false;
    velocity[0] = vx;
    velocity[1] = vy;
    return true;
}
