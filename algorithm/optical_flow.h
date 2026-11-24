#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdbool.h>

/* 下视、光轴沿机体 +Z。计入安装偏移并投影，得到旋转中心离地高度。 */
bool optical_flow_height(float distance_m, float roll_rad, float pitch_rad, float *height_m);

/* 旋转中心的水平速度，沿当前航向的前/右，单位 m/s。
 * distance 为光轴斜距，climb 为旋转中心向上为正的升降速度。
 * gyro 为左滚、前倾为正的角速度 rad/s，当前用接收时 INS 快照近似积分窗口均值。 */
bool optical_flow_velocity(const float integrated_rad[2], float integration_s,
                           float distance_m, const float gyro_rads[2], float roll_rad,
                           float pitch_rad, float climb_mps, float velocity_mps[2]);

#endif
