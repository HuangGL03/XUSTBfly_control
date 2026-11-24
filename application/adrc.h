#ifndef __ADRC_H
#define __ADRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 一阶线性 ADRC / LADRC 控制器
 *
 * 适用对象：
 *   角速度内环：rate_dot = f + b0 * u
 *
 * 说明：
 *   z1 估计被控量，例如角速度 gyro/rate
 *   z2 估计总扰动，例如气动扰动、模型误差、耦合项
 *   out 为控制器输出，单位由执行机构决定
 *
 * 控制律：
 *   u = (kp * (ref - z1) - z2) / b0
 *
 * 注意：
 *   b0 是被控对象控制增益估计值。
 *   若控制方向反了，可以把 b0 改成负值，或在调用处反号。
 */
typedef struct
{
    float h;          /* 控制周期，单位 s，例如 0.001f */
    float b0;         /* 控制增益估计值，不能为 0 */
    float kp;         /* 状态误差反馈增益 */
    float wo;         /* ESO 观测器带宽，单位 rad/s */

    float beta1;      /* ESO 参数：2 * wo */
    float beta2;      /* ESO 参数：wo * wo */

    float z1;         /* ESO 状态 1：被控量估计值 */
    float z2;         /* ESO 状态 2：总扰动估计值 */

    float err;        /* ref - z1 */
    float out;        /* 控制输出 */
    float out_limit;  /* 输出限幅，正负对称 */
} adrc_1st_t;

void ADRC_1st_Init(adrc_1st_t *adrc,
                   float h,
                   float b0,
                   float kp,
                   float wo,
                   float out_limit);

void ADRC_1st_Reset(adrc_1st_t *adrc, float feedback, float out_init);

float ADRC_1st_Calc(adrc_1st_t *adrc, float ref, float feedback);

void ADRC_1st_SetKp(adrc_1st_t *adrc, float kp);
void ADRC_1st_SetB0(adrc_1st_t *adrc, float b0);
void ADRC_1st_SetWo(adrc_1st_t *adrc, float wo);
void ADRC_1st_SetOutputLimit(adrc_1st_t *adrc, float out_limit);

#ifdef __cplusplus
}
#endif

#endif /* __ADRC_H */
