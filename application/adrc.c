#include "adrc.h"

#define ADRC_EPSILON        0.000001f
#define ADRC_WO_MIN         0.1f
#define ADRC_OUT_LIMIT_MIN  0.0f

static float ADRC_Limit(float x, float min, float max)
{
    if (x > max)
    {
        return max;
    }

    if (x < min)
    {
        return min;
    }

    return x;
}

static float ADRC_Abs(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

static void ADRC_1st_UpdateESOParam(adrc_1st_t *adrc)
{
    if (adrc == 0)
    {
        return;
    }

    if (adrc->wo < ADRC_WO_MIN)
    {
        adrc->wo = ADRC_WO_MIN;
    }

    adrc->beta1 = 2.0f * adrc->wo;
    adrc->beta2 = adrc->wo * adrc->wo;
}

void ADRC_1st_Init(adrc_1st_t *adrc,
                   float h,
                   float b0,
                   float kp,
                   float wo,
                   float out_limit)
{
    if (adrc == 0)
    {
        return;
    }

    adrc->h = h;
    if (adrc->h <= 0.0f)
    {
        adrc->h = 0.001f;
    }

    adrc->b0 = b0;
    if (ADRC_Abs(adrc->b0) < ADRC_EPSILON)
    {
        adrc->b0 = 1.0f;
    }

    adrc->kp = kp;
    if (adrc->kp < 0.0f)
    {
        adrc->kp = 0.0f;
    }

    adrc->wo = wo;

    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->err = 0.0f;
    adrc->out = 0.0f;

    adrc->out_limit = out_limit;
    if (adrc->out_limit < ADRC_OUT_LIMIT_MIN)
    {
        adrc->out_limit = ADRC_OUT_LIMIT_MIN;
    }

    ADRC_1st_UpdateESOParam(adrc);
}

void ADRC_1st_Reset(adrc_1st_t *adrc, float feedback, float out_init)
{
    if (adrc == 0)
    {
        return;
    }

    adrc->z1 = feedback;
    adrc->z2 = 0.0f;
    adrc->err = 0.0f;
    adrc->out = ADRC_Limit(out_init, -adrc->out_limit, adrc->out_limit);
}

float ADRC_1st_Calc(adrc_1st_t *adrc, float ref, float feedback)
{
    float e;
    float u0;
    float u;

    if (adrc == 0)
    {
        return 0.0f;
    }

    /*
     * ESO 使用上一周期 out 更新状态。
     * 一阶扩张状态观测器：
     *   z1_dot = z2 + b0 * u + beta1 * (y - z1)
     *   z2_dot = beta2 * (y - z1)
     */
    e = feedback - adrc->z1;

    adrc->z1 += adrc->h *
                (adrc->z2 + adrc->b0 * adrc->out + adrc->beta1 * e);

    adrc->z2 += adrc->h *
                (adrc->beta2 * e);

    /*
     * 状态误差反馈 + 扰动补偿：
     *   u = (kp * (ref - z1) - z2) / b0
     */
    adrc->err = ref - adrc->z1;
    u0 = adrc->kp * adrc->err;
    u = (u0 - adrc->z2) / adrc->b0;

    adrc->out = ADRC_Limit(u, -adrc->out_limit, adrc->out_limit);

    return adrc->out;
}

void ADRC_1st_SetKp(adrc_1st_t *adrc, float kp)
{
    if (adrc == 0)
    {
        return;
    }

    if (kp < 0.0f)
    {
        kp = 0.0f;
    }

    adrc->kp = kp;
}

void ADRC_1st_SetB0(adrc_1st_t *adrc, float b0)
{
    if (adrc == 0)
    {
        return;
    }

    if (ADRC_Abs(b0) < ADRC_EPSILON)
    {
        return;
    }

    adrc->b0 = b0;
}

void ADRC_1st_SetWo(adrc_1st_t *adrc, float wo)
{
    if (adrc == 0)
    {
        return;
    }

    adrc->wo = wo;
    ADRC_1st_UpdateESOParam(adrc);
}

void ADRC_1st_SetOutputLimit(adrc_1st_t *adrc, float out_limit)
{
    if (adrc == 0)
    {
        return;
    }

    if (out_limit < ADRC_OUT_LIMIT_MIN)
    {
        out_limit = ADRC_OUT_LIMIT_MIN;
    }

    adrc->out_limit = out_limit;
    adrc->out = ADRC_Limit(adrc->out, -adrc->out_limit, adrc->out_limit);
}
