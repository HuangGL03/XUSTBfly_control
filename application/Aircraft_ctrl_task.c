 /**
 * @file    Aircraft_ctrl_task.c
 * @brief   飞行器核心控制任务
 * @note
 *
 * 本文件实现飞行器主控制任务，包括：
 * 1. 飞行模式状态机切换；
 * 2. 遥控 RUN 模式控制；
 * 3. PC Auto 模式控制；
 * 4. 姿态角外环 PID；
 * 5. 角速度内环 PID；
 * 6. 左右电机差速混控；
 * 7. 左右电机/翅翼不一致补偿；
 * 8. Roll 方向慢速自学习 trim；
 * 9. Pitch/Yaw 舵机混控；
 * 10. 电机与舵机 PWM 输出；
 * 11. 加入内环 ADRC 控制模式 2026.6.10；
 * 12. RUN 模式下 E=2 进入高速 RUN 模式。
 *
 * IMU方向：
 * roll  左+ 右-
 * pitch 前+ 后-
 * yaw   顺+ 逆-
 */

#include "Aircraft_ctrl_task.h"
#include "flow_config.h"
#include "cmsis_os.h"
#include "mavlink.h"
#include "tim.h"
#include "pid.h"
#include "adrc.h"
#include "INS_task.h"
#include "filter.h"
#include "spl06.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>


/* ========================================================== */
/* ================= 全局状态对象与外部传感器 =============== */
/* ========================================================== */

aircraft_control_t aircraft_control;

int set_yaw = 0;
int now_yaw = 0;

/* SPL06 气压计对象 */
extern SPL06_t spl06_inst;


/* ========================================================== */
/* ======================= PID / ADRC ======================= */
/* ========================================================== */

/* Pitch / Roll / Yaw 角度外环 PID */
pid_type_def PID_control_pitch;
pid_type_def PID_control_roll;
pid_type_def PID_control_yaw;

/* 高度 PID */
pid_type_def PID_control_altitude;

/* 角速度内环 PID */
pid_type_def PID_rate_pitch;
pid_type_def PID_rate_roll;
pid_type_def PID_rate_yaw;

/* 角速度内环 ADRC */
adrc_1st_t ADRC_rate_pitch;
adrc_1st_t ADRC_rate_roll;
adrc_1st_t ADRC_rate_yaw;


/* ========================================================== */
/* ======================= 滤波器对象 ======================= */
/* ========================================================== */

LPF2ndFilter gyro_pitch_lpf;
LPF2ndFilter gyro_roll_lpf;

LPF2ndFilter angle_pitch_lpf;
LPF2ndFilter angle_roll_lpf;

LPF2ndFilter acc_pitch_lpf;
LPF2ndFilter acc_roll_lpf;


/* ========================================================== */
/* ======================= 输出与偏置量 ===================== */
/* ========================================================== */

int32_t left_motor_pwm  = 1000;
int32_t right_motor_pwm = 1000;

float pitch_servo_angle = 900.0f;
float pitch_bias = 0.0f;


/* ========================================================== */
/* ======================= 油门与高度参数 =================== */
/* ========================================================== */

float base_throttle = 1000.0f;
float diff_scale = 0.3f;
float max_motor_diff = 200.0f;
float add_speed = 0.0f;
float altitude = 0.0f;
float altitude_bias = 0.0f;
float altitude_aim = 1.6f;
float altitude_motor = 0.0f;


/* ========================================================== */
/* ============ 左右电机 / 翅翼一致性补偿参数 =============== */
/* ========================================================== */

#define MOTOR_PWM_BASE          1000.0f

#define MOTOR_CMD_MIN           0.0f
#define MOTOR_CMD_MAX           900.0f


/* ---------------- Auto 模式 PC 指令保护限幅 ---------------- */

#define AUTO_ROLL_LIMIT_DEG     30.0f
#define AUTO_PITCH_LIMIT_DEG    30.0f
#define AUTO_YAW_LIMIT_DEG      180.0f

#define AUTO_THRUST_LIMIT_MIN   0.0f
#define AUTO_THRUST_LIMIT_MAX   1.0f


/* ========================================================== */
/* ================= RUN 遥控灵敏度参数 ===================== */
/* ========================================================== */

/*
 * 普通 RUN：
 * Right_X -> Roll 目标角
 * Right_Y -> Pitch 目标角
 * Left_X  -> Yaw 目标角变化速度
 */


#define RUN_THROTTLE_SCALE      3.40f  					  /* 普通 RUN 与手动油门光流模式共用的基础推力比例 */
#define RUN_ROLL_SCALE          0.18f
#define RUN_PITCH_SCALE         0.20f
#define RUN_YAW_SCALE           0.004f

/*
 * 高速 RUN： * 仅当： * aircraft_behaviour == Aircraft_RUN   * 且遥控器 E == 2 * 才启用。* 建议先使用这一组较温和参数试飞。
 */
#define RUN_FAST_ROLL_SCALE     0.65f
#define RUN_FAST_PITCH_SCALE    0.35f
#define RUN_FAST_YAW_SCALE      0.004f
                                                          /* 光流模式的可调参数统一在 algorithm/flow_config.h。 */
static pid_type_def PID_flow_vx;    					  /* 机体前向速度 -> 目标俯仰角 */
static pid_type_def PID_flow_vy;     					  /* 机体右向速度 -> 取反后作为目标横滚角 */
static pid_type_def PID_flow_height;  					  /* 离地高度 -> 基础推力，Iout 包含切入时的推力 */

static struct {
    bool tracking_height; 								  /* 已建立有效的高度跟踪基准；清零后在下一次有效测距时重建 */
    uint32_t tick_ms;                                     /* 上一次运行垂直控制的系统时刻，单位 ms */
    uint32_t sample_ms;                                   /* 上一次用于高度差分的观测时间戳，单位 ms */
    float previous_height_m;                              /* 上一次参与差分的离地高度，单位 m */
    float target_height_m;                                /* 光流定高的固定离地高度目标，单位 m */
    float vz_mps;                                         /* 低通滤波后的实测升降速度，向上为正，单位 m/s */
    float thrust;                                         /* 上次经过变化率限制后输出的基础推力指令 */
} flow_run;
/* ---------------- Roll 自学习补偿参数 ---------------- */

#define ROLL_TRIM_ENABLE        1
#define ROLL_TRIM_LIMIT         80.0f

float roll_trim_learn_rate = 0.00008f;
float roll_trim_error_lpf = 0.0f;

float left_motor_gain  = 1.00f;
float right_motor_gain = 1.00f;

float left_motor_bias  = 0.0f;
float right_motor_bias = 0.0f;

float roll_motor_trim = 0.0f;


/* ========================================================== */
/* ======================= 电机标定表 ======================= */
/* ========================================================== */

typedef struct
{
    float cmd;
    float pwm;
} motor_calib_point_t;


static const motor_calib_point_t left_motor_map[] =
{
    {0.0f,   0.0f},
    {100.0f, 110.0f},
    {200.0f, 200.0f},
    {300.0f, 300.0f},
    {400.0f, 400.0f},
    {500.0f, 500.0f},
    {600.0f, 600.0f},
    {700.0f, 700.0f},
    {800.0f, 800.0f},
    {900.0f, 900.0f},
};


static const motor_calib_point_t right_motor_map[] =
{
    {0.0f,   0.0f},
    {100.0f, 100.0f},
    {200.0f, 200.0f},
    {300.0f, 300.0f},
    {400.0f, 400.0f},
    {500.0f, 500.0f},
    {600.0f, 600.0f},
    {700.0f, 700.0f},
    {800.0f, 800.0f},
    {900.0f, 900.0f},
};


/* ========================================================== */
/* =============== PID 增益参数表 {Kp, Ki, Kd} ============== */
/* ========================================================== */
/***1***/
/* 角度外环 */
//fp32 PID_Roll[3]  = {3.2f, 0.0f, 0.0f};
//fp32 PID_Pitch[3] = {4.0f, 0.0f, 0.0f};
//fp32 PID_Yaw[3]   = {7.6f, 0.0f, 0.0f};

///* 角速度内环 */
//fp32 PID_Rate_Roll[3]  = {0.36f, 0.0f, 0.05f};
//fp32 PID_Rate_Pitch[3] = {0.36f, 0.0f, 0.002f};
//fp32 PID_Rate_Yaw[3]   = {0.48f, 0.0f, 0.0f};
/***3***/
///* 角度外环 */
fp32 PID_Roll[3]  = {3.2f, 0.0f, 0.0f};
fp32 PID_Pitch[3] = {4.5f, 0.0f, 0.0f};
fp32 PID_Yaw[3]   = {7.6f, 0.0f, 0.0f};

/* 角速度内环 */
fp32 PID_Rate_Roll[3]  = {0.36f, 0.0f, 0.05f};
fp32 PID_Rate_Pitch[3] = {0.40f, 0.0f, 0.002f};
fp32 PID_Rate_Yaw[3]   = {0.48f, 0.0f, 0.0f};
/***2***/
//fp32 PID_Roll[3]  = {4.4f, 0.0f, 0.0f};
//fp32 PID_Pitch[3] = {5.0f, 0.0f, 0.0f};
//fp32 PID_Yaw[3]   = {7.6f, 0.0f, 0.0f};

///* 角速度内环 */
//fp32 PID_Rate_Roll[3]  = {0.58f, 0.0f, 0.05f};
//fp32 PID_Rate_Pitch[3] = {0.49f, 0.0f, 0.002f};
//fp32 PID_Rate_Yaw[3]   = {0.5f, 0.0f, 0.0f};

///***4***/
//fp32 PID_Roll[3]  = {3.2f, 0.0f, 0.0f};
//fp32 PID_Pitch[3] = {4.8f, 0.0f, 0.0f};
//fp32 PID_Yaw[3]   = {7.6f, 0.0f, 0.0f};

///* 角速度内环 */
//fp32 PID_Rate_Roll[3]  = {0.36f, 0.0f, 0.05f};
//fp32 PID_Rate_Pitch[3] = {0.46f, 0.0f, 0.002f};
//fp32 PID_Rate_Yaw[3]   = {0.4f, 0.0f, 0.0f};


/* 高度 */
fp32 PID_Alt[3] = {0.1f, 0.0f, 0.0f};


/* ========================================================== */
/* ======================= ADRC 参数 ========================= */
/* ========================================================== */

#define ADRC_CTRL_DT_S          0.001f

#define ADRC_ROLL_B0            0.050f
#define ADRC_ROLL_KP            0.028f
#define ADRC_ROLL_WO            35.0f
#define ADRC_ROLL_OUT_LIMIT     500.0f

#define ADRC_PITCH_B0           1.0f
#define ADRC_PITCH_KP           0.49f
#define ADRC_PITCH_WO           30.0f
#define ADRC_PITCH_OUT_LIMIT    100.0f

#define ADRC_YAW_B0             1.0f
#define ADRC_YAW_KP             0.80f
#define ADRC_YAW_WO             30.0f
#define ADRC_YAW_OUT_LIMIT      100.0f


/* ========================================================== */
/* ================== 宏定义与函数声明 ====================== */
/* ========================================================== */

#define rc_deadband_limit(input, output, deadband)           \
{                                                            \
    if ((input) > (deadband) || (input) < -(deadband))       \
    {                                                        \
        (output) = (input);                                  \
    }                                                        \
    else                                                     \
    {                                                        \
        (output) = 0;                                        \
    }                                                        \
}


void aircraft_init(aircraft_control_t *init);

void aricraft_behaviour_mode_set(
    aircraft_control_t *aircraft_mode_set);

void aricraft_fly_ctrl(
    aircraft_control_t *aircraft_fly_ctrl);

void Bias(
    aircraft_control_t *aircraft_fly_ctrl);

double num_limit(
    double num,
    double min,
    double max);


static void PC_Data_Reset(
    aircraft_control_t *aircraft_ctrl);

static float angle_limit_180(
    float angle);

static float Motor_Map_Interpolate(
    float cmd,
    const motor_calib_point_t *map,
    uint8_t len);

static float Motor_Left_CmdToPwm(
    float cmd);

static float Motor_Right_CmdToPwm(
    float cmd);

static void Roll_Motor_Trim_Update(
    aircraft_control_t *aircraft_fly_ctrl,
    float target_roll,
    float target_pitch,
    float compensated_thrust);

static void Motor_Output_Write(
    aircraft_control_t *aircraft_fly_ctrl);
static void Flip_Motor_Output(
    aircraft_control_t *aircraft,
    float base_cmd,
    float diff_cmd);

static void Flip_Controller_Reset(
    aircraft_control_t *aircraft);

/* ========================================================== */
/* ===================== 开环翻滚参数 ======================== */
/* ========================================================== */

/*
 * 翻滚方向：
 * +1：左电机加速，右电机减速
 * -1：右电机加速，左电机减速
 */
#define FLIP_DIR                    1.0f

/* 最低允许触发油门 */
#define FLIP_TRIGGER_MIN_THRUST     220.0f

/* 翻滚时使用的基础油门范围 */
#define FLIP_BASE_MIN               260.0f
#define FLIP_BASE_MAX               760.0f

/* 起翻前两侧同时补油 */
#define FLIP_PREBOOST_ADD           110.0f

/* 主翻滚差速 */
#define FLIP_DRIVE_DIFF             73.0f

/* 翻滚末端反向刹车差速 */
#define FLIP_BRAKE_DIFF             40.0f

/* 翻滚完成后两侧补油 */
#define FLIP_RECOVER_ADD            75.0f

/* 起翻前补油时间 */
#define FLIP_PREBOOST_MS            100U

/* 翻滚最大允许时间 */
#define FLIP_TIMEOUT_MS             950U

/* 累计翻到多少度开始刹车 */
#define FLIP_BRAKE_START_DEG        225.0f

/* 至少翻过多少度才允许完成 */
#define FLIP_FINISH_MIN_DEG         325.0f

/* 回到起始横滚角附近 */
#define FLIP_FINISH_ROLL_ERR_DEG    35.0f

/* Roll 角速度低于这个值才结束 */
#define FLIP_FINISH_RATE_DPS        160.0f

/* 翻滚结束补油时间 */
#define FLIP_RECOVER_MS             90U


/* ========================================================== */
/* ===================== 翻滚状态机 ========================== */
/* ========================================================== */

#define FLIP_IDLE                   0
#define FLIP_PREBOOST               1
#define FLIP_DRIVE                  2
#define FLIP_BRAKE                  3
#define FLIP_RECOVER                4
#define FLIP_WAIT_RELEASE           5


static uint8_t flip_state = FLIP_IDLE;

static uint32_t flip_start_time = 0;
static uint32_t flip_last_tick = 0;
static uint32_t flip_recover_start_time = 0;

/* 触发翻滚瞬间的基础油门 */
static float flip_base_cmd = 0.0f;

/* 开始翻滚时的 Roll 角 */
static float flip_start_roll = 0.0f;

/* 陀螺积分得到的累计翻滚角 */
static float flip_accum_angle = 0.0f;


/* ========================================================== */
/* ================== RTOS 主控制任务 ======================= */
/* ========================================================== */

static void Flow_Run_PID_Init(void)
{
    const fp32 vx[3] = {FLOW_RUN_VX_KP, FLOW_RUN_VX_KI, FLOW_RUN_VX_KD};
    const fp32 vy[3] = {FLOW_RUN_VY_KP, FLOW_RUN_VY_KI, FLOW_RUN_VY_KD};
    /* 高度积分由 Flow_Run_Thrust 按输出约束更新，通用 PID 内的 Ki 保持为 0。 */
    const fp32 height[3] = {FLOW_RUN_HEIGHT_KP, 0.0f, FLOW_RUN_HEIGHT_KD};
    /* X/Y 速度环直接将角度指令交给原姿态环；FLT_MAX 放行所有有限 float 输出。 */
    PID_init(&PID_flow_vx, PID_POSITION, vx, FLT_MAX, FLOW_RUN_XY_IOUT_MAX_DEG);
    PID_init(&PID_flow_vy, PID_POSITION, vy, FLT_MAX, FLOW_RUN_XY_IOUT_MAX_DEG);
    PID_init(&PID_flow_height, PID_POSITION, height, FLOW_RUN_THRUST_MAX, FLOW_RUN_THRUST_MAX);
}

/* 在 PWM 写入后取数，便于对照速度环输出、姿态目标和实际 CCR。 */
static void Flow_Record_Control(aircraft_control_t *aircraft)
{
    OpticalFlow_Control_t *control = &aircraft->flow_data.control;
    *control = (OpticalFlow_Control_t){
        .update_ms = HAL_GetTick(),
        .velocity_sample_ms = aircraft->flow_data.flow_update_ms,
        .flow_mode_active = aircraft->aircraft_behaviour == Aircraft_FLOW_RUN,
        .velocity_valid = aircraft->flow_data.velocity_valid,
        .vx_feedback_mps = aircraft->flow_data.vx_body_mps,
        .vy_feedback_mps = aircraft->flow_data.vy_body_mps,
        .motor_left_ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4),
        .motor_right_ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3),
        .servo_left_ccr = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2),
        .servo_right_ccr = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1),
    };
    if (control->flow_mode_active) {
        control->xy_pid_active = FLOW_RUN_XY_VELOCITY_ENABLE && aircraft->flow_data.velocity_valid;
        if (control->xy_pid_active) {
            control->vx_feedback_mps = PID_flow_vx.fdb;
            control->vy_feedback_mps = PID_flow_vy.fdb;
        }
        control->height_pid_active = FLOW_RUN_HEIGHT_ENABLE && aircraft->flow_data.height_valid;
        control->vx_set_mps = PID_flow_vx.set;
        control->vy_set_mps = PID_flow_vy.set;
        control->vx_unclamped_deg = PID_flow_vx.Pout + PID_flow_vx.Iout + PID_flow_vx.Dout;
        control->vy_unclamped_deg = PID_flow_vy.Pout + PID_flow_vy.Iout + PID_flow_vy.Dout;
        control->vx_pid_out_deg = PID_flow_vx.out;
        control->vy_pid_out_deg = PID_flow_vy.out;
        control->pitch_target_deg = PID_control_pitch.set;
        control->roll_target_deg = PID_control_roll.set;
        control->pitch_rate_out = PID_rate_pitch.out;
        control->roll_rate_out = PID_rate_roll.out;
        if (control->height_pid_active) {
            control->height_pid_out = PID_flow_height.out;
        }
    }
    mavlink_flow_publish_control(control);
}

void Aircraft_ctrl_task(void const *pvParameters)
{
    Flow_Run_PID_Init();

    PID_init(
        &PID_control_pitch,
        PID_POSITION,
        PID_Pitch,
        1000,
        10000);

    PID_init(
        &PID_control_roll,
        PID_POSITION,
        PID_Roll,
        1000,
        10000);

    PID_init(
        &PID_control_yaw,
        PID_POSITION,
        PID_Yaw,
        1000,
        500);

    PID_init(
        &PID_control_altitude,
        PID_POSITION,
        PID_Alt,
        1000,
        10000);


    PID_init(
        &PID_rate_roll,
        PID_POSITION,
        PID_Rate_Roll,
        1000,
        500.0f);

    PID_init(
        &PID_rate_pitch,
        PID_POSITION,
        PID_Rate_Pitch,
        300,
        100.0f);

    PID_init(
        &PID_rate_yaw,
        PID_POSITION,
        PID_Rate_Yaw,
        300,
        100.0f);


    ADRC_1st_Init(
        &ADRC_rate_roll,
        ADRC_CTRL_DT_S,
        ADRC_ROLL_B0,
        ADRC_ROLL_KP,
        ADRC_ROLL_WO,
        ADRC_ROLL_OUT_LIMIT);

    ADRC_1st_Init(
        &ADRC_rate_pitch,
        ADRC_CTRL_DT_S,
        ADRC_PITCH_B0,
        ADRC_PITCH_KP,
        ADRC_PITCH_WO,
        ADRC_PITCH_OUT_LIMIT);

    ADRC_1st_Init(
        &ADRC_rate_yaw,
        ADRC_CTRL_DT_S,
        ADRC_YAW_B0,
        ADRC_YAW_KP,
        ADRC_YAW_WO,
        ADRC_YAW_OUT_LIMIT);


    LPF2nd_Init_30Hz(&gyro_pitch_lpf);
    LPF2nd_Init_30Hz(&gyro_roll_lpf);

    LPF2nd_Init_30Hz(&acc_pitch_lpf);
    LPF2nd_Init_30Hz(&acc_roll_lpf);

    LPF2nd_Init_30Hz(&angle_pitch_lpf);
    LPF2nd_Init_30Hz(&angle_roll_lpf);


    aircraft_init(&aircraft_control);


    while (1)
    {
        /* 每个控制周期只取一次快照，本周期内各环使用同一组观测值和有效标志。 */
        mavlink_flow_get_snapshot(&aircraft_control.flow_data, HAL_GetTick());
        aricraft_behaviour_mode_set(&aircraft_control);
        aricraft_fly_ctrl(&aircraft_control);
        Flow_Record_Control(&aircraft_control);

        vTaskDelay(1);
    }
}


/* ========================================================== */
/* ==================== 初始化函数 ========================== */
/* ========================================================== */

void aircraft_init(aircraft_control_t *init)
{
    init->aircraft_behaviour =
        Aircraft_ZERO_FORCE;

    init->last_aircraft_behaviour =
        Aircraft_ZERO_FORCE;


    init->remote_data =
        get_remote_control_point();

    init->IMU_data =
        get_imu_data_point();

    init->pc_data =
        get_pc_data_point();


    init->motor_left_out  = 0;
    init->motor_right_out = 0;

    init->servo_left_out  = 0;
    init->servo_right_out = 0;

    init->Pitch_Rate_Set = 0;
    init->Roll_Rate_Set  = 0;

    init->yaw_set_angle = 0;


    /* 电机最低输出 */
    __HAL_TIM_SetCompare(
        &htim3,
        TIM_CHANNEL_3,
        1000);

    __HAL_TIM_SetCompare(
        &htim3,
        TIM_CHANNEL_4,
        1000);


    /* 舵机回中 */
    __HAL_TIM_SetCompare(
        &htim5,
        TIM_CHANNEL_1,
        900);

    __HAL_TIM_SetCompare(
        &htim5,
        TIM_CHANNEL_2,
        900);
}


/* ========================================================== */
/* ======================= 限幅函数 ========================= */
/* ========================================================== */

double num_limit(
    double num,
    double min,
    double max)
{
    if (num > max)
    {
        return max;
    }

    if (num < min)
    {
        return min;
    }

    return num;
}


/* ========================================================== */
/* ====================== PC 数据清零 ======================= */
/* ========================================================== */

static void PC_Data_Reset(
    aircraft_control_t *aircraft_ctrl)
{
    if (aircraft_ctrl == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    PC_Data_Clear();

    taskEXIT_CRITICAL();
}


/* ========================================================== */
/* ===================== 角度 ±180 ========================== */
/* ========================================================== */

static float angle_limit_180(
    float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}


/* ========================================================== */
/* ======================= 手动微调 ========================= */
/* ========================================================== */

void Bias(
    aircraft_control_t *aircraft_fly_ctrl)
{
    if (aircraft_fly_ctrl->remote_data->Right_Y < -50)
    {
        pitch_bias -= 0.2f;
    }
    else if (aircraft_fly_ctrl->remote_data->Right_Y > 50)
    {
        pitch_bias += 0.2f;
    }


    if (aircraft_fly_ctrl->remote_data->Left_X < -50)
    {
        base_throttle -= 0.2f;
    }
    else if (aircraft_fly_ctrl->remote_data->Left_X > 50)
    {
        base_throttle += 0.2f;
    }
}


/* ========================================================== */
/* =================== 电机标定插值 ========================= */
/* ========================================================== */

static float Motor_Map_Interpolate(
    float cmd,
    const motor_calib_point_t *map,
    uint8_t len)
{
    uint8_t i;


    if (len < 2)
    {
        return cmd;
    }


    if (cmd <= map[0].cmd)
    {
        return map[0].pwm;
    }


    if (cmd >= map[len - 1].cmd)
    {
        return map[len - 1].pwm;
    }


    for (i = 0; i < len - 1; i++)
    {
        if (cmd >= map[i].cmd &&
            cmd <= map[i + 1].cmd)
        {
            float x0 = map[i].cmd;
            float y0 = map[i].pwm;

            float x1 = map[i + 1].cmd;
            float y1 = map[i + 1].pwm;

            float ratio =
                (cmd - x0) / (x1 - x0);

            return
                y0 + ratio * (y1 - y0);
        }
    }


    return cmd;
}


/* ========================================================== */
/* ================= 左电机 CMD -> PWM ====================== */
/* ========================================================== */

static float Motor_Left_CmdToPwm(
    float cmd)
{
    float pwm;


    cmd = (float)num_limit(
        cmd,
        MOTOR_CMD_MIN,
        MOTOR_CMD_MAX);


    pwm = Motor_Map_Interpolate(
        cmd,
        left_motor_map,
        sizeof(left_motor_map) /
        sizeof(left_motor_map[0]));


    pwm =
        left_motor_bias +
        left_motor_gain * pwm;


    pwm = (float)num_limit(
        pwm,
        MOTOR_CMD_MIN,
        MOTOR_CMD_MAX);


    return pwm;
}


/* ========================================================== */
/* ================= 右电机 CMD -> PWM ====================== */
/* ========================================================== */

static float Motor_Right_CmdToPwm(
    float cmd)
{
    float pwm;


    cmd = (float)num_limit(
        cmd,
        MOTOR_CMD_MIN,
        MOTOR_CMD_MAX);


    pwm = Motor_Map_Interpolate(
        cmd,
        right_motor_map,
        sizeof(right_motor_map) /
        sizeof(right_motor_map[0]));


    pwm =
        right_motor_bias +
        right_motor_gain * pwm;


    pwm = (float)num_limit(
        pwm,
        MOTOR_CMD_MIN,
        MOTOR_CMD_MAX);


    return pwm;
}


/* ========================================================== */
/* =================== Roll 自动 Trim ======================= */
/* ========================================================== */

static void Roll_Motor_Trim_Update(
    aircraft_control_t *aircraft_fly_ctrl,
    float target_roll,
    float target_pitch,
    float compensated_thrust)
{
#if ROLL_TRIM_ENABLE

    float roll_angle;
    float roll_gyro;
    float roll_inner_out;


    roll_angle =
        aircraft_fly_ctrl
        ->IMU_data
        ->INS_angle[2];

    roll_gyro =
        aircraft_fly_ctrl
        ->IMU_data
        ->INS_gyro[2];


    /*
     * ADRC 模式使用 ADRC 输出作为长期偏差依据。
     * PID 模式使用 PID 输出。
     */
    if (aircraft_fly_ctrl->aircraft_behaviour ==
        Aircraft_RUN_ADRC)
    {
        roll_inner_out =
            ADRC_rate_roll.out;
    }
    else
    {
        roll_inner_out =
            PID_rate_roll.out;
    }


    if (
        fabsf(target_roll) < 2.0f &&
        fabsf(target_pitch) < 3.0f &&
        compensated_thrust > 250.0f &&
        fabsf(roll_angle) < 12.0f &&
        fabsf(roll_gyro) < 80.0f)
    {
        /*
         * 对长期 Roll 内环输出做慢速低通。
         */
        roll_trim_error_lpf =
            roll_trim_error_lpf * 0.995f +
            roll_inner_out * 0.005f;


        /*
         * 缓慢学习静态左右不一致。
         */
        roll_motor_trim +=
            roll_trim_learn_rate *
            roll_trim_error_lpf;


        roll_motor_trim =
            (float)num_limit(
                roll_motor_trim,
                -ROLL_TRIM_LIMIT,
                ROLL_TRIM_LIMIT);
    }
    else
    {
        /*
         * 未满足学习条件时只让误差平均值缓慢衰减。
         */
        roll_trim_error_lpf *= 0.999f;
    }

#endif
}


/* ========================================================== */
/* ===================== 最终电机输出 ======================= */
/* ========================================================== */

static void Motor_Output_Write( aircraft_control_t *aircraft_fly_ctrl)
{
    float left_pwm_offset;
    float right_pwm_offset;

    left_pwm_offset =   Motor_Left_CmdToPwm( aircraft_fly_ctrl ->motor_left_out);
    right_pwm_offset =  Motor_Right_CmdToPwm( aircraft_fly_ctrl ->motor_right_out);

    left_motor_pwm =   (int32_t)(  MOTOR_PWM_BASE +  left_pwm_offset);
    right_motor_pwm =  (int32_t)(  MOTOR_PWM_BASE +   right_pwm_offset);
    left_motor_pwm =   (int32_t)num_limit(   left_motor_pwm,  1000,  1900);
    right_motor_pwm =  (int32_t)num_limit(   right_motor_pwm, 1000,  1900);

    /* * TIM3 CH4 -> 左电机  * TIM3 CH3 -> 右电机   */
    __HAL_TIM_SetCompare(  &htim3,   TIM_CHANNEL_4, (uint32_t)left_motor_pwm);
    __HAL_TIM_SetCompare(  &htim3,   TIM_CHANNEL_3, (uint32_t)right_motor_pwm);
}

/* ========================================================== */
/* =================== 翻滚电机直接输出 ===================== */
/* ========================================================== */

static void Flip_Motor_Output(
    aircraft_control_t *aircraft,
    float base_cmd,
    float diff_cmd)
{
    float left_cmd;
    float right_cmd;

    /*
     * 正 diff：
     * 左电机增加
     * 右电机减少
     */
    left_cmd  =  base_cmd +  FLIP_DIR * diff_cmd;
    right_cmd =  base_cmd -  FLIP_DIR * diff_cmd;

    left_cmd =(float)num_limit(     left_cmd,  0.0f,    900.0f);
    right_cmd =(float)num_limit(    right_cmd, 0.0f,    900.0f);
 
    aircraft->motor_left_out =       left_cmd;
    aircraft->motor_right_out =      right_cmd;

    Motor_Output_Write(  aircraft);

     /*
     * 翻滚期间舵机回中。
     *
     * TIM5 CH2 -> 左舵机
     * TIM5 CH1 -> 右舵机
     */
    __HAL_TIM_SetCompare(
        &htim5,
        TIM_CHANNEL_1,
        900);

    __HAL_TIM_SetCompare(
        &htim5,
        TIM_CHANNEL_2,
        900);
}


/* ========================================================== */
/* ================= 翻滚结束控制器复位 ===================== */
/* ========================================================== */

static void Flip_Controller_Reset(
    aircraft_control_t *aircraft)
{
    /*
     * 清除翻滚前的 PID 历史状态。
     * 特别是 Roll Rate PID 有 D 项，
     * 如果不清除，重新进入正常控制时
     * 第一帧可能出现比较大的瞬态。
     */
    PID_clear(
        &PID_control_roll);

    PID_clear(
        &PID_control_pitch);

    PID_clear(
        &PID_control_yaw);

    PID_clear(
        &PID_rate_roll);

    PID_clear(
        &PID_rate_pitch);

    PID_clear(
        &PID_rate_yaw);


    /*
     * 翻滚结束后重新锁定当前 Yaw。
     */
    aircraft->yaw_set_angle =
        aircraft
        ->IMU_data
        ->INS_angle[0];
}
/* 读取当前水平控制方式对应的 ID 248 目标，超时保持调用方的零目标。 */
static void Flow_Apply_PC_Target(const PC_Data_t *pc, float *x, float *y)
{
    const float limit = FLOW_RUN_XY_VELOCITY_ENABLE ? FLOW_RUN_XY_MAX_MPS : FLOW_RUN_ATTITUDE_MAX_DEG;
    taskENTER_CRITICAL();
    bool fresh = pc->flow_target_rx_count != 0U &&
        (uint32_t)(HAL_GetTick() - pc->flow_target_update_ms) <= FLOW_PC_TARGET_TIMEOUT_MS;
    float target_x = FLOW_RUN_XY_VELOCITY_ENABLE ? pc->vx_set_mps : pc->flow_pitch_set_deg;
    float target_y = FLOW_RUN_XY_VELOCITY_ENABLE ? pc->vy_set_mps : pc->flow_roll_set_deg;
    taskEXIT_CRITICAL();
    if (fresh) {
        *x = (float)num_limit(target_x, -limit, limit);
        *y = (float)num_limit(target_y, -limit, limit);
    }
}

/* ========================================================== */
/* ==================== 状态机模式设置  ====================== */
/* ========================================================== */

/* 切入控速模式时重置高度跟踪；从飞行模式切入时，用左右电机指令的均值衔接推力。 */
static void Flow_Run_Enter(const aircraft_control_t *aircraft)
{
    PID_clear(&PID_flow_vx);
    PID_clear(&PID_flow_vy);
    flip_state = FLIP_IDLE;
    if (!FLOW_RUN_HEIGHT_ENABLE) {
        return;
    }
    flow_run.target_height_m = FLOW_RUN_TARGET_HEIGHT_M;
    flow_run.tracking_height = false;
    flow_run.tick_ms = HAL_GetTick();
    flow_run.thrust = 0.0f;
    if (aircraft->last_aircraft_behaviour == Aircraft_RUN ||
        aircraft->last_aircraft_behaviour == Aircraft_Auto ||
        aircraft->last_aircraft_behaviour == Aircraft_RUN_ADRC) 
	{
        flow_run.thrust = (float)num_limit(   (aircraft->motor_left_out + aircraft->motor_right_out) * 0.5f, 0.0f, FLOW_RUN_THRUST_MAX);
    }
}

/* 首帧或测距恢复时衔接推力，并从当前测量重新估算升降速度。 */
static void Flow_Run_Update_Height(const OpticalFlow_Data_t *flow, uint32_t elapsed_ms)
{
    uint32_t sample_ms = flow->height_update_ms - flow_run.sample_ms;
    if (!flow_run.tracking_height || elapsed_ms > FLOW_TIMEOUT_MS || sample_ms > FLOW_TIMEOUT_MS) {
        PID_clear(&PID_flow_height);
        PID_flow_height.Iout = flow_run.thrust;  /* 零速度误差时衔接上一周期的推力 */
        flow_run.vz_mps = 0.0f;
        flow_run.tracking_height = true;
    } else if (sample_ms >= FLOW_RUN_VZ_SAMPLE_MIN_MS) {
        float seconds = (float)sample_ms * 0.001f;
        float vz = (flow->height_m - flow_run.previous_height_m) / seconds;
        flow_run.vz_mps += seconds / (FLOW_RUN_VZ_FILTER_TAU_S + seconds) * (vz - flow_run.vz_mps);
    } else {
        return;
    }
    flow_run.previous_height_m = flow->height_m;
    flow_run.sample_ms = flow->height_update_ms;
}

/* 高度有效时按固定目标计算推力，测距失效时使用手动油门。 */
static float Flow_Run_Thrust(const OpticalFlow_Data_t *flow, float throttle)
{
    /* 仅调水平速度时，同一油门杆位在切换前后给出相同基础推力。 */
    if (!FLOW_RUN_HEIGHT_ENABLE) {
        return throttle * RUN_THROTTLE_SCALE;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - flow_run.tick_ms;
    float dt = (float)num_limit(elapsed_ms, 0, FLOW_RUN_DT_MAX_MS) * 0.001f;
    float step = FLOW_RUN_THRUST_SLEW * dt;
    float requested = throttle * FLOW_RUN_MANUAL_SCALE;  /* 测距失效时回到普通 RUN 的油门映射 */
    flow_run.tick_ms = now;

    if (flow->height_valid) {
        Flow_Run_Update_Height(flow, elapsed_ms);
        /* 固定高度的期望升降速度为零，D 项用于抑制实际升降。 */
        float error_rate = -flow_run.vz_mps;
        float error = flow_run.target_height_m - flow->height_m;
        float before_integral = PID_flow_height.Iout + PID_flow_height.Kp * error
                              + PID_flow_height.Kd * error_rate;
        /* 仅在本周期可达到的输出范围内累积积分，允许积分减轻已有饱和。 */
        float reachable_min = fmaxf(0.0f, flow_run.thrust - step);
        float reachable_max = fminf(FLOW_RUN_THRUST_MAX, flow_run.thrust + step);
        float max_integral_step = FLOW_RUN_HEIGHT_I_SLEW * dt;
        float integral_step = (float)num_limit(FLOW_RUN_HEIGHT_KI * error * dt,
                                               -max_integral_step, max_integral_step);
        if (integral_step > 0.0f) {
            integral_step = fminf(integral_step, fmaxf(0.0f, reachable_max - before_integral));
        } else {
            integral_step = fmaxf(integral_step, fminf(0.0f, reachable_min - before_integral));
        }
        PID_flow_height.Iout += integral_step;
        requested = PID_calc_with_rate(&PID_flow_height, flow->height_m,
                                       flow_run.target_height_m, error_rate);
    } else {
        flow_run.tracking_height = false;
    }

    requested = (float)num_limit(requested, 0.0f, FLOW_RUN_THRUST_MAX);
    flow_run.thrust += (float)num_limit(requested - flow_run.thrust, -step, step);
    return flow_run.thrust;
}

void aricraft_behaviour_mode_set(
    aircraft_control_t *aircraft_mode_set)
{
    taskENTER_CRITICAL();
    uint8_t switch_b = aircraft_mode_set->remote_data->B;
    uint8_t switch_c = aircraft_mode_set->remote_data->C;
    uint8_t switch_e = aircraft_mode_set->remote_data->E;
    taskEXIT_CRITICAL();

    /*
     * B=0 C=0
     * 安全停机
     */
    if (
        switch_b == 0 &&
        switch_c == 0)
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_ZERO_FORCE;
    }
    /*
     * B=2 C=0
     * 普通 RUN
     *
     * E=2 不改变状态机，
     * 而是在 RUN 控制内部进入高速 RUN。
     */
    else if (
        switch_b == 2 &&
        switch_c == 0)
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_RUN;
    }

    /*
     * B=1 C=0
     * 光流控速进入后保持，停机和普通 RUN 的拨杆组合优先。
     */
    else if (aircraft_mode_set->last_aircraft_behaviour == Aircraft_FLOW_RUN ||
             (switch_b == 1 && switch_c == 0))
    {
        aircraft_mode_set->aircraft_behaviour = Aircraft_FLOW_RUN;
        if (aircraft_mode_set->last_aircraft_behaviour != Aircraft_FLOW_RUN) {
            Flow_Run_Enter(aircraft_mode_set);
        }
    }

    /*
     * E=2 B=2 C=2
     * Pitch 内环调试模式
     *
     * 必须放在 Auto 前面。
     */
    else if (
        switch_e == 2 &&
        switch_b == 2 &&
        switch_c == 2)
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_InRing_R;
    }

    /*
     * B=2 C=2
     * PC Auto
     */
    else if (
        switch_b == 2 &&
        switch_c == 2)
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_Auto;
    }

    /*
     * B=0 C=2
     * ADRC RUN
     */
    else if (
        switch_b == 0 &&
        switch_c == 2)
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_RUN_ADRC;
    }

    /*
     * B=2 C=1
     * Roll 外环调试
     */
    else if (
        switch_b == 2 &&
        switch_c == 1)
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_OutRing_R;
    }

    else
    {
        aircraft_mode_set->aircraft_behaviour =
            Aircraft_ZERO_FORCE;
    }


    /* ====================================================== */
    /* ================= Auto 模式切入处理 ================== */
    /* ====================================================== */

    if (
        aircraft_mode_set->aircraft_behaviour ==
            Aircraft_Auto &&
        aircraft_mode_set->last_aircraft_behaviour !=
            Aircraft_Auto)
    {
        /*
         * 清除上一帧 PC 残留命令。
         */
        PC_Data_Reset(
            aircraft_mode_set);


        /*
         * Auto 刚进入时锁定当前航向。
         */
        aircraft_mode_set->yaw_set_angle =
            aircraft_mode_set
            ->IMU_data
            ->INS_angle[0];
    }


    /* ====================================================== */
    /* ================= ADRC 模式切入处理 ================== */
    /* ====================================================== */

    if (
        aircraft_mode_set->aircraft_behaviour ==
            Aircraft_RUN_ADRC &&
        aircraft_mode_set->last_aircraft_behaviour !=
            Aircraft_RUN_ADRC)
    {
        ADRC_1st_Reset(
            &ADRC_rate_roll,
            -aircraft_mode_set
                ->IMU_data
                ->INS_gyro[2],
            0.0f);


        ADRC_1st_Reset(
            &ADRC_rate_pitch,
            -aircraft_mode_set
                ->IMU_data
                ->INS_gyro[0],
            0.0f);


        ADRC_1st_Reset(
            &ADRC_rate_yaw,
            -aircraft_mode_set
                ->IMU_data
                ->INS_gyro[1],
            0.0f);
    }


    /* ====================================================== */
    /* ========== 从安全状态进入飞行状态初始化 ============== */
    /* ====================================================== */

    if (
        (
            aircraft_mode_set->aircraft_behaviour ==
                Aircraft_RUN ||

            aircraft_mode_set->aircraft_behaviour == Aircraft_FLOW_RUN ||

            aircraft_mode_set->aircraft_behaviour ==
                Aircraft_Auto ||

            aircraft_mode_set->aircraft_behaviour ==
                Aircraft_RUN_ADRC
        )
        &&
        aircraft_mode_set->last_aircraft_behaviour ==
            Aircraft_ZERO_FORCE)
    {
        /*
         * 锁定当前 Yaw。
         */
        aircraft_mode_set->yaw_set_angle =
            aircraft_mode_set
            ->IMU_data
            ->INS_angle[0];


        /*
         * 高度零点。
         */
        altitude_bias =
            spl06_inst.filtered_altitude;


        SPL06_Calibrate_Reference(
            &spl06_inst,
            0.0f);


        /*
         * 清除姿态外环状态。
         */
        PID_clear(
            &PID_control_yaw);

        PID_clear(
            &PID_control_roll);

        PID_clear(
            &PID_control_pitch);


        /*
         * 如果希望每次重新起飞都重新学习 trim，
         * 可以打开：
         */
        /* roll_motor_trim = 0.0f; */
    }


    aircraft_mode_set->last_aircraft_behaviour =
        aircraft_mode_set->aircraft_behaviour;
}


/* ========================================================== */
/* ==================== 飞行控制主函数 ====================== */
/* ========================================================== */

void aricraft_fly_ctrl(
    aircraft_control_t *aircraft_fly_ctrl)
{
    /* ====================================================== */
    /* ===================== 安全停机 ======================== */
    /* ====================================================== */

    if (
        aircraft_fly_ctrl->aircraft_behaviour ==
        Aircraft_ZERO_FORCE)
    {
         /*
         * 安全模式下强制取消翻滚。
         */
        flip_state = FLIP_IDLE;
        flip_accum_angle = 0.0f;
        flip_base_cmd = 0.0f;
		
		
		
		
		/* 电机最低 */
        __HAL_TIM_SetCompare(
            &htim3,
            TIM_CHANNEL_3,
            1000);

        __HAL_TIM_SetCompare(
            &htim3,
            TIM_CHANNEL_4,
            1000);


        /* 舵机回中 */
        __HAL_TIM_SetCompare(
            &htim5,
            TIM_CHANNEL_1,
            900);

        __HAL_TIM_SetCompare(
            &htim5,
            TIM_CHANNEL_2,
            900);
    }


    /* ====================================================== */
    /* =================== Pitch 外环调试 =================== */
    /* ====================================================== */

    else if ( aircraft_fly_ctrl->aircraft_behaviour ==  Aircraft_InRing_P)
    {
        if ( aircraft_fly_ctrl->remote_data->F == 2)
        {
            PID_control_pitch.Kp =  0.1f *   aircraft_fly_ctrl  ->remote_data  ->Left_Y;
        }
    }


    /* ====================================================== */
    /* =================== Pitch 内环调试 =================== */
    /* ====================================================== */

    else if ( aircraft_fly_ctrl->aircraft_behaviour ==   Aircraft_InRing_R)
    {
        if ( aircraft_fly_ctrl->remote_data->F == 2)
        {
            PID_rate_pitch.Kp =   0.05f *   aircraft_fly_ctrl   ->remote_data   ->Left_Y;
        }
    }


    /* ====================================================== */
    /* ==================== Roll 外环调试 =================== */
    /* ====================================================== */

    else if ( aircraft_fly_ctrl->aircraft_behaviour ==  Aircraft_OutRing_R)
    {
        if (    aircraft_fly_ctrl->remote_data->F == 2)    
			{
            PID_control_roll.Kp =
                0.05f *
                aircraft_fly_ctrl
                ->remote_data
                ->Left_Y;
        }
    }


    /* ====================================================== */
    /* ================= RUN / Auto / ADRC ================== */
    /* ====================================================== */

    else if (
        aircraft_fly_ctrl->aircraft_behaviour ==   Aircraft_RUN || 
        aircraft_fly_ctrl->aircraft_behaviour ==   Aircraft_FLOW_RUN ||
        aircraft_fly_ctrl->aircraft_behaviour ==   Aircraft_Auto ||
        aircraft_fly_ctrl->aircraft_behaviour ==   Aircraft_RUN_ADRC)
    {
        float target_roll = 0.0f;
        float target_pitch = 0.0f;

        float base_thrust = 0.0f;

        float Yaw_Rate_Set = 0.0f;

        float roll_inner_out = 0.0f;
        float pitch_inner_out = 0.0f;
        float yaw_inner_out = 0.0f;


        const float stick_deadband = aircraft_fly_ctrl->aircraft_behaviour == Aircraft_FLOW_RUN
                                        ? FLOW_RUN_STICK_DEADBAND : RC_DEADBAND;
        int Right_X = 0;
        int Right_Y = 0;
        int Left_X = 0;


        /* ================================================== */
        /* =================== 遥控器死区 ==================== */
        /* ================================================== */

        taskENTER_CRITICAL();
        float right_x = aircraft_fly_ctrl->remote_data->Right_X;
        float right_y = aircraft_fly_ctrl->remote_data->Right_Y;
        uint8_t switch_c = aircraft_fly_ctrl->remote_data->C;
        taskEXIT_CRITICAL();

        rc_deadband_limit(right_x, Right_X, stick_deadband);
        rc_deadband_limit(right_y, Right_Y, stick_deadband);


        rc_deadband_limit(
            aircraft_fly_ctrl
                ->remote_data
                ->Left_X,
            Left_X,
            stick_deadband);


        /* ================================================== */
        /* ==================== RUN 模式 ====================== */
        /* ================================================== */

        if (
            aircraft_fly_ctrl->aircraft_behaviour ==  Aircraft_RUN ||
            aircraft_fly_ctrl->aircraft_behaviour ==  Aircraft_RUN_ADRC)
        {
            float roll_scale  =     RUN_ROLL_SCALE;
            float pitch_scale =     RUN_PITCH_SCALE;
            float yaw_scale   =     RUN_YAW_SCALE;

            /*
             * =================================================
             * 高速 RUN 模式
             * =================================================
             * 条件  Aircraft_RUN  &&  E == 2  
             * 注意：Aircraft_RUN_ADRC 当前不进入高速模式。
             */
            if (
                aircraft_fly_ctrl->aircraft_behaviour ==  Aircraft_RUN &&
                aircraft_fly_ctrl->remote_data->E == 2)
            {
                roll_scale  =   RUN_FAST_ROLL_SCALE;
                pitch_scale =   RUN_FAST_PITCH_SCALE;
                yaw_scale   =   RUN_FAST_YAW_SCALE;
            }


            /*
             * Roll 目标角：
             *
             * 普通：
             * target_roll = -Right_X * 0.40
             *
             * 高速：
             * target_roll = -Right_X * 0.55
             */
            target_roll =    -Right_X *   roll_scale;


            /*
             * Pitch 目标角
             */
            target_pitch =    Right_Y *   pitch_scale;


            /*
             * Yaw 为增量式目标角。
             *
             * 普通：
             * 0.004
             *
             * 高速：
             * 0.006
             */
            aircraft_fly_ctrl->yaw_set_angle = 
			aircraft_fly_ctrl->yaw_set_angle -   Left_X *  yaw_scale;

            /* 油门保持原逻辑 */
            base_thrust =   aircraft_fly_ctrl   ->remote_data   ->Left_Y *  RUN_THROTTLE_SCALE;
        }


        else if (aircraft_fly_ctrl->aircraft_behaviour == Aircraft_FLOW_RUN)
        {
            const OpticalFlow_Data_t *flow = &aircraft_fly_ctrl->flow_data;
            bool stick_override = switch_c == 1U && (Right_X != 0 || Right_Y != 0);
            if (!FLOW_RUN_XY_VELOCITY_ENABLE) {
                if (stick_override) {
                    target_pitch = Right_Y * RUN_PITCH_SCALE;
                    target_roll = -Right_X * RUN_ROLL_SCALE;
                } else {
                    Flow_Apply_PC_Target(aircraft_fly_ctrl->pc_data, &target_pitch, &target_roll);
                }
            } else if (isfinite(flow->vx_body_mps) && isfinite(flow->vy_body_mps)) {
                float desired_vx = 0.0f;
                float desired_vy = 0.0f;
                if (stick_override) {
                    desired_vx = Right_Y * (FLOW_RUN_XY_MAX_MPS / 100.0f);
                    desired_vy = Right_X * (FLOW_RUN_XY_MAX_MPS / 100.0f);
                } else {
                    Flow_Apply_PC_Target(aircraft_fly_ctrl->pc_data, &desired_vx, &desired_vy);
                }
                /* 姿态解算中前倾为正、左滚为正；向右加速需要负的横滚目标角。 */
                float vx_feedback = flow->vx_body_mps * FLOW_RUN_VX_FEEDBACK_SCALE;
                float vy_feedback = flow->vy_body_mps * FLOW_RUN_VY_FEEDBACK_SCALE;
                target_pitch = PID_calc(&PID_flow_vx, vx_feedback, desired_vx);
                target_roll = -PID_calc(&PID_flow_vy, vy_feedback, desired_vy);
            } else {
                PID_clear(&PID_flow_vx);
                PID_clear(&PID_flow_vy);
            }
            /* 偏航由左横杆控制，推力由原定高逻辑计算。 */
            aircraft_fly_ctrl->yaw_set_angle -= Left_X * FLOW_RUN_YAW_SCALE;
            base_thrust = Flow_Run_Thrust(flow, aircraft_fly_ctrl->remote_data->Left_Y);
        }

        /* ================================================== */
        /* ==================== Auto 模式 ===================== */
        /* ================================================== */

        else if (
            aircraft_fly_ctrl->aircraft_behaviour ==
            Aircraft_Auto)
        {
            /*
             * PC Roll。
             */
            target_roll =
                (float)num_limit(
                    aircraft_fly_ctrl
                        ->pc_data
                        ->roll,
                    -AUTO_ROLL_LIMIT_DEG,
                    AUTO_ROLL_LIMIT_DEG);


            /*
             * PC Pitch。
             */
            target_pitch =
                (float)num_limit(
                    -aircraft_fly_ctrl
                        ->pc_data
                        ->pitch,
                    -AUTO_PITCH_LIMIT_DEG,
                    AUTO_PITCH_LIMIT_DEG);


            /*
             * 当前仍使用遥控器 Left_X
             * 微调 Auto 航向。
             */
            aircraft_fly_ctrl->yaw_set_angle = aircraft_fly_ctrl->yaw_set_angle + Left_X * 0.0004f;


            /*
             * 如果后面希望 PC 直接指定绝对 Yaw：
             *
             * aircraft_fly_ctrl->yaw_set_angle =
             *     angle_limit_180(
             *         (float)num_limit(
             *             aircraft_fly_ctrl->pc_data->yaw,
             *             -AUTO_YAW_LIMIT_DEG,
             *             AUTO_YAW_LIMIT_DEG));
             */


            {
                float pc_thrust_limit;
                pc_thrust_limit =  (float)num_limit
				(       aircraft_fly_ctrl  ->pc_data  ->thrust,
                        AUTO_THRUST_LIMIT_MIN,
                        AUTO_THRUST_LIMIT_MAX );


                /* 遥控器基础油门 */
                base_thrust =  aircraft_fly_ctrl   ->remote_data   ->Left_Y *   5.2f;

                /*  PC 附加油门 */
                base_thrust +=  pc_thrust_limit *  600.0f;
            }
        }


        /* ================================================== */
        /* ================= 姿态掉高补偿 ==================== */
        /* ================================================== */

        {
            float roll_rad;
            float pitch_rad;

            float cos_roll;
            float cos_pitch;

            float compensated_thrust;


            roll_rad =
                aircraft_fly_ctrl
                    ->IMU_data
                    ->INS_angle[2] *
                0.0174533f;


            pitch_rad =
                aircraft_fly_ctrl
                    ->IMU_data
                    ->INS_angle[1] *
                0.0174533f;


            cos_roll =
                cosf(roll_rad);

            cos_pitch =
                cosf(pitch_rad);


            /*
             * 避免接近 90° 后补偿爆炸。
             */
            if (cos_roll < 0.5f)
            {
                cos_roll = 0.5f;
            }


            if (cos_pitch < 0.5f)
            {
                cos_pitch = 0.5f;
            }


            compensated_thrust =
                base_thrust /
                (cos_roll * cos_pitch);


            compensated_thrust =
                (float)num_limit(
                    compensated_thrust,
                    0.0f,
                    850.0f);


            /*
             * 当前关闭姿态掉高补偿，
             * 保持原飞行手感。
             *
             * 如果希望开启掉高补偿，
             * 删除下面这两行。
             */
            compensated_thrust =
                base_thrust;


            compensated_thrust =
                (float)num_limit(
                    compensated_thrust,
                    0.0f,
                    850.0f);

 /* ================================================== */
/* ============ RUN 模式增强开环翻滚 ================= */
/* ================================================== */

/*
 * 翻滚结构：
 *
 * FLIP_PREBOOST
 *      ↓
 * FLIP_DRIVE
 *      ↓
 * FLIP_BRAKE
 *      ↓
 * FLIP_RECOVER
 *      ↓
 * FLIP_WAIT_RELEASE
 *
 * 电机仍然是开环控制。
 *
 * IMU 只负责：
 * 1. 判断已经翻了多少度
 * 2. 判断什么时候进入刹车
 * 3. 判断什么时候完成
 */

if (
    aircraft_fly_ctrl->aircraft_behaviour ==
    Aircraft_RUN)
{
    uint32_t now_tick =
        HAL_GetTick();

    float current_roll =
        aircraft_fly_ctrl
        ->IMU_data
        ->INS_angle[2];

    float current_pitch =
        aircraft_fly_ctrl
        ->IMU_data
        ->INS_angle[1];

    /*
     * 和你的正常 Roll Rate PID
     * 使用完全相同的符号。
     */
    float roll_rate =
        -aircraft_fly_ctrl
        ->IMU_data
        ->INS_gyro[2];


    /* ================================================== */
    /* =================== 等待触发 ====================== */
    /* ================================================== */

    if (flip_state == FLIP_IDLE)
    {
        /*
         * F=2 触发。
         ** 同时限制：
         * 不能大坡度触发
         * 不能大 Pitch 触发
         * 不能本身已经高速滚转时触发
         */
        if (
            aircraft_fly_ctrl->remote_data->F == 2 &&

            compensated_thrust >FLIP_TRIGGER_MIN_THRUST &&

            fabsf(current_roll) <20.0f &&

            fabsf(current_pitch) <20.0f &&

            fabsf(roll_rate) <120.0f)
        {
            flip_state =FLIP_PREBOOST;

            flip_start_time =now_tick;

            flip_last_tick =now_tick;

            flip_start_roll =current_roll;

            flip_accum_angle =0.0f;


            /** 关键改动：
             ** 不再固定使用 220 / 270。
             * 记录触发翻滚瞬间的基础油门。
             */
            flip_base_cmd =(float)num_limit(compensated_thrust,FLIP_BASE_MIN,FLIP_BASE_MAX);
        }
    }


    /* ================================================== */
    /* ================= Roll 角速度积分 ================= */
    /* ================================================== */

    if (
        flip_state == FLIP_PREBOOST || flip_state == FLIP_DRIVE || flip_state == FLIP_BRAKE)
    {
        uint32_t dt_ms = now_tick - flip_last_tick;

        /** 防止某一帧任务阻塞导致积分跳变。*/
        if (dt_ms > 10U){dt_ms = 10U;}
        flip_last_tick =now_tick;
        /*
         * 如果 INS_gyro 单位是 deg/s，
         * 积分后直接得到 deg。
         */
        flip_accum_angle +=  fabsf(roll_rate) * ((float)dt_ms * 0.001f);
    }


    /* ================================================== */
    /* ================== 第一阶段：补油 ================= */
    /* ================================================== */

    if (flip_state == FLIP_PREBOOST)
    {
        uint32_t elapsed =  now_tick - flip_start_time;
        if (  elapsed <  FLIP_PREBOOST_MS)
        {
            /*左右同时加油*/
            Flip_Motor_Output(aircraft_fly_ctrl,flip_base_cmd +FLIP_PREBOOST_ADD,0.0f);
            return;
        }

        flip_state =  FLIP_DRIVE;
    }


    /* ================================================== */
    /* ================= 第二阶段：主翻滚 ================ */
    /* ================================================== */

    if (flip_state == FLIP_DRIVE)
    {
        uint32_t elapsed =  now_tick -   flip_start_time;

        /* * 还没翻到 275°：* 继续保持大差速。*/
        if (
            flip_accum_angle <   FLIP_BRAKE_START_DEG &&  elapsed < FLIP_TIMEOUT_MS)
        {
            Flip_Motor_Output( aircraft_fly_ctrl, flip_base_cmd, FLIP_DRIVE_DIFF);
            return;
        }

        /*  * 接近一圈后， * 进入反向刹车阶段。 */
        flip_state =  FLIP_BRAKE;
    }


    /* ================================================== */
    /* ================= 第三阶段：刹车 ================== */
    /* ================================================== */

    if (flip_state == FLIP_BRAKE)
    {
        uint32_t elapsed =  now_tick - flip_start_time;
        float roll_error;
        uint8_t flip_finished =   0;


        /*  * 当前 Roll 与起始 Roll 的差。
         *  * angle_limit_180 可以处理
         *  +180 / -180 跨界。
         */
        roll_error =  angle_limit_180( current_roll - flip_start_roll);
      
		/*
         * 三个条件同时满足：
         * 1. 至少已经累计翻过 325°
         * 2. 当前 Roll 回到起始姿态附近
         * 3. Roll Rate 已经降下来
         * * 才认为真正完成一圈。
         */
        if ( flip_accum_angle >= FLIP_FINISH_MIN_DEG &&  fabsf(roll_error) <= FLIP_FINISH_ROLL_ERR_DEG &&  fabsf(roll_rate) <=  FLIP_FINISH_RATE_DPS)
        {
            flip_finished =  1;
        }

        /** 超时保险。 * 防止因为 IMU 异常，* 一直保持翻滚差速。*/
        if ( elapsed >=  FLIP_TIMEOUT_MS)
        {
            flip_finished =  1;
        }


        if (!flip_finished)
        {
            /* * 反向小差速** 主翻滚：
             * left ↑
             * right ↓
             *
             * 刹车：
             * left ↓
             * right ↑
             */
            Flip_Motor_Output(  aircraft_fly_ctrl,  flip_base_cmd,  -FLIP_BRAKE_DIFF);
            return;
        }

        flip_state =   FLIP_RECOVER;
        flip_recover_start_time =   now_tick;
    }

    /* ================================================== */
    /* ================= 第四阶段：恢复 ================== */
    /* ================================================== */

    if (flip_state == FLIP_RECOVER)
    {
        uint32_t recover_elapsed =  now_tick -  flip_recover_start_time;


        if ( recover_elapsed <  FLIP_RECOVER_MS)
        {   /** 两边同时加油，迅速恢复升力。**/
            Flip_Motor_Output(  aircraft_fly_ctrl,  flip_base_cmd +   FLIP_RECOVER_ADD,  0.0f);
            return;
        }


        /** 翻滚完成。
         * 清除 PID 历史，
         * 然后恢复普通控制*/
        Flip_Controller_Reset(
            aircraft_fly_ctrl);


        flip_state =
            FLIP_WAIT_RELEASE;
    }


    /* ================================================== */
    /* =================== 等待 F 松开 =================== */
    /* ================================================== */

    if (
        flip_state ==
        FLIP_WAIT_RELEASE)
    {
        /*
         * 这里不要 return。
         * 飞机已经恢复正常 PID。
         * 只是等待 F 松开后允许下一次触发。
         */
        if (
            aircraft_fly_ctrl
                ->remote_data
                ->F != 2)
        {
            flip_state =
                FLIP_IDLE;
        }
    }
}

            /* ============================================== */
            /* ==================== Yaw 外环 ================= */
            /* ============================================== */

            if ( aircraft_fly_ctrl->yaw_set_angle >    180.0f)
            {
                aircraft_fly_ctrl->yaw_set_angle -=   360.0f;
            }


            if ( aircraft_fly_ctrl->yaw_set_angle <   -180.0f)
            {
                aircraft_fly_ctrl->yaw_set_angle +=   360.0f;
            }

            Yaw_Rate_Set =   yaw_PID_calc(    &PID_control_yaw,   aircraft_fly_ctrl   ->IMU_data   ->INS_angle[0],  aircraft_fly_ctrl ->yaw_set_angle);

            /* ============================================== */
            /* ==================== 姿态外环 ================= */
            /* ============================================== */

            aircraft_fly_ctrl->Roll_Rate_Set =   PID_calc(  &PID_control_roll,   aircraft_fly_ctrl  ->IMU_data   ->INS_angle[2],  target_roll);
            aircraft_fly_ctrl->Pitch_Rate_Set =  PID_calc(  &PID_control_pitch,  aircraft_fly_ctrl  ->IMU_data   ->INS_angle[1],  target_pitch);

            /* ============================================== */
            /* ==================== 角速度内环 =============== */
            /* ============================================== */

            if (
                aircraft_fly_ctrl->aircraft_behaviour ==
                Aircraft_RUN_ADRC)
            {
                /*
                 * Roll ADRC
                 */
                roll_inner_out =
                    ADRC_1st_Calc(
                        &ADRC_rate_roll,

                        aircraft_fly_ctrl
                            ->Roll_Rate_Set,

                        -aircraft_fly_ctrl
                            ->IMU_data
                            ->INS_gyro[2]);


                /*
                 * Pitch ADRC
                 */
                pitch_inner_out =
                    ADRC_1st_Calc(
                        &ADRC_rate_pitch,

                        aircraft_fly_ctrl
                            ->Pitch_Rate_Set,

                        -aircraft_fly_ctrl
                            ->IMU_data
                            ->INS_gyro[0]);


                /*
                 * Yaw ADRC
                 */
                yaw_inner_out =
                    ADRC_1st_Calc(
                        &ADRC_rate_yaw,

                        Yaw_Rate_Set,

                        -aircraft_fly_ctrl
                            ->IMU_data
                            ->INS_gyro[1]);
            }

            else
            {
                /*  * Roll PID  */
                PID_calc(&PID_rate_roll,-aircraft_fly_ctrl->IMU_data->INS_gyro[2],aircraft_fly_ctrl->Roll_Rate_Set);

                /*  * Pitch PID */
                PID_calc(&PID_rate_pitch,-aircraft_fly_ctrl->IMU_data->INS_gyro[0],aircraft_fly_ctrl->Pitch_Rate_Set);

                /*  * Yaw PID   */
                PID_calc(  &PID_rate_yaw,  -aircraft_fly_ctrl   ->IMU_data   ->INS_gyro[1],   Yaw_Rate_Set);

                roll_inner_out  =  PID_rate_roll.out;
                pitch_inner_out =  PID_rate_pitch.out;
                yaw_inner_out   =  PID_rate_yaw.out;
            }


            /* ============================================== */
            /* ============== 左右电机 Roll 混控 ============ */
            /* ============================================== */

            /*
             * 正负 Roll 使用不同差速比例，
             * 用于补偿左右机构响应差异。
             */
            if (roll_inner_out > 0.0f)
            {
                aircraft_fly_ctrl->motor_left_out =compensated_thrust + roll_inner_out * 0.7f;
                aircraft_fly_ctrl->motor_right_out =compensated_thrust - roll_inner_out * 0.3f;
            }
            else
            {
                aircraft_fly_ctrl->motor_left_out =compensated_thrust + roll_inner_out * 0.3f;
                aircraft_fly_ctrl->motor_right_out =compensated_thrust - roll_inner_out * 0.7f;
            }

            /* ============================================== */
            /* ================= Roll 自动 Trim ============== */
            /* ============================================== */

            Roll_Motor_Trim_Update( aircraft_fly_ctrl,  target_roll,  target_pitch,  compensated_thrust);

            aircraft_fly_ctrl->motor_left_out +=  roll_motor_trim;
            aircraft_fly_ctrl->motor_right_out -=    roll_motor_trim;


            /* ============================================== */
            /* =================== 电机限幅 ================== */
            /* ============================================== */

            aircraft_fly_ctrl->motor_left_out =  (float)num_limit(   aircraft_fly_ctrl  ->motor_left_out,  0.0f,    900.0f);
            aircraft_fly_ctrl->motor_right_out =  (float)num_limit(  aircraft_fly_ctrl  ->motor_right_out,   0.0f,  900.0f);


            /* ============================================== */
            /* ================= 气压高度环预留 ============== */
            /* ============================================== */

            altitude =spl06_inst.filtered_altitude -altitude_bias;
            PID_calc(&PID_control_altitude,altitude,altitude_aim);
            PID_control_altitude.out =(float)num_limit(PID_control_altitude.out,-50.0f,100.0f);

            if (altitude < 0.3f)   {  altitude_motor = 0.0f; }
            else   {altitude_motor =PID_control_altitude.out * 100.0f; }

            /** 当前 altitude_motor  * 尚未叠加到电机输出。*/
            /* ============================================== */
            /* ================= 最终电机 PWM ================ */
            /* ============================================== */
                Motor_Output_Write( aircraft_fly_ctrl);
             /* ============================================== */
            /* ==================== 舵机混控 ================= */
            /* ============================================== */

            {   float current_throttle_ratio;
                float pitch_control_val;
                float yaw_control_val;
                current_throttle_ratio =   compensated_thrust /  800.0f;

                /** 低油门时防止 Pitch 控制被过度放大。*/
                if (  current_throttle_ratio <  0.4f)  
				{
                    current_throttle_ratio =  0.4f;
                }


                /*
                 * =================================================
                 * Pitch 控制
                 * =================================================
                 * 注意这里改为 pitch_inner_out。
                 * 因此：RUN/Auto    -> PID 输出
                 *       RUN_ADRC    -> ADRC 输出
                 */
                pitch_control_val =  (-pitch_inner_out / current_throttle_ratio) + pitch_bias;
                /* =================================================
                 * Yaw 控制
                 * =================================================
                 * 同样统一使用 yaw_inner_out。 */
                yaw_control_val =	-yaw_inner_out;
                /** 舵机在上。** Pitch：* 左右差动** Yaw：* 左右同向   */
                aircraft_fly_ctrl->servo_left_out =   860.0f -   pitch_control_val +   yaw_control_val;
                aircraft_fly_ctrl->servo_right_out =  940.0f +   pitch_control_val +  yaw_control_val;
                 /** 舵机限幅。 */
                aircraft_fly_ctrl->servo_left_out =   (float)num_limit(  aircraft_fly_ctrl  ->servo_left_out,  500.0f,  1300.0f);
                aircraft_fly_ctrl->servo_right_out =  (float)num_limit(  aircraft_fly_ctrl  ->servo_right_out, 500.0f,  1300.0f);
                /** TIM5 CH2 -> 左舵机**/
                __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,(uint32_t)aircraft_fly_ctrl->servo_left_out);
                /** TIM5 CH1 -> 右舵机**/
                __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,(uint32_t)aircraft_fly_ctrl->servo_right_out);
                /** 调试变量*/
                set_yaw =  (int)aircraft_fly_ctrl->yaw_set_angle;
                now_yaw =  (int)PID_control_yaw.fdb;
            }
        }
    }
}
 