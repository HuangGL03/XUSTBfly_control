#ifndef __AIRCRAFT_CTRL_TASK_H__
#define __AIRCRAFT_CTRL_TASK_H__
#include "INS_task.h"
#include "mavlink_flow.h"
#include "mavlink.h" 
#define RC_DEADBAND  5               //遥控器死区值
typedef enum
{
	
	Aircraft_ZERO_FORCE = 0, 
	Aircraft_InRing_P,    
	Aircraft_InRing_R,	
	Aircraft_OutRing_R,
	
	Aircraft_RUN_ADRC,//ADRC正常运行
	Aircraft_Auto,  //自动运行
	Aircraft_RUN,   //正常运行    
    
    Aircraft_FLOW_RUN,  /* B=1、C=0，光流水平控速与测距定高模式 */
} aircraft_behaviour_e;

typedef struct
{
	const ELRS_Data *remote_data;//遥控器api获取
	OpticalFlow_Data_t flow_data; /* 当前飞控周期的光流与测距快照，使用前分别检查速度和高度有效位。 */
	const IMU_data_t *IMU_data;//IMU数据实例化
	const PC_Data_t *pc_data;//上位机数据
	
	aircraft_behaviour_e aircraft_behaviour; //飞行器状态机初始化
    aircraft_behaviour_e last_aircraft_behaviour; //上一次飞行器状态机
    
	
	float Roll_Rate_Set;//期望roll角速度
	float Pitch_Rate_Set;//期望pitch角速度
	float motor_left_out;//左电机输出
	float motor_right_out;//右电机输出
//	float pitch_motor_out;//pitch舵机输出
	float servo_left_out;  // 左侧舵机 PWM 输出
    float servo_right_out; // 右侧舵机 PWM 输出	
//	float yaw_motor_out;//yaw舵机输出
	float yaw_set_angle;//yaw期望角度
} aircraft_control_t;
void Aircraft_ctrl_task(void const *pvParameters);
extern int set_yaw;
extern int now_yaw;

#endif