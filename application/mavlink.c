/**
  ********************************************************
  * @file       mavlink.c/h
  * @brief      Ò£¿ØÆ÷Êı¾İ´¦Àí£¬½âÎöelrsµÄÒ£¿ØÆ÷Êı¾İ£¬Í¨¹ı´®¿ÚµÄ¿ÕÏĞÖĞ¶ÏÀ´´¦ÀíÊı¾İ£¬Ê¹ÓÃDMA
  *             À´»º½âCPUÑ¹Á¦¡£
  *             ºóĞøÓÅ»¯µÄ·½Ïò£º
  *             1.Ôö¼ÓDMAµÄ¶ÏÏßÖØÆô´¦Àí£¬´®¿ÚÖĞ¶ÏµôÏß´¦ÀíÀ´±£Ö¤ÈÈ²å°ÎµÄÎÈ¶¨ĞÔ
  *             2.Ê¹ÓÃDMAË«»º´æÇøÀ´±£Ö¤Êı¾İ´¦Àí²»»áÓµ¶Â³ö´í£¬Ä¿Ç°µ¥»º´æ²âÊÔÕı³££¨³öÎÊÌâ¿ÉÒÔ¹Ø×¢ÕâÀï£©
  *             3.µÚÒ»°æÊ¹ÓÃµÄÒ£¿ØÆ÷ÊÇRADIOMASTER POCKET,ÏÂÃæ×¢ÊÍÖĞÍ¨µÀÓëÒ£¸Ğ²»Ò»¶¨¶ÔÓ¦£¬ºóĞøÊ¹ÓÃÕß×Ô¼º
	*             È·ÈÏ±ê¶¨
	*             
	*             1.0.1°æ±¾¸üĞÂ£º½â¾öÁËÖ®Ç°Ò£¿ØÆ÷ĞèÒªÈí¸´Î»²ÅÄÜÊ¹ÓÃµÄÎÊÌâ£¬½â¾ö·½°¸ÊÇÔÚGPIOÒÔ¼°UARTºÍDMA³õÊ¼»¯Ö®Ç°
	*             ÊÖ¶¯¼ÓÁËÒ»¸ö5ÃëÑÓÊ±£¬²»ºÏÀíµ«ÓĞĞ§£¬²Â²âÊÇÒòÎªÔÚ³õÊ¼»¯´®¿ÚÊ±²»Õı³£Êı¾İ½øÈëµ¼ÖÂÕû¸ö´®¿Ú¹ÒµôÁË
	*
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     2025.10.16        ÕÅÕ×²©          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ********************************************************
  */
#include "common/mavlink.h"
#include "mavlink_types.h"
#include "usart.h"
#include <math.h>
#include "flow_rx.h"
#include "mavlink_flow.h"
#include "ringbuffer.h"//»·ĞÎ»º³åÇøÍ·ÎÄ¼ş
#include "stdio.h"
#define  MAVLINK_READ_BUFFER_SIZE 128
#include "cmsis_os.h"
#include "string.h"
#include "Aircraft_ctrl_task.h"
#define M_PI 3.14159265358979323846f
//#include "mavlink_msg_rc_channels.h"
/*»º³åÇø¹ÜÀíÆ÷*/
//ringbuffer¹ÜÀí±äÁ¿
RingBuffer  m_Mavlink_RX_RingBuffMgr;
/*MAVLINK½ÓÊÕÊı¾İ»º³åÇøÊı×é*/
uint8_t   m_Mavlink_RX_Buff[MAVLINK_READ_BUFFER_SIZE*10];
uint8_t mavlink_byte ;
mavlink_rc_channels_override_t packet;//Ò£¿ØÆ÷Êı¾İ½á¹¹Ìå
mavlink_set_attitude_target_t packet2;//×ËÌ¬Êı¾İ½á¹¹Ìå
ELRS_Data elrs_data;
PC_Data_t pc_data;
int mavlink_num_test=0;//pc
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          »ñÈ¡Ò£¿ØÆ÷Êı¾İÖ¸Õë
  * @param[in]      none
  * @retval         Ò£¿ØÆ÷Êı¾İÖ¸Õë
  */
const ELRS_Data *get_remote_control_point(void)
{
    return &elrs_data;
}
/**
  * @brief          »ñÈ¡ÉÏÎ»»úÊı¾İÖ¸Õë
  * @param[in]      none
  * @retval         ÉÏÎ»»úÊı¾İÖ¸Õë
  */
const PC_Data_t *get_pc_data_point(void)
{
    return &pc_data;
}

void PC_Data_Clear(void)
{
    memset(&pc_data, 0, sizeof(pc_data));
}

float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return float_Map(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return float_Map(input_value, median, input_max, output_median, output_max);
    }
}
/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Quaternion_to_Euler_Transform(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ½«ÉÏÎ»»ú·¢À´µÄËÄÔªÊı×ª»»ÎªÅ·À­½Ç£¬Í¬Ê±½«ĞÅÏ¢´«Èë½á¹¹Ìå
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Quaternion_to_Euler_Transform(mavlink_set_attitude_target_t* packet,PC_Data_t* pc_data)
{ 
    float siny_cosp=0;
    float cosy_cosp=0;
    float sinp=0;
    float cosp=0;
    siny_cosp = 2.0f * (packet->q[0] * packet->q[3] + packet->q[1] * packet->q[2]);
    cosy_cosp = 1.0f - 2.0f * (packet->q[2] * packet->q[2] + packet->q[3] * packet->q[3]);
    pc_data->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
    sinp = 2.0f * (packet->q[0] * packet->q[1] + packet->q[2] * packet->q[3]); // ÕâÊÇ±ê×¼Roll(x)µÄ¹«Ê½
    cosp = 1.0f - 2.0f * (packet->q[1] * packet->q[1] + packet->q[2] * packet->q[2]);
    pc_data->pitch = atan2f(sinp, cosp) * (180.0f / M_PI);  
    sinp = 2.0f * (packet->q[0] * packet->q[2] - packet->q[3] * packet->q[1]); // ÕâÊÇ±ê×¼Pitch(y)µÄ¹«Ê½
    if (fabsf(sinp) >= 1.0f) 
{
     pc_data->roll = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI); 
} 
    else 
{
     pc_data->roll = asinf(sinp) * (180.0f / M_PI);
}
   pc_data->thrust = packet->thrust;//´«ÈëÓÍÃÅ
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlink_RB_Init(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ³õÊ¼»¯Ò»¸öÑ­»·¶ÓÁĞ£¬ÓÃÀ´¹ÜÀí½ÓÊÕµ½µÄ´®¿ÚÊı¾İ¡£ÆäÊµ¾ÍÊÇÒ»¸öÊı¾İ»º³åÇø
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Init(void)
{
	//½«m_Mavlink_RX_Buffm_Mavlink_RX_RingBuffMgr»·¶ÓÁĞ½øĞĞ¹ØÁª¹ÜÀí¡£
	rbInit(&m_Mavlink_RX_RingBuffMgr, m_Mavlink_RX_Buff, sizeof(m_Mavlink_RX_Buff));
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlink_RB_Clear(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ¹éÁãringbufferÀïÃæµÄÉèÖÃ¡£
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Clear(void)
{
	rbClear(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: uint8_t  Mavlink_RB_IsOverFlow(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:  Òç³öÎª1£¬·´Ö®Îª0
* ¹¦    ÄÜ: ÅĞ¶Ï»º³åÆ÷ÊÇ·ñÒç³ö
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
uint8_t  Mavlink_RB_IsOverFlow(void)
{
	return m_Mavlink_RX_RingBuffMgr.flagOverflow;
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlink_RB_Push(uint8_t data)
* ÊäÈë²ÎÊı: data£º´ıÑ¹ÈëµÄÊı¾İ
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ½«½ÓÊÕµÄÊı¾İÑ¹Èë»º³åÇø
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Push(uint8_t data)
{
	rbPush(&m_Mavlink_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: uint8_t Mavlink_RB_Pop(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı: uint8_t ¶Á³öµÄÊı¾İ
* ¹¦    ÄÜ: ´Ó»º³åÆ÷¶Á³öÊı¾İ
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_Pop(void)
{
	return rbPop(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: uint8_t Mavlink_RB_HasNew(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ÅĞ¶ÏÊÇ·ñÓĞĞÂµÄÊı¾İ
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_HasNew(void)
{
	return !rbIsEmpty(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: uint16_t Mavlink_RB_Count(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ÅĞ¶ÏÓĞ¶àÉÙ¸öĞÂÊı¾İ
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
uint16_t Mavlink_RB_Count(void)
{
	return rbGetCount(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlink_Rece_Enable(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: Ê¹ÄÜDMA½ÓÊÕ
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlink_Rece_Enable(void)
{
	HAL_UART_Receive_IT(&huart3, &mavlink_byte, 1);
}
/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlink_Init(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ³õÊ¼»¯MAVLINK£ºÊ¹ÄÜ½ÓÊÕ£¬ringbuffer¹ØÁª
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlink_Init(void)
{
	Mavlink_Rece_Enable();
	Mavlink_RB_Init();
}
/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlin_RX_InterruptHandler(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ´®¿ÚÖĞ¶ÏµÄ´¦Àíº¯Êı£¬Ö÷ÒªÊÇ½²Êı¾İÑ¹Èëringbuffer¹ÜÀíÆ÷
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlin_RX_InterruptHandler(void)
{
	//Êı¾İÑ¹Èë
	rbPush(&m_Mavlink_RX_RingBuffMgr,mavlink_byte);
}
/*ÔÚ¡°mavlink_helpers.hÖĞĞèÒªÊ¹ÓÃ¡±*/
mavlink_system_t mavlink_system =
{
	1,
	1
}; // System ID, 1-255, Component/Subsystem ID, 1-255
/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ÖØĞÂ¶¨ÒåmavlinkµÄ·¢ËÍº¯Êı£¬Óëµ×²ãÓ²¼ş½Ó¿Ú¹ØÁªÆğÀ´
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)ch, length, 2000);
}
/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: void Mavlink_Msg_Handle(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ´¦Àí´ÓQGCÉÏÎ»»ú´«¹ıÀ´µÄÊı¾İĞÅÏ¢
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
void Mavlink_Msg_Handle(mavlink_message_t msg)
{
	switch(msg.msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
		//printf("this is heartbeat from QGC/r/n");
		break;
	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        mavlink_msg_set_attitude_target_decode(&msg, &packet2);//½âÎöÊı¾İ
        Quaternion_to_Euler_Transform(&packet2,&pc_data);//´¦Àí´«ÈëÊı¾İ
	    mavlink_num_test++;
		break;
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:  /* ID 84£º»úÌåÏµË®Æ½ËÙ¶ÈÖ¸Áî */
    {
        mavlink_set_position_target_local_ned_t cmd;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &cmd);
        if ((cmd.target_system != 0 && cmd.target_system != mavlink_system.sysid) ||
            (cmd.target_component != 0 && cmd.target_component != mavlink_system.compid)) {
            break;
        }
        /* 8 ºÍ 9 µÄËÙ¶ÈÖá¾ùÎªÇ°¡¢ÓÒ¡¢ÏÂ£»vx¡¢vy ±ØĞëÍ¬Ê±ÆôÓÃ¡£ */
        if ((cmd.coordinate_frame != MAV_FRAME_BODY_NED && cmd.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) ||
            (cmd.type_mask & (POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE)) ||
            !isfinite(cmd.vx) || !isfinite(cmd.vy)) {
            break;
        }
        taskENTER_CRITICAL();
        pc_data.vx_set_mps = cmd.vx;
        pc_data.vy_set_mps = cmd.vy;
        pc_data.velocity_update_ms = HAL_GetTick();
        ++pc_data.velocity_rx_count;
        taskEXIT_CRITICAL();
        break;
    }
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
	{
		mavlink_msg_rc_channels_override_decode(&msg, &packet);
	elrs_data.channels[0] = packet.chan1_raw; // Roll (Í¨³£)
    elrs_data.channels[1] = packet.chan2_raw; // Pitch (Í¨³£)
    elrs_data.channels[2] = packet.chan3_raw; // Throttle (Í¨³£)
    elrs_data.channels[3] = packet.chan4_raw; // Yaw (Í¨³£)
    elrs_data.channels[4] = packet.chan5_raw; // Aux1
    elrs_data.channels[5] = packet.chan6_raw; // Aux2
    elrs_data.channels[6] = packet.chan7_raw; // Aux3
    elrs_data.channels[7] = packet.chan8_raw; // Aux4
    elrs_data.channels[8]  = packet.chan9_raw;
    elrs_data.channels[9]  = packet.chan10_raw;
    elrs_data.channels[10] = packet.chan11_raw;
    elrs_data.channels[11] = packet.chan12_raw;
    elrs_data.channels[12] = packet.chan13_raw;
    elrs_data.channels[13] = packet.chan14_raw;
    elrs_data.channels[14] = packet.chan15_raw;
    elrs_data.channels[15] = packet.chan16_raw;
	elrs_data.Left_X = float_Map_with_median(elrs_data.channels[3], 1000, 2000, 1500, -100, 100);
    elrs_data.Left_Y = float_Map_with_median(elrs_data.channels[2], 1000, 2000, 1500, 0, 100);
    elrs_data.Right_X = float_Map_with_median(elrs_data.channels[0], 1000, 2000, 1500, -100, 100);
    elrs_data.Right_Y = float_Map_with_median(elrs_data.channels[1], 1000, 2000, 1500, -100, 100);
    elrs_data.S1 = float_Map_with_median(elrs_data.channels[8], 1000, 2000, 1500, 0, 100);
    elrs_data.S2 = float_Map_with_median(elrs_data.channels[9], 1000, 2000, 1500, 0, 100);
    elrs_data.A = elrs_data.channels[10] > 1500 ? 1 : 0;
    elrs_data.B = (elrs_data.channels[5] < 1200) ? 0 : ((elrs_data.channels[5] > 1800) ? 2 : 1);
    elrs_data.C = (elrs_data.channels[6] < 1200) ? 0 : ((elrs_data.channels[6] > 1800) ? 2 : 1);
    elrs_data.D = elrs_data.channels[11] > 1500 ? 1 : 0;
    elrs_data.E = (elrs_data.channels[4] < 1200) ? 1 : ((elrs_data.channels[4] > 1800) ? 2 : 0);
    elrs_data.F = (elrs_data.channels[7] < 1200) ? 1 : ((elrs_data.channels[7] > 1800) ? 2 : 0);
	}
	break;
	default:
		break;
	}
}
/*******************************º¯ÊıÉùÃ÷****************************************
* º¯ÊıÃû³Æ: Loop_Mavlink_Parse(void)
* ÊäÈë²ÎÊı:
* ·µ»Ø²ÎÊı:
* ¹¦    ÄÜ: ÔÚmainº¯ÊıÖĞ²»¼ä¶Ïµ÷ÓÃ´Ëº¯Êı
* ×÷    Õß: by Across
* ÈÕ    ÆÚ: 2018/06/02
*******************************************************************************/
mavlink_message_t msg;
mavlink_status_t status;

void Loop_Mavlink_Parse(void)
{
	if(Mavlink_RB_IsOverFlow())
	{
		Mavlink_RB_Clear();
	}
    static uint16_t last_drop_count = 0;
	while(Mavlink_RB_HasNew())
	{
		uint8_t read = Mavlink_RB_Pop();
		uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, read, &msg, &status);
		if (status.packet_rx_drop_count > last_drop_count)
        {
			last_drop_count = status.packet_rx_drop_count; // ¸üĞÂ¼ÆÊıÆ÷
        }
		if(result)
        {
            // ĞÅºÅ´¦Àíº¯Êı
            Mavlink_Msg_Handle(msg);
        }
		
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        flow_rx_wrap_from_isr();
        return;
    }

	if(huart == &huart3)
	{
		Mavlin_RX_InterruptHandler();
		HAL_UART_Receive_IT(&huart3, &mavlink_byte, 1);//ÖØÆôÖĞ¶Ï
	}

}
/* Í¬Ò»×é×ËÌ¬ºÍ½ÇËÙ¶ÈÓÉ INS Ô­×Ó·¢²¼¡£½âËãµÄÕı·½ÏòÎª×ó¹ö¡¢Ç°Çã£¬
 * ×ªµ½ MAVLink Ç°ÓÒÏÂ»úÌåÏµÊ±£¬ËÄÔªÊıÏòÁ¿²¿ºÍ½ÇËÙ¶È¾ù·´×ª X¡¢Y¡£ */
static void send_attitude_quaternion(void)
{
    static const float repr_offset_q[4] = {0};  /* ÎŞ¶îÍâÏÔÊ¾Ğı×ª */
    FlowAttitude_t attitude;
    if (!ins_get_flow_attitude(&attitude) ||
        (uint32_t)(HAL_GetTick() - attitude.update_ms) > 100U) {
        return;
    }
    mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, attitude.update_ms,
        attitude.quat[0], -attitude.quat[1], -attitude.quat[2], attitude.quat[3],
        -attitude.gyro_rads[0], -attitude.gyro_rads[1], attitude.gyro_rads[2],
        repr_offset_q);
}

void mavlink_task(void const *pvParameters)
{
    static const char flow_name[10] = "FLOW_EST";  /* ID 250 ¹Ì¶¨¿í¶ÈÃû³Æ */
    uint32_t last_telemetry_ms = HAL_GetTick();
    uint32_t last_heartbeat_ms = last_telemetry_ms - 500U;  /* Æô¶¯Á¢¼´·¢ĞÄÌø */
    while (1)
    {
        Loop_Mavlink_Parse();
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - last_heartbeat_ms;
        if (elapsed >= 500U) {
            last_heartbeat_ms += elapsed - elapsed % 500U;
            /* ¹Ì¶¨±êÊ¶£ºÆËÒí·ÉĞĞÆ÷¡¢Í¨ÓÃ·É¿Ø£¬Ä£Ê½Îª 0£¬×´Ì¬ÌîÎ´Öª¡£ */
            mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_FLAPPING_WING,
                MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_UNINIT);
        }

        elapsed = now - last_telemetry_ms;
        if (elapsed >= 50U) {
            last_telemetry_ms += elapsed - elapsed %50U;
            send_attitude_quaternion();
            OpticalFlow_Data_t flow;
            uint32_t snapshot_ms = HAL_GetTick();
            mavlink_flow_get_snapshot(&flow, snapshot_ms);
            /* ID 250£ºÂË²¨ºóµÄÇ°Ïò¡¢ÓÒÏòËÙ¶È (m/s) ºÍÀëµØ¸ß¶È (m)¡£ */
            mavlink_msg_debug_vect_send(MAVLINK_COMM_0, flow_name, (uint64_t)snapshot_ms * 1000U,
                flow.vx_body_mps, flow.vy_body_mps, flow.height_m);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


























///**
//  ********************************************************
//  * @file       mavlink.c/h
//  * @brief      ???????,??elrs??????,??????????????,??DMA
//  *             ???CPU???
//  *             ???????:
//  *             1.??DMA???????,??????????????????
//  *             2.??DMA?????????????????,?????????(?????????)
//  *             3.??????????RADIOMASTER POCKET,???????????????,???????
//	*             ????
//	*             
//	*             1.0.1????:????????????????????,??????GPIO??UART?DMA?????
//	*             ??????5???,??????,????????????????????????????
//	*
//  * @note       
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.1     2025.10.16        ???          1. done
//  *
//  @verbatim
//  ==============================================================================

//  ==============================================================================
//  @endverbatim
//  ********************************************************
//  */
//#include "mavlink.h"
//#include "mavlink_types.h"
//#include "usart.h"
//#include "ringbuffer.h"//????????
//#include "stdio.h"
//#define  MAVLINK_READ_BUFFER_SIZE 128
//#include "cmsis_os.h"
//#include "string.h"
////#include "mavlink_msg_rc_channels.h"
///*??????*/
////ringbuffer????
//RingBuffer  m_Mavlink_RX_RingBuffMgr;
///*MAVLINK?????????*/
//uint8_t   m_Mavlink_RX_Buff[MAVLINK_READ_BUFFER_SIZE*10];
//uint8_t mavlink_byte ;
//mavlink_rc_channels_override_t packet;
//mavlink_message_t lastmsg;
//ELRS_Data elrs_data;
//extern float altitude;

///**
//  * @brief          get remote control data point
//  * @param[in]      none
//  * @retval         remote control data point
//  */
///**
//  * @brief          ?????????
//  * @param[in]      none
//  * @retval         ???????
//  */
//const ELRS_Data *get_remote_control_point(void)
//{
//    return &elrs_data;
//}
//float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
//{
//    float output_value;
//    if (input_value < input_min)
//    {
//        output_value = output_min;
//    }
//    else if (input_value > input_max)
//    {
//        output_value = output_max;
//    }
//    else
//    {
//        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
//    }
//    return output_value;
//}
//float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
//{
//    float output_median = (output_max - output_min) / 2 + output_min;
//    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
//    {
//        return output_min;
//    }

//    if (input_value < median)
//    {
//        return float_Map(input_value, input_min, median, output_min, output_median);
//    }
//    else
//    {
//        return float_Map(input_value, median, input_max, output_median, output_max);
//    }
//}
///*******************************????****************************************
//* ????: void Mavlink_RB_Init(void)
//* ????:
//* ????:
//* ?    ?: ?????????,????????????????????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlink_RB_Init(void)
//{
//	//?m_Mavlink_RX_Buffm_Mavlink_RX_RingBuffMgr??????????
//	rbInit(&m_Mavlink_RX_RingBuffMgr, m_Mavlink_RX_Buff, sizeof(m_Mavlink_RX_Buff));
//}

///*******************************????****************************************
//* ????: void Mavlink_RB_Clear(void)
//* ????:
//* ????:
//* ?    ?: ??ringbuffer??????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlink_RB_Clear(void)
//{
//	rbClear(&m_Mavlink_RX_RingBuffMgr);
//}

///*******************************????****************************************
//* ????: uint8_t  Mavlink_RB_IsOverFlow(void)
//* ????:
//* ????:  ???1,???0
//* ?    ?: ?????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//uint8_t  Mavlink_RB_IsOverFlow(void)
//{
//	return m_Mavlink_RX_RingBuffMgr.flagOverflow;
//}

///*******************************????****************************************
//* ????: void Mavlink_RB_Push(uint8_t data)
//* ????: data:??????
//* ????:
//* ?    ?: ???????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlink_RB_Push(uint8_t data)
//{
//	rbPush(&m_Mavlink_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
//}

///*******************************????****************************************
//* ????: uint8_t Mavlink_RB_Pop(void)
//* ????:
//* ????: uint8_t ?????
//* ?    ?: ????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//uint8_t Mavlink_RB_Pop(void)
//{
//	return rbPop(&m_Mavlink_RX_RingBuffMgr);
//}

///*******************************????****************************************
//* ????: uint8_t Mavlink_RB_HasNew(void)
//* ????:
//* ????:
//* ?    ?: ?????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//uint8_t Mavlink_RB_HasNew(void)
//{
//	return !rbIsEmpty(&m_Mavlink_RX_RingBuffMgr);
//}

///*******************************????****************************************
//* ????: uint16_t Mavlink_RB_Count(void)
//* ????:
//* ????:
//* ?    ?: ?????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//uint16_t Mavlink_RB_Count(void)
//{
//	return rbGetCount(&m_Mavlink_RX_RingBuffMgr);
//}

///*******************************????****************************************
//* ????: void Mavlink_Rece_Enable(void)
//* ????:
//* ????:
//* ?    ?: ??DMA??
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlink_Rece_Enable(void)
//{
//	HAL_UART_Receive_IT(&huart3, &mavlink_byte, 1);
//}
///*******************************????****************************************
//* ????: void Mavlink_Init(void)
//* ????:
//* ????:
//* ?    ?: ???MAVLINK:????,ringbuffer??
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlink_Init(void)
//{
//	Mavlink_Rece_Enable();
//	Mavlink_RB_Init();
//}
///*******************************????****************************************
//* ????: void Mavlin_RX_InterruptHandler(void)
//* ????:
//* ????:
//* ?    ?: ?????????,????????ringbuffer???
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlin_RX_InterruptHandler(void)
//{
//	//????
//	rbPush(&m_Mavlink_RX_RingBuffMgr,mavlink_byte);
//}
///*?“mavlink_helpers.h?????”*/
//mavlink_system_t mavlink_system =
//{
//	1,
//	1
//}; // System ID, 1-255, Component/Subsystem ID, 1-255
///*******************************????****************************************
//* ????: void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
//* ????:
//* ????:
//* ?    ?: ????mavlink?????,???????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
//{
//	HAL_UART_Transmit(&huart3, (uint8_t *)ch, length, 2000);
//}
///*******************************????****************************************
//* ????: void Mavlink_Msg_Handle(void)
//* ????:
//* ????:
//* ?    ?: ???QGC???????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//void Mavlink_Msg_Handle(mavlink_message_t msg)
//{
//	switch(msg.msgid)
//	{
//	case MAVLINK_MSG_ID_HEARTBEAT:
//		//printf("this is heartbeat from QGC/r/n");
//		break;
//	case MAVLINK_MSG_ID_SYS_STATUS:
////		  osd_vbat = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
////			osd_curr = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)
////			osd_battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
//		break;
//	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
//	{
//		mavlink_msg_rc_channels_override_decode(&msg, &packet);
//	elrs_data.channels[0] = packet.chan1_raw; // Roll (??)
//    elrs_data.channels[1] = packet.chan2_raw; // Pitch (??)
//    elrs_data.channels[2] = packet.chan3_raw; // Throttle (??)
//    elrs_data.channels[3] = packet.chan4_raw; // Yaw (??)
//    elrs_data.channels[4] = packet.chan5_raw; // Aux1
//    elrs_data.channels[5] = packet.chan6_raw; // Aux2
//    elrs_data.channels[6] = packet.chan7_raw; // Aux3
//    elrs_data.channels[7] = packet.chan8_raw; // Aux4
//    elrs_data.channels[8]  = packet.chan9_raw;
//    elrs_data.channels[9]  = packet.chan10_raw;
//    elrs_data.channels[10] = packet.chan11_raw;
//    elrs_data.channels[11] = packet.chan12_raw;
//    elrs_data.channels[12] = packet.chan13_raw;
//    elrs_data.channels[13] = packet.chan14_raw;
//    elrs_data.channels[14] = packet.chan15_raw;
//    elrs_data.channels[15] = packet.chan16_raw;
//	elrs_data.Left_X = float_Map_with_median(elrs_data.channels[3], 1000, 2000, 1500, -100, 100);
//    elrs_data.Left_Y = float_Map_with_median(elrs_data.channels[2], 1000, 2000, 1500, 0, 100);
//    elrs_data.Right_X = float_Map_with_median(elrs_data.channels[0], 1000, 2000, 1500, -100, 100);
//    elrs_data.Right_Y = float_Map_with_median(elrs_data.channels[1], 1000, 2000, 1500, -100, 100);
//    elrs_data.S1 = float_Map_with_median(elrs_data.channels[8], 1000, 2000, 1500, 0, 100);
//    elrs_data.S2 = float_Map_with_median(elrs_data.channels[9], 1000, 2000, 1500, 0, 100);
//    elrs_data.A = elrs_data.channels[10] > 1500 ? 1 : 0;
//    elrs_data.B = (elrs_data.channels[5] < 1200) ? 0 : ((elrs_data.channels[5] > 1800) ? 2 : 1);
//    elrs_data.C = (elrs_data.channels[6] < 1200) ? 0 : ((elrs_data.channels[6] > 1800) ? 2 : 1);
//    elrs_data.D = elrs_data.channels[11] > 1500 ? 1 : 0;
//    elrs_data.E = (elrs_data.channels[4] < 1200) ? 1 : ((elrs_data.channels[4] > 1800) ? 2 : 0);
//    elrs_data.F = (elrs_data.channels[7] < 1200) ? 1 : ((elrs_data.channels[7] > 1800) ? 2 : 0);
//	}
//	break;
//	default:
//		break;
//	}
//}
///*******************************????****************************************
//* ????: Loop_Mavlink_Parse(void)
//* ????:
//* ????:
//* ?    ?: ?main???????????
//* ?    ?: by Across
//* ?    ?: 2018/06/02
//*******************************************************************************/
//mavlink_message_t msg;
//mavlink_status_t status;

//void Loop_Mavlink_Parse(void)
//{
//	if(Mavlink_RB_IsOverFlow())
//	{
//		Mavlink_RB_Clear();
//	}
//    static uint16_t last_drop_count = 0;
//	while(Mavlink_RB_HasNew())
//	{
//		uint8_t read = Mavlink_RB_Pop();
//		uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, read, &msg, &status);
//		if (status.packet_rx_drop_count > last_drop_count)
//        {
//			last_drop_count = status.packet_rx_drop_count; // ?????
//        }
//		if(result)
//        {
//            // ??????
//            Mavlink_Msg_Handle(msg);
//        }
//		
//	}
//}
//static void mavlink_test_heartbeat2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
//{
//#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
//	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
//	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
//	{
//		return;
//	}
//#endif
//	mavlink_message_t msg;
//	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//	uint16_t i;
//	mavlink_heartbeat_t packet_in =
//	{
//		963497464,17,84,151,218,3
//	};
//	mavlink_heartbeat_t packet1, packet2;
//	memset(&packet1, 0, sizeof(packet1));
//	packet1.custom_mode = packet_in.custom_mode;
//	packet1.type = packet_in.type;
//	packet1.autopilot = packet_in.autopilot;
//	packet1.base_mode = packet_in.base_mode;
//	packet1.system_status = packet_in.system_status;
//	packet1.mavlink_version = packet_in.mavlink_version;


//#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
//	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
//	{
//		// cope with extensions
//		memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
//	}
//#endif
//	memset(&packet2, 0, sizeof(packet2));
//	mavlink_msg_heartbeat_send(MAVLINK_COMM_0 , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
//	MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//	if(huart == &huart3)
//	{
//		Mavlin_RX_InterruptHandler();
//		HAL_UART_Receive_IT(&huart3, &mavlink_byte, 1);//????
//	}

//}
///*
//***********
//send message:float-altitude
//***********
//*/

//void mavlink_send_altitude(float altitude) {
//    mavlink_message_t msg;
//    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
//    uint16_t len;
//    
//    // ???????
//    uint32_t time_boot_ms = HAL_GetTick();
//    
//    // ??????,??"Altitude"????
//    mavlink_msg_named_value_float_pack(
//        mavlink_system.sysid,     // ??ID
//        mavlink_system.compid,    // ??ID
//        &msg,
//        time_boot_ms,
//        "Altitude",              // ????
//        altitude                 // ???
//    );
//    
//    // ?????????
//    len = mavlink_msg_to_send_buffer(buffer, &msg);
//    
//    // ??UART??
//    HAL_UART_Transmit(&huart3, buffer, len, 100);
//}

//void mavlink_task(void const *pvParameters)
//{
//	uint8_t send_divider = 0;  // 
//	 while (1)
//    {
//      Loop_Mavlink_Parse();
//	  mavlink_test_heartbeat2(1,1,&lastmsg);
//			send_divider++;
//        if (send_divider >= 100) {
//            send_divider = 0;
//						mavlink_send_altitude(altitude);
//				}
//	  vTaskDelay(10);
//	}
//}