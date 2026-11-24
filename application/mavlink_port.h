/* 循环 DMA 接收：完成中断累计回绕次数，任务根据剩余传输计数 NDTR 读取新增字节。 */
#ifndef __MAVLINK_PORT_H__
#define __MAVLINK_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "common/mavlink.h"

typedef struct
{
    UART_HandleTypeDef *huart;     /* 绑定串口,其 hdmarx 必须配成 DMA_CIRCULAR */
    mavlink_channel_t   chan;      /* MAVLINK_COMM_0/1/...,各端口唯一 */
    uint8_t            *dma_buf;   /* DMA 环形缓冲,须位于 DMA 可访问且缓存一致的区域 */
    uint16_t            buf_size;  /* 缓冲字节数 */
    uint16_t            tail;      /* 软件读指针(已喂给 parser 到此) */
    volatile uint32_t   dma_wraps; /* DMA TC 次数,由 ISR 更新 */
    uint32_t            consumed;  /* 任务累计消费字节数 */
    uint32_t            overrun_count;  /* 未读取的数据被 DMA 覆盖的累计次数 */
    uint32_t            dropped_bytes;  /* 因覆盖而丢失的累计字节数 */
    mavlink_message_t   msg;       /* 本端口独立的拼包结果 */
    mavlink_status_t    status;    /* 本端口独立的解析状态 */
    void (*on_message)(const mavlink_message_t *msg); /* 整帧派发回调 */
} MavlinkPort_t;

/**
  * @brief  启动端口 DMA 接收(circular,启动一次后长期运行)。
  */
HAL_StatusTypeDef MavPort_Start(MavlinkPort_t *port);

/**
  * @brief  排空 DMA 已生产的新字节并喂给 parser;只能在任务中调用。
  * @return 本次消费的字节数。
  */
uint32_t MavPort_Drain(MavlinkPort_t *port);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_PORT_H__ */
