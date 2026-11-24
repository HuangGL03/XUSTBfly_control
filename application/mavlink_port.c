/**
  ********************************************************
  * @file       mavlink_port.c
  * @brief      通用 MAVLink 串口端口实现(见 mavlink_port.h 说明)。
  ********************************************************
  */
#include "mavlink_port.h"

HAL_StatusTypeDef MavPort_Start(MavlinkPort_t *port)
{
    port->tail = 0;
    port->dma_wraps = 0;
    port->consumed = 0;
    mavlink_reset_channel_status(port->chan);

    /* circular 模式启动一次即长期运行,TC 回调只累计圈数。 */
    return HAL_UART_Receive_DMA(port->huart,
                                        port->dma_buf,
                                        port->buf_size);
}

/* DMA 环形接收 + NDTR + 官方解析器
 * N 是环形缓冲容量，不是某条消息长度；N-NDTR 是本圈的写入位置。
 * produced=TC圈数*N+(N-NDTR)，consumed 是软件累计读量，二者差为待解析字节数。
 * tail 随每个字节递增并回绕，因此一帧跨缓冲末尾时仍连续喂同一 parser。
 * 若积压超过 N，较老字节已被覆盖：统计丢字节并重置残帧，而不是解析已损坏的前半帧。
 * 每条物理流使用自己的 port->chan；这里在任务上下文逐字节解析，ISR 不解码协议。 */
uint32_t MavPort_Drain(MavlinkPort_t *port)
{
    uint32_t wraps_before;
    uint32_t wraps_after;
    uint16_t remaining;

    /* TC ISR 可能恰好夹在读取回绕计数和 NDTR 之间,不一致时重读。 */
    do
    {
        wraps_before = port->dma_wraps;
        remaining = (uint16_t)__HAL_DMA_GET_COUNTER(port->huart->hdmarx);
        wraps_after = port->dma_wraps;
    } while (wraps_before != wraps_after);

    /* position 允许等于 buf_size:表示 NDTR 已到 0、TC ISR 尚未执行。 */
    uint32_t position = (uint32_t)port->buf_size - remaining;
    uint32_t produced = wraps_after * (uint32_t)port->buf_size + position;
    uint32_t available = produced - port->consumed;

    if (available > port->buf_size)
    {
        uint32_t dropped = available - port->buf_size;
        port->overrun_count++;
        port->dropped_bytes += dropped;
        port->consumed = produced - port->buf_size;
        port->tail = (uint16_t)(port->consumed % port->buf_size);
        available = port->buf_size;

        /* 覆盖可能截断了一帧,丢弃 parser 中尚未完成的残帧。 */
        mavlink_reset_channel_status(port->chan);
    }

    uint32_t drained = available;

    while (available-- > 0U)
    {
        uint8_t byte = port->dma_buf[port->tail];

        if (mavlink_parse_char(port->chan, byte, &port->msg, &port->status))
        {
            if (port->on_message != NULL)
            {
                port->on_message(&port->msg);
            }
        }

        if (++port->tail >= port->buf_size)
        {
            port->tail = 0;
        }
        port->consumed++;
    }

    return drained;
}
