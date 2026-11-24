#pragma once

/* 光流接收任务：从 USART2 的 DMA 环形缓冲区读取并解析报文。 */
void flow_rx_task(void const *argument);
/* USART2 接收完成中断入口，每次记录一次 DMA 回绕。 */
void flow_rx_wrap_from_isr(void);
