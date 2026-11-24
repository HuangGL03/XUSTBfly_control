#include "flow_rx.h"
#include "mavlink_port.h"
#include "mavlink_flow.h"
#include "cmsis_os.h"
#include "usart.h"

/* DMA1 无法访问 DTCM，512 字节的循环接收缓冲区放在 AXI SRAM；本工程未开启数据缓存。 */
static uint8_t rx_buffer[512] __attribute__((section(".ARM.__at_0x24000000"), aligned(32)));
/* USART2 的字节流使用独立的 MAVLink 通道 1，解出完整报文后交给光流模块。 */
static MavlinkPort_t port = {
    .huart = &huart2,
    .chan = MAVLINK_COMM_1,
    .dma_buf = rx_buffer,
    .buf_size = sizeof(rx_buffer),
    .on_message = mavlink_flow_dispatch
};

void flow_rx_wrap_from_isr(void)
{
    port.dma_wraps++;  /* DMA 每写满一圈触发一次完成中断，任务用圈数计算累计字节数 */
}

void flow_rx_task(void const *argument)
{
    (void)argument;
    for (;;) {
        if (MavPort_Start(&port) == HAL_OK) {
            while (huart2.RxState == HAL_UART_STATE_BUSY_RX) {
                MavPort_Drain(&port);
                osDelay(2);
            }
        }
        /* 启动失败或接收中止后，清理本次接收状态，再延时重试。 */
        HAL_UART_AbortReceive(&huart2);
        osDelay(10);
    }
}
