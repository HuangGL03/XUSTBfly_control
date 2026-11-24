#ifndef SPL06_H
#define SPL06_H

#include "main.h" // 包含 HAL 库
#define SPL06_LOWPASS_ALPHA      0.2f    // 一阶低通平滑系数（0.1-0.3最佳）
#define SPL06_AVG_WINDOW        10       // 滑动平均窗口长度（10个值最优）

// --- SPL06 设备结构体 ---
typedef struct {
    // I2C 句柄
    I2C_HandleTypeDef *hi2c;
    // DMA 接收缓冲区 (必须 32 字节对齐以适配 STM32H7 D-Cache)
    uint8_t rx_buf[6] __attribute__((aligned(32)));
    // 状态标志位
    volatile uint8_t dma_busy;    // 1: DMA正在搬运中，不要发起新请求
    volatile uint8_t data_ready;  // 1: 数据已搬运到RAM，等待解算
    // 校准系数 (从寄存器读取)
    int16_t c0, c1, c01, c11, c20, c21, c30;
    int32_t c00, c10;
    // 缩放因子
    float kP; 
    float kT;
    // 最终数据
    int32_t raw_p;      // 原始气压 (调试用)
    float temperature;  // 摄氏度
    float pressure;     // 原始气压 (Pa)
    float altitude;     // 原始绝对高度 (m)
    // ========== 原有滤波字段 ==========
    float filtered_pressure; // 滤波后气压
    float filtered_altitude; // 滤波后高度
    float last_filtered_press; // 上一次滤波结果（一阶低通核心）
    // ========== 新增：滑动平均 + 基准气压 ==========
    float press_buf[SPL06_AVG_WINDOW]; // 滑动平均缓存
    uint8_t buf_idx;                   // 缓存索引
    uint8_t buf_full;                  // 缓存是否填满
    float ref_pressure;                // 校准后的基准气压（替代固定101325）
} SPL06_t;

// --- API 函数声明 ---
uint8_t SPL06_Init(SPL06_t *dev, I2C_HandleTypeDef *hi2c);
void SPL06_Start_Read_DMA(SPL06_t *dev);
void SPL06_Process_Data(SPL06_t *dev);
void SPL06_On_DMA_Complete(SPL06_t *dev); // 放在中断回调中
// ========== 替换原有滤波函数 + 新增校准函数 ==========
void SPL06_Apply_Combo_Filter(SPL06_t *dev); // 组合滤波（替代原一阶滤波）
void SPL06_Calibrate_Reference(SPL06_t *dev, float ref_alt); // 基准气压校准

#endif