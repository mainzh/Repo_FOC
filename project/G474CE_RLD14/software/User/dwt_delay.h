#ifndef DWT_DELAY_H
#define DWT_DELAY_H

#include "stm32g4xx_hal.h"  // 依赖STM32F4 HAL库

/**
 * @brief  DWT周期计数器初始化
 * @retval uint8_t：0=初始化成功，1=初始化失败（CYCCNT未使能）
 * @note   使能DWT_CYCCNT计数器，需确保内核调试接口未禁用
 */
uint8_t DWT_Delay_Init(void);

/**
 * @brief  微秒级延时（基于DWT_CYCCNT计数）
 * @param  us：延时微秒数（最大支持约24.85秒，因32位计数器@168MHz最大计数值对应4294967295/168000000≈24.85s）
 * @retval 无
 * @note   需先调用DWT_Delay_Init()初始化
 */
void DWT_Delay_us(uint32_t us);

#endif /* DWT_DELAY_H */
