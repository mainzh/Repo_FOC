#include "dwt_delay.h"
#include "main.h"
// 全局变量：存储系统时钟频率（单位：Hz），默认170MHz（STM32G474默认系统时钟）
static uint32_t g_sysclk_freq = 170000000;

/**
 * @brief  DWT周期计数器初始化
 * @details 使能CYCCNT计数器，并读取系统时钟频率（适配不同时钟配置）
 */
uint8_t DWT_Delay_Init(void)
{
    // 1. 使能DWT外设（需先解锁CoreDebug）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能跟踪单元
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;         // 先清零使能位
    DWT->CYCCNT = 0;                             // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;          // 使能CYCCNT计数器

    // 2. 验证初始化结果（检查CYCCNT是否开始计数）
    if (DWT->CYCCNT == 0)
    {
        // 读取系统时钟频率（从RCC寄存器获取，适配实际配置）
        RCC_ClkInitTypeDef clk_init_struct = {0};
        HAL_RCC_GetClockConfig(&clk_init_struct, NULL);
        g_sysclk_freq = HAL_RCC_GetSysClockFreq();  // 获取实际SYSCLK频率
        return 0;
    }
    return 1;  // 初始化失败（可能调试接口被禁用）
}

/**
 * @brief  微秒级延时实现
 * @details 计算目标计数值 = 当前计数值 + us×系统时钟频率(MHz)，循环等待计数器达到目标值
 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t target_cnt = 0;
    uint32_t start_cnt = 0;

    // 检查DWT是否已初始化（CYCCNT使能位是否置1）
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        return;  // DWT未初始化，直接返回
    }

    start_cnt = DWT->CYCCNT;  // 记录延时开始时的计数器值
    // 计算目标计数值：us × (g_sysclk_freq / 1000000) = us × 系统时钟频率(MHz)
    target_cnt = start_cnt + (us * ((g_sysclk_freq / 1000000)/20));

    // 等待计数器达到目标值（处理溢出情况：若target_cnt溢出，先等CYCCNT到最大值，再等至target_cnt）
    if (target_cnt < start_cnt)
    {
        while (DWT->CYCCNT < 0xFFFFFFFF);  // 等待计数器溢出清零
    }
    while (DWT->CYCCNT < target_cnt);
}
