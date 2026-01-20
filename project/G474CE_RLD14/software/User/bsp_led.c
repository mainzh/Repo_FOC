#include "bsp_led.h"

/* LED初始化 */
void bsp_led_init(void)
{
    /* 使用STM32CubeMX生产STM32配置代码时，在 gpio.c 中初始化所有IO */
    //MX_GPIO_Init();
}

/* LED试验 */
void bsp_led_example(void)
{
      BSP_LED0(0);
      BSP_LED1(1);
      HAL_Delay(500);
      
      BSP_LED0(1);
      BSP_LED1(0);
      HAL_Delay(500);
}
