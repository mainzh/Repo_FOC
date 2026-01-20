#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "gpio.h"


/* 引脚定义 */
#define BSP_LED0_GPIO_Port    GPIOC
#define BSP_LED0_Pin          GPIO_PIN_13    /* IO置高电平，LED灭 */
#define BSP_LED1_GPIO_Port    GPIOC
#define BSP_LED1_Pin          GPIO_PIN_14    /* IO置高电平，LED灭 */

/* LED模式定义 */
#define BSP_LED0(x)    do{ x ? \
                      HAL_GPIO_WritePin(BSP_LED0_GPIO_Port, BSP_LED0_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(BSP_LED0_GPIO_Port, BSP_LED0_Pin, GPIO_PIN_RESET); \
                  }while(0)                                                          /* x = 1 , IO置高电平；x = 0 , IO置低电平 */

#define BSP_LED1(x)    do{ x ? \
                      HAL_GPIO_WritePin(BSP_LED1_GPIO_Port, BSP_LED1_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(BSP_LED1_GPIO_Port, BSP_LED1_Pin, GPIO_PIN_RESET); \
                  }while(0)                                                          /* x = 1 , IO置高电平；x = 0 , IO置低电平 */

/* LED模式取反 */
#define BSP_LED0_TOGGLE()    do{HAL_GPIO_TogglePin(BSP_LED0_GPIO_Port, BSP_LED0_Pin);}while(0)       /* LED = !LED */

#define BSP_LED1_TOGGLE()    do{HAL_GPIO_TogglePin(BSP_LED1_GPIO_Port, BSP_LED1_Pin);}while(0)       /* LED = !LED */

/* 接口函数 */
void bsp_led_init(void);    /* LED初始化 */
void bsp_led_example(void);    /* LED试验 */

#endif    /* __BSP_LED_H */
