#ifndef __IN_SERIAL_H
#define __IN_SERIAL_H

#include "usart.h"
#include <stdio.h>


/* 接口定义 */
#define SERIAL_USARTx            USART1            /* 使用第几号串口 */
#define SERIAL_huartx            huart1            /* 所使用串口的句柄 */
#define SERIAL_hdma_usartx_rx    hdma_usart1_rx    /* DMA串口接收通道的句柄 */
#define SERIAL_USART_ISR         ISR               /* USART的 状态寄存器SR */
#define SERIAL_USART_TDR         TDR               /* USART的 发送数据寄存器TDR */
#define RS485_EN_GPIO_Port       GPIOA            /* RS485发送使能引脚 */
#define RS485_EN_Pin             GPIO_PIN_5       

#define RS485DIR_TX HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);    /* 置1为串口发送 */
#define RS485DIR_RX HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);  /* 置0为串口接收 */

#define SERIAL_PRINTF_EN         1                 /*使能(1)/禁止(0) 串口printf*/
#define SERIAL_RX_EN             1                 /*使能(1)/禁止(0) 串口接收*/

#define USART_RX_LEN             256               /* 定义最大接收字节数 */

extern DMA_HandleTypeDef SERIAL_hdma_usartx_rx;

extern uint8_t g_usart_rx_buf[USART_RX_LEN];     /* 接收缓冲区, 最大USART_REC_LEN个字节，用于存储完整的数据帧 */

/* 接口函数 */
void in_serial_init(void);    /* 串口初始化 */

#endif    /* __IN_SERIAL_H */
