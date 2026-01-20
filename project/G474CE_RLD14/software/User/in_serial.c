#include "in_serial.h"

/* Use "printf" function without definition "use MicroLIB" ------------------------------------------------------------------*/
#if SERIAL_PRINTF_EN    /*使能串口printf*/
/* 使用AC6编译器
 */
#if (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting\n\t");    /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* 声明main函数为无参数格式，否则部分例程可能出现半主机模式 */
#else
/* 使用AC5编译器时
 */
#pragma import(__use_no_semihosting)    /* 声明不使用半主机模式 */

/* 定义__FILE
 */
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

/* 不使用半主机模式，至少需重定义 __ttywrch函数、_sys_exit函数、_sys_command_string函数，以同时兼容AC5和AC6模式 */

#endif

/*重定义 __ttywrch函数
 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 重定义 _sys_exit函数
 * 避免使用半主机模式
 */
void _sys_exit(int x)
{
    x = x;
}

/*重定义 _sys_command_string函数
 */
char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* __FILE在stdio.h中定义
 */
FILE __stdout;

/* printf函数重定向
 * 重定义fputc函数
 * 供printf函数最终调用，输出字符串 */
int fputc(int ch, FILE *f)
{
    /* 等待上一个字符串发送完成
     * 对 USART的 状态寄存器SR 的 位6 TC发送完成(Transmission complete) 位与操作 0X40(即01000000)
     * 当TC为0，表示发送未完成，(SR & 0X40)为0，while循环条件为真，因为无操作，所以等待
     * 当TC为1，表示发送已完成，(SR & 0X40)为1，while循环条件为假，跳过while循环 */
    while ((SERIAL_USARTx->SERIAL_USART_ISR & 0X40) == 0);
    
    /* 将要发送的字符 ch 写入到 USART的 发送数据寄存器TDR */
    SERIAL_USARTx->SERIAL_USART_TDR = (uint8_t) ch;
    
    RS485DIR_TX;    /* 串口发送 */
    
    return ch;
}
#endif
/* Use "printf" function without definition "use MicroLIB" ------------------------------------------------------------------*/

/* USART_RX ------------------------------------------------------------------*/
#if SERIAL_RX_EN    /*使能串口接收*/

/* 接收缓冲区, 最大USART_REC_LEN个字节，用于存储完整的数据帧 */
uint8_t g_usart_rx_buf[USART_RX_LEN];

/* 串口初始化 */
void in_serial_init(void)
{
    //MX_USART1_UART_Init();
    
    RS485DIR_RX;    /* 串口接收 */
    
    /* 开启DMA接收串口中断 */
    HAL_UART_Receive_DMA(&SERIAL_huartx,g_usart_rx_buf,2);
    
//    /* 开启DMA接收串口空闲中断 */
//    HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_huartx,(uint8_t *)g_usart_rx_buf,sizeof(g_usart_rx_buf));
//    /* 关闭DMA传输过半中断(否则也将触发RxEventCallback回调函数) */
//    __HAL_DMA_DISABLE_IT(&SERIAL_hdma_usartx_rx,DMA_IT_HT);    /* (DMA通道的指针地址 , 要关闭的中断) */
}

/* UART接收完成回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 判断使用此回调函数的串口 */
    if(huart->Instance==SERIAL_USARTx)
    {
        /* 发送接收到的数据 */
        RS485DIR_TX;    /* 串口发送 */
        HAL_UART_Transmit_DMA(&SERIAL_huartx,g_usart_rx_buf,2);
        
        /* 数据的处理逻辑 */
        
        
        /* 再次开启接收中断 */
        RS485DIR_RX;    /* 串口接收 */
        HAL_UART_Receive_DMA(&SERIAL_huartx,g_usart_rx_buf,2);
    }
}

/* UART接收事件回调函数 */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    /* 判断使用此回调函数的串口 */
//    if(huart->Instance==SERIAL_USARTx)
//    {
//        /* 发送接收到的数据 */
////        printf("→mcu：串口接送到数据    ");
//        RS485DIR_TX;    /* 串口发送 */
//        HAL_UART_Transmit_DMA(&SERIAL_huartx,(uint8_t *)g_usart_rx_buf,Size);
//        
//        /* 数据的处理逻辑 */
//        
//        
//        
//        RS485DIR_RX;    /* 串口接收 */
//        
//        /* 再次开启DMA接收串口空闲中断 */
//        HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_huartx,(uint8_t *)g_usart_rx_buf,sizeof(g_usart_rx_buf));
//        /* 关闭DMA传输过半中断(否则也将触发RxEventCallback回调函数) */
//        __HAL_DMA_DISABLE_IT(&SERIAL_hdma_usartx_rx,DMA_IT_HT);    /* (DMA通道的指针地址 , 要关闭的中断) */
//    }
//}
#endif
/* USART_RX ------------------------------------------------------------------*/
