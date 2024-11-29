#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "board_cfg.h"
#include "glazer_api.h"

#define UART_RX_DMA_SIZE (1024)

#define USART3_BUFLEN 14
#define USART3_MAX_LEN USART3_BUFLEN * 2

extern uint8_t usart3_buf[USART3_BUFLEN];


void uart_receive_handler(UART_HandleTypeDef *huart);
void uart_receive_init(UART_HandleTypeDef *huart);











#endif
