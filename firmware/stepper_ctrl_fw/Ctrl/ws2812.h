#ifndef __BSP_WS2812_H__
#define __BSP_WS2812_H__

#include "board_cfg.h"

#define ONE_PULSE        (68)                           //1 码计数个数
#define ZERO_PULSE       (28)                           //0 码计数个数
#define RESET_PULSE      (80)                           //80 复位电平个数（不能低于40）
#define WS_LED_NUMS      (1)                          	//led 个数
#define LED_DATA_LEN     (24)                           //led 长度，单个需要24个字节
#define WS2812_DATA_LEN  (WS_LED_NUMS*LED_DATA_LEN)     //ws2812灯条需要的数组长度

void ws2812_Init(uint8_t led_nums);
void ws2812_Set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num);
void ws2812_Set_All_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t led_nums);

#endif