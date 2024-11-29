#include "ws2812.h"

/******** Glabal Variables ********/
uint16_t RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = {0};
uint16_t *ptr;

/**
 * @brief   设定单个ws2812灯珠颜色
 * @param[in] R：R值
 * @param[in]	G：G值
 * @param[in] B：B值
 * @param[in] num：第 num 个灯珠
 * @retval  NULL
 */
void ws2812_Set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    // 指针偏移:需要跳过复位信号的N个0
    uint16_t *p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);

    for (uint16_t i = 0; i < 8; i++)
    {
        // 填充数组
        p[i] = (G << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        ptr[i] = p[i];
        p[i + 8] = (R << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        ptr[i + 8] = p[i + 8];
        p[i + 16] = (B << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        ptr[i + 16] = p[i + 16];
    }
}

/**
 * @brief   设定所有ws2812灯珠颜色
 * @param[in] R：R值
 * @param[in]	G：G值
 * @param[in] B：B值
 * @param[in] led_nums：共有 led_nums 个灯珠
 * @retval  NULL
 */
void ws2812_Set_All_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t led_nums)
{
    uint16_t num_data;
    num_data = 80 + led_nums * 24;
    for (uint8_t i = 0; i < led_nums; i++)
    {
        ws2812_Set_RGB(R, G, B, i);
    }
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)RGB_buffur, (num_data));
}