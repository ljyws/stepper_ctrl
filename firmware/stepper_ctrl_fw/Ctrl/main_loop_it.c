#include "main_loop_it.h"

bool systick_20khz_flag = false;
uint8_t lit_1ms_divider = 0;

loop_it_t loop_it;

/**
 * @brief  系统计时器修改为20KHz
 * @param  NULL
 * @retval NULL
 **/
void loop_it_sysTick_20khz(void)
{
    systick_20khz_flag = true;
    HAL_SYSTICK_Config(SystemCoreClock / 20000); // 更新为20K中断
}


void SysTick_Handler(void)
{
    if (systick_20khz_flag)
    {
        encoder_update();

        if (encode_cali.trigger)
            encoder_calibration_interrupt_callback();
        else
            motor_ctrl_callback();
		
        lit_1ms_divider++;
        if (lit_1ms_divider >= 20)
        {
            lit_1ms_divider = 0;
			loop_second_base_1ms();
            HAL_IncTick();
        }
    }
    else
    {
        HAL_IncTick();
    }

    loop_it.systick_count++;
}

/**
 * @brief  中断优先级覆盖
 * @param  NULL
 * @retval NULL
 **/
void loop_it_priority_overlay(void)
{
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);            
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0); 
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);    
}


