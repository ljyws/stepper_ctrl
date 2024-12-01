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

/**
 * @brief This function handles System tick timer.
 * 启动初期由HAL库自动初始化的SysTick为1KHz
 * 由REIN库接管后修改的SysTick为20KHz
 **/
void SysTick_Handler(void)
{
    if (systick_20khz_flag)
    {
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
        encoder_update();

        if (encode_cali.trigger)
            encoder_calibration_interrupt_callback();
        else
            motor_ctrl_callback();
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
        lit_1ms_divider++;
        if (lit_1ms_divider >= 20)
        {
            lit_1ms_divider = 0;
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


