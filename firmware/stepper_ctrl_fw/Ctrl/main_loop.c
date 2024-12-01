#include "main_loop.h"

void main_loop()
{
	HAL_Delay(1000);

    tb67h450_init();
    encoder_init();
    motor_ctrl_init();
    encoder_calibration_init();
	uart_receive_init(&huart3);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    loop_it_priority_overlay();
    loop_it_sysTick_20khz();

    //HAL_TIM_Base_Start(&htim1);

    for(;;)
    {
        encoder_calibration_loop_callback();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
        encoder_update();
			
        if (encode_cali.trigger)
            encoder_calibration_interrupt_callback();
        else
            motor_ctrl_callback();
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    }
}
