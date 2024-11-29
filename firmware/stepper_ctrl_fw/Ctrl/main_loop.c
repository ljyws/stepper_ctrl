#include "main_loop.h"

void main_loop()
{
	uart_receive_init(&huart3);
    tb67h450_init();
    encoder_init();
    motor_ctrl_init();
    encoder_calibration_init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);
}

void breath_led(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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

    if (htim->Instance == TIM4)
    {
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
        encoder_calibration_loop_callback();
    }
}
