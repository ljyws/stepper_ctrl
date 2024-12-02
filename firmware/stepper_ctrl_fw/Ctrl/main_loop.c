#include "main_loop.h"

uint32_t time_1ms_count = 0;
uint32_t major_cycle_count = 0;
static uint32_t time_second_1ms = 0;
static uint32_t time_second_10ms = 0;
static uint32_t time_second_20ms = 0;
static uint32_t time_second_50ms = 0;
static uint32_t time_second_100ms = 0;
static uint32_t time_second_500ms = 0;

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


    for(;;)
    {
		major_cycle_count++;
		time_second_run();
        encoder_calibration_loop_callback();
    }
}

/**
 * @brief 副时钟10ms执行
 */
void time_second_10ms_serve(void)
{

}

/**
 * @brief 副时钟20ms执行
 */
void time_second_20ms_serve(void)
{

}

/**
 * @brief 副时钟50ms执行
 */
void time_second_50ms_serve(void)
{

}

/**
* @brief 副时钟100ms执行
*/
void time_second_100ms_serve(void)
{

	
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}

/**
 * @brief 副时钟500ms执行
 */
void time_second_500ms_serve(void)
{
//	for(int i = 20*10; i<255*10; i++)
//	{
//		ws2812_Set_All_RGB(i/10,0,i/10,1);
//	}
//	for(int j = 255*10; j>20*10; j--)
//	{
//		ws2812_Set_All_RGB(j/10,0,j/10,1);
//	}
}


void loop_second_base_1ms(void)
{
	time_1ms_count++;
	time_second_1ms++;
	if(!(time_second_1ms % 10))		
		time_second_10ms++;	
	if(!(time_second_1ms % 20))		
		time_second_20ms++;	
	if(!(time_second_1ms % 50))		
		time_second_50ms++;		
	if(!(time_second_1ms % 100))	
		time_second_100ms++;		
	if(!(time_second_1ms % 500))	
		time_second_500ms++;
	if(!(time_second_1ms % 1000))	
		time_second_1ms = 0;		
}

/**
 * @brief 副时钟循环执行
 */
void time_second_run(void)
{
	if(time_second_10ms)		
	{
		time_second_10ms--;	
		time_second_10ms_serve();		
	}
	if(time_second_20ms)	
	{
		time_second_20ms--;		
		time_second_20ms_serve();		
	}
	if(time_second_50ms)
	{
		time_second_50ms--;
		time_second_50ms_serve();	
	}
	if(time_second_100ms)		
	{	
		time_second_100ms--;
		time_second_100ms_serve();
	}
	if(time_second_500ms)		
	{	
		time_second_500ms--;
		time_second_500ms_serve();
	}
}


