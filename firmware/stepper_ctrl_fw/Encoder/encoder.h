#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "board_cfg.h"

#define enc_cs_down HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
#define enc_cs_up HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);


typedef struct
{
	uint16_t raw_;							//SPI读取到的数据
	uint16_t angle;						    //SPI输出的角度
	bool no_mag_flag;			            //磁铁数据有效标志
	bool pc_flag;					        //奇偶校验位	
	uint16_t rectify_angle;		            //校准的角度数据
	bool rectify_valid;		                //校准数据有效标志
}encoder_t;
extern encoder_t encoder;

bool encoder_init(void);

void encoder_update();

void encoder_get_angle();

void mt6816_test();
#endif
