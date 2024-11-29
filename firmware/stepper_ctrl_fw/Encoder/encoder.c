#include "encoder.h"

encoder_t encoder;

uint16_t *Read_QuickCali_DATA = (uint16_t*)STOCKPILE_APP_CALI_ADDR;

bool encoder_init()
{
	encoder.raw_ = 0;
	encoder.angle = 0;
	encoder_get_angle();
	
	encoder.rectify_valid = true;
	for(uint16_t i=0; i<(CALI_ENCODER_RES); i++)
	{
		if(Read_QuickCali_DATA[i] == 0xFFFF)
			encoder.rectify_valid = false;
	}
	return true;
}

void encoder_get_angle()
{
	uint16_t data_t[2];
	uint16_t data_r[2];
	uint8_t h_count;
	
	data_t[0] = (0x80 | 0x03) << 8;
	data_t[1] = (0x80 | 0x04) << 8;
	
	enc_cs_down;
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
	enc_cs_up;
	enc_cs_down;
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
	enc_cs_up;
	encoder.raw_ = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		
	encoder.angle = encoder.raw_ >> 2;
	encoder.no_mag_flag = (bool)(encoder.raw_ & (0x0001 << 1));
}

void encoder_update()
{
	encoder_get_angle();
	encoder.rectify_angle = Read_QuickCali_DATA[encoder.angle];
}


