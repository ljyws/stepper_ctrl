#include "encoder_calibration.h"

encoder_cali_t encode_cali;

void encoder_calibration_init(void)
{
	encode_cali.trigger = false;
	encode_cali.error_code = CALI_NO_ERROR;
	encode_cali.error_data = 0;
	encode_cali.state = CALI_DISABLE;
	encode_cali.out_location = 0;
	encode_cali.rcd_x = 0;
	encode_cali.rcd_y = 0;
	encode_cali.result_num = 0;
}

static void encoder_calibration_data_check(void)
{
	uint32_t count;	
	int32_t sub_data; 

	/******************** 检查平均值连续性及方向 ********************/
	// 求解平均值数据
	for (count = 0; count < MOVE_STEP_NUM + 1; count++)
	{
		encode_cali.coder_data_f[count] = (uint16_t)cycle_average((int32_t)encode_cali.coder_data_f[count], (int32_t)encode_cali.coder_data_r[count], CALI_ENCODER_RES);
	}
	// 平均值数据检查
	sub_data = cycle_sub((int32_t)encode_cali.coder_data_f[0], (int32_t)encode_cali.coder_data_f[MOVE_STEP_NUM - 1], CALI_ENCODER_RES);
	if (sub_data == 0)
	{
		encode_cali.error_code = CALI_ERROR_AVERAGE_DIR;
		return;
	}
	if (sub_data > 0)
	{
		encode_cali.dir = true;
	}
	if (sub_data < 0)
	{
		encode_cali.dir = false;
	}

	for (count = 1; count < MOVE_STEP_NUM; count++)
	{
		sub_data = cycle_sub((int32_t)encode_cali.coder_data_f[count], (int32_t)encode_cali.coder_data_f[count - 1], CALI_ENCODER_RES);
		if (abs(sub_data) > (CALI_GATHER_ENCODER_RES * 3 / 2))
		{
			encode_cali.error_code = CALI_ERROR_AVERAGE_CONTINUITY;
			encode_cali.error_data = count;
			return;
		}
		if (abs(sub_data) < (CALI_GATHER_ENCODER_RES * 1 / 2))
		{
			encode_cali.error_code = CALI_ERROR_AVERAGE_CONTINUITY;
			encode_cali.error_data = count;
			return;
		}
		if (sub_data == 0)
		{
			encode_cali.error_code = CALI_ERROR_AVERAGE_DIR;
			encode_cali.error_data = count;
			return;
		}
		if ((sub_data > 0) && (!encode_cali.dir))
		{
			encode_cali.error_code = CALI_ERROR_AVERAGE_DIR;
			encode_cali.error_data = count;
			return;
		}
		if ((sub_data < 0) && (encode_cali.dir))
		{
			encode_cali.error_code = CALI_ERROR_AVERAGE_DIR;
			encode_cali.error_data = count;
			return;
		}
	}

	uint32_t step_num = 0;
	if (encode_cali.dir)
	{
		for (count = 0; count < MOVE_STEP_NUM; count++)
		{
			sub_data = (int32_t)encode_cali.coder_data_f[cycle_rem(count + 1, MOVE_STEP_NUM)] - (int32_t)encode_cali.coder_data_f[cycle_rem(count, MOVE_STEP_NUM)];
			if (sub_data < 0)
			{
				step_num++;
				encode_cali.rcd_x = count; // 使用区间前标
				encode_cali.rcd_y = (CALI_ENCODER_RES - 1) - encode_cali.coder_data_f[cycle_rem(encode_cali.rcd_x, MOVE_STEP_NUM)];
			}
		}
		if (step_num != 1)
		{
			encode_cali.error_code = CALI_ERROR_PHASESTEP;
			return;
		}
	}
	else
	{
		for (count = 0; count < MOVE_STEP_NUM; count++)
		{
			sub_data = (int32_t)encode_cali.coder_data_f[cycle_rem(count + 1, MOVE_STEP_NUM)] - (int32_t)encode_cali.coder_data_f[cycle_rem(count, MOVE_STEP_NUM)];
			if (sub_data > 0)
			{
				step_num++;
				encode_cali.rcd_x = count; // 使用区间前标
				encode_cali.rcd_y = (CALI_ENCODER_RES - 1) - encode_cali.coder_data_f[cycle_rem(encode_cali.rcd_x + 1, MOVE_STEP_NUM)];
			}
		}
		if (step_num != 1)
		{
			encode_cali.error_code = CALI_ERROR_PHASESTEP;
			return;
		}
	}

	// 校准OK
	encode_cali.error_code = CALI_NO_ERROR;
	return;
}

void encoder_calibration_interrupt_callback(void)
{
	uint8_t auto_calibration_speed_ = 2;
	uint8_t calibration_speed_ = 1;

	switch (encode_cali.state)
	{
	case CALI_DISABLE:
	{
		if (encode_cali.trigger)
		{
			tb67h450_set_foc_current_vector(encode_cali.out_location, CURRENT_CALI_CURRENT);
			encode_cali.out_location = MOVE_PULSE_NUM;
			encode_cali.gather_count = 0;
			encode_cali.state = CALI_FORWARD_ENCLDER_AUTOCALI;
			encode_cali.error_code = CALI_NO_ERROR;
			encode_cali.error_data = 0;
		}
		break;
	}

	case CALI_FORWARD_ENCLDER_AUTOCALI:
	{
		encode_cali.out_location += auto_calibration_speed_;
		tb67h450_set_foc_current_vector(encode_cali.out_location, CURRENT_CALI_CURRENT);
		if (encode_cali.out_location == 2 * MOVE_PULSE_NUM)
		{
			encode_cali.out_location = MOVE_PULSE_NUM;
			encode_cali.state = CALI_FORWARE_MEASURE;
		}
		break;
	}

	case CALI_FORWARE_MEASURE:
	{
		if ((encode_cali.out_location % MOVE_DIVEDE_NUM) == 0)
		{
			encode_cali.coder_data_gather[encode_cali.gather_count++] = encoder.angle;
			if (encode_cali.gather_count == ENC_CALI_GATHER_QUANTITY)
			{
				encode_cali.coder_data_f[(encode_cali.out_location - MOVE_PULSE_NUM) / MOVE_DIVEDE_NUM] = cycle_data_average(encode_cali.coder_data_gather, ENC_CALI_GATHER_QUANTITY, CALI_ENCODER_RES);

				encode_cali.gather_count = 0;

				encode_cali.out_location += calibration_speed_;
			}
		}
		else
		{
			encode_cali.out_location += calibration_speed_;
		}
		tb67h450_set_foc_current_vector(encode_cali.out_location, CURRENT_CALI_CURRENT);
		if (encode_cali.out_location > (2 * MOVE_PULSE_NUM))
		{
			encode_cali.state = CALI_REVERSE_RET;
		}
		break;
	}

	case CALI_REVERSE_RET:
	{
		encode_cali.out_location += calibration_speed_;
		tb67h450_set_foc_current_vector(encode_cali.out_location, CURRENT_CALI_CURRENT);
		if (encode_cali.out_location == (2 * MOVE_PULSE_NUM + MOVE_DIVEDE_NUM * 20))
		{
			encode_cali.state = CALI_REVERSE_GAP;
		}
		break;
	}

	case CALI_REVERSE_GAP:
	{
		encode_cali.out_location -= calibration_speed_;
		tb67h450_set_foc_current_vector(encode_cali.out_location, CURRENT_CALI_CURRENT);
		if (encode_cali.out_location == (2 * MOVE_PULSE_NUM))
		{
			encode_cali.state = CALI_REVERSE_MEASURE;
		}
		break;
	}

	case CALI_REVERSE_MEASURE:
	{
		if ((encode_cali.out_location % MOVE_DIVEDE_NUM) == 0)
		{
			encode_cali.coder_data_gather[encode_cali.gather_count++] = encoder.angle;
			if (encode_cali.gather_count == ENC_CALI_GATHER_QUANTITY)
			{
				encode_cali.coder_data_r[(encode_cali.out_location - MOVE_PULSE_NUM) / MOVE_DIVEDE_NUM] = cycle_data_average(encode_cali.coder_data_gather, ENC_CALI_GATHER_QUANTITY, CALI_ENCODER_RES);
				encode_cali.gather_count = 0;
				encode_cali.out_location -= calibration_speed_;
			}
		}
		else
		{
			encode_cali.out_location -= calibration_speed_;
		}
		tb67h450_set_foc_current_vector(encode_cali.out_location, CURRENT_CALI_CURRENT);
		if (encode_cali.out_location < MOVE_PULSE_NUM)
		{
			encode_cali.state = CALI_OPERATION;
		}
		break;
	}

	case CALI_OPERATION:
	{
		tb67h450_set_foc_current_vector(0, 0);
		break;
	}

	default:
		break;
	}
}

void encoder_calibration_loop_callback(void)
{
	int32_t data_i32;
	uint16_t data_u16;

	if (encode_cali.state != CALI_OPERATION)
		return;

	tb67h450_sleep();
	encoder_calibration_data_check();

	if (encode_cali.error_code == CALI_NO_ERROR)
	{
		int32_t step_x;
		int32_t step_y;
		encode_cali.result_num = 0;
		flash_erase(&stockpile_quick_cali);
		flash_write_begin(&stockpile_quick_cali);
		if (encode_cali.dir)
		{
			for (step_x = encode_cali.rcd_x; step_x < encode_cali.rcd_x + MOVE_STEP_NUM + 1; step_x++)
			{
				data_i32 = cycle_sub(encode_cali.coder_data_f[cycle_rem(step_x + 1, MOVE_STEP_NUM)], encode_cali.coder_data_f[cycle_rem(step_x, MOVE_STEP_NUM)], CALI_ENCODER_RES);
				if (step_x == encode_cali.rcd_x)
				{
					for (step_y = encode_cali.rcd_y; step_y < data_i32; step_y++)
					{
						data_u16 = cycle_rem(MOVE_DIVEDE_NUM * step_x + MOVE_DIVEDE_NUM * step_y / data_i32, MOVE_PULSE_NUM);
						flash_write_halfword(&stockpile_quick_cali, &data_u16, 1);
						encode_cali.result_num++;
					}
				}
				else if (step_x == encode_cali.rcd_x + MOVE_STEP_NUM)
				{
					for (step_y = 0; step_y < encode_cali.rcd_y; step_y++)
					{
						data_u16 = cycle_rem(MOVE_DIVEDE_NUM * step_x + MOVE_DIVEDE_NUM * step_y / data_i32, MOVE_PULSE_NUM);
						flash_write_halfword(&stockpile_quick_cali, &data_u16, 1);
						encode_cali.result_num++;
					}
				}
				else
				{
					for (step_y = 0; step_y < data_i32; step_y++)
					{
						data_u16 = cycle_rem(MOVE_DIVEDE_NUM * step_x + MOVE_DIVEDE_NUM * step_y / data_i32, MOVE_PULSE_NUM);
						flash_write_halfword(&stockpile_quick_cali, &data_u16, 1);
						encode_cali.result_num++;
					}
				}
			}
		}
		else
		{
			for (step_x = encode_cali.rcd_x + MOVE_STEP_NUM; step_x > encode_cali.rcd_x - 1; step_x--)
			{
				data_i32 = cycle_sub(encode_cali.coder_data_f[cycle_rem(step_x, MOVE_STEP_NUM)], encode_cali.coder_data_f[cycle_rem(step_x + 1, MOVE_STEP_NUM)], CALI_ENCODER_RES);
				if (step_x == encode_cali.rcd_x + MOVE_STEP_NUM)
				{
					for (step_y = encode_cali.rcd_y; step_y < data_i32; step_y++)
					{
						data_u16 = cycle_rem(MOVE_DIVEDE_NUM * (step_x + 1) - MOVE_DIVEDE_NUM * step_y / data_i32, MOVE_PULSE_NUM);
						flash_write_halfword(&stockpile_quick_cali, &data_u16, 1);
						encode_cali.result_num++;
					}
				}
				else if (step_x == encode_cali.rcd_x)
				{
					for (step_y = 0; step_y < encode_cali.rcd_y; step_y++)
					{
						data_u16 = cycle_rem(MOVE_DIVEDE_NUM * (step_x + 1) - MOVE_DIVEDE_NUM * step_y / data_i32, MOVE_PULSE_NUM);
						flash_write_halfword(&stockpile_quick_cali, &data_u16, 1);
						encode_cali.result_num++;
					}
				}
				else
				{
					for (step_y = 0; step_y < data_i32; step_y++)
					{
						data_u16 = cycle_rem(MOVE_DIVEDE_NUM * (step_x + 1) - MOVE_DIVEDE_NUM * step_y / data_i32, MOVE_PULSE_NUM);
						flash_write_halfword(&stockpile_quick_cali, &data_u16, 1);
						encode_cali.result_num++;
					}
				}
			}
		}
		flash_write_end(&stockpile_quick_cali);

		if (encode_cali.result_num != CALI_ENCODER_RES)
			encode_cali.error_code = CALI_ERROR_ANALYSIS_QUANTITY;
	}
	if (encode_cali.error_code == CALI_NO_ERROR)
	{
		encoder.rectify_valid = true;
	}
	else
	{
		encoder.rectify_valid = false;
		flash_erase(&stockpile_quick_cali);
	}

	motor_ctrl.stall_flag = true;

	encode_cali.state = CALI_DISABLE;
	encode_cali.trigger = false;

	HAL_Delay(1000);
	mcu_software_reset();
}
