#include "glazer_api.h"

int32_t spd_;
int32_t pos_;
void glazer_uart_rx_data_unpack(uint8_t *buff)
{
#ifdef LIFT_MOTOR
    if(buff[0] == 0xAA && buff[1] == 0xAF && buff[2] == 0x01)
    {
        switch ((int)buff[3])
        {
        case 1:
        {
			spd_ = buff[4]<<24 | buff[5] << 16 | buff[6] << 8 | buff[7];
			pos_ = buff[8]<<24 | buff[9] << 16 | buff[10] << 8 | buff[11];
            glazer_set_motor_pos(pos_,spd_);
            break;
        }

        case 2:
        {
            glazer_reset_encoder_pos();
            break;
        }

        case 3:
        {
            glazer_reset_stall_flag();
        }

        case 4:
        {
            // glazer_set_pid_parm()
            break;
        }

        case 5:
        {
            // glazer_set_dce_parm()
            break;
        }
        
        case 6:
        {
            glazer_entry_encoder_calibration();
            break;
        }

        case 7:
        {
            glazer_set_break();
            break;
        }

        default:
            break;
        }
    }

#elif defined (DOOR_MOTOR)
    if(buff[0] == 0xAA && buff[1] == 0xAF && buff[2] == 0x01)
    {
        switch ((int)buff[3])
        {
        case 1:
        {
			spd_ = buff[4]<<24 | buff[5] << 16 | buff[6] << 8 | buff[7];
			pos_ = buff[8]<<24 | buff[9] << 16 | buff[10] << 8 | buff[11];
            glazer_set_motor_spd(spd_);
            break;
        }

        case 2:
        {
            glazer_reset_encoder_pos();
            break;
        }

        case 3:
        {
            glazer_reset_stall_flag();
        }

        case 4:
        {
            // glazer_set_pid_parm()
            break;
        }

        case 5:
        {
            // glazer_set_dce_parm()
            break;
        }
        
        case 6:
        {
            glazer_entry_encoder_calibration();
            break;
        }

        case 7:
        {
            glazer_set_break();
            break;
        }

        default:
            break;
        }
    }
#endif 
}

void glazer_entry_encoder_calibration(void)
{
    encode_cali.trigger = true;
}

int32_t glazer_get_encoder_pos()
{
    return motor_get_pos();
}

int32_t glazer_get_encoder_spd()
{
    return motor_get_spd();
}

void glazer_set_motor_pos(int32_t _pos, int32_t _spd)
{
    motor_set_pos(_pos, _spd);
}

void glazer_set_motor_spd(int32_t _spd)
{
    motor_set_spd(_spd);
}

void glazer_reset_encoder_pos(void)
{
    motor_reset_encoder();
}

void glazer_reset_stall_flag(void)
{
    motor_ctrl_clear_stall();
}

bool glazer_get_stail_flag(void)
{
    return motor_get_stall_flag();
}

bool glazer_get_is_calibration()
{
    return motor_get_is_calibration();
}

void glazer_set_pid_parm(uint16_t _kp, uint16_t _ki, uint16_t _kd)
{
    motor_set_pid_parm(_kp, _ki, _kd);
}

void glazer_set_dce_parm(uint16_t _kp, uint16_t _ki, uint16_t _kv, uint16_t _kd)
{
    motor_set_dce_parm(_kp, _ki, _kv, _kd);
}

int glazer_get_curr_ctrl_mode()
{
    return motor_get_curr_ctrl_mode();
}

void glazer_set_break()
{
    motor_run_break();
}

