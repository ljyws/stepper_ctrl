#include "motor.h"

motor_ctrl_t motor_ctrl;

void controller_cur_2_electric(int16_t current)
{
	motor_ctrl.foc_current = current;

	if (motor_ctrl.foc_current > 0)
		motor_ctrl.foc_location = motor_ctrl.est_position + MOVE_DIVEDE_NUM;
	else if (motor_ctrl.foc_current < 0)
		motor_ctrl.foc_location = motor_ctrl.est_position - MOVE_DIVEDE_NUM;
	else
		motor_ctrl.foc_location = motor_ctrl.est_position;

	tb67h450_set_foc_current_vector(motor_ctrl.foc_location, motor_ctrl.foc_current);
}

void controller_pid_2_electric(int32_t _speed)
{
	controller.pid.v_error_last = controller.pid.v_error;
	controller.pid.v_error = _speed - motor_ctrl.est_speed;

	if (controller.pid.v_error > (1024 * 1024))
		controller.pid.v_error = (1024 * 1024);
	if (controller.pid.v_error < (-1024 * 1024))
		controller.pid.v_error = (-1024 * 1024);

	controller.pid.output_kp = ((controller.pid.kp) * (controller.pid.v_error));
	controller.pid.i_mut += ((controller.pid.ki) * (controller.pid.v_error));
	controller.pid.i_dec = (controller.pid.i_mut >> 10);
	controller.pid.i_mut -= (controller.pid.i_dec << 10);

	controller.pid.output_ki += (controller.pid.i_dec);
	if (controller.pid.output_ki > (CURRENE_RATED_CURRENT << 10))
		controller.pid.output_ki = (CURRENE_RATED_CURRENT << 10);
	else if (controller.pid.output_ki < (-(CURRENE_RATED_CURRENT << 10)))
		controller.pid.output_ki = (-(CURRENE_RATED_CURRENT << 10));

	controller.pid.output_kd = (controller.pid.output_kd) * (controller.pid.v_error - controller.pid.v_error_last);

	controller.pid.output = (controller.pid.output_kp + controller.pid.output_ki + controller.pid.output_kd) >> 10;
	if (controller.pid.output > CURRENE_RATED_CURRENT)
		controller.pid.output = CURRENE_RATED_CURRENT;
	else if (controller.pid.output < -CURRENE_RATED_CURRENT)
		controller.pid.output = -CURRENE_RATED_CURRENT;

	motor_ctrl.foc_current = controller.pid.output;

	if (motor_ctrl.foc_current > 0)
		motor_ctrl.foc_location = motor_ctrl.est_position + MOVE_DIVEDE_NUM;
	else if (motor_ctrl.foc_current < 0)
		motor_ctrl.foc_location = motor_ctrl.est_position - MOVE_DIVEDE_NUM;
	else
		motor_ctrl.foc_location = motor_ctrl.est_position;

	tb67h450_set_foc_current_vector(motor_ctrl.foc_location, motor_ctrl.foc_current);
}

void controller_dce_2_electric(int32_t _location, int32_t _speed)
{
	controller.dce.p_error = _location - motor_ctrl.est_position;
	controller.dce.v_error = (_speed - motor_ctrl.est_speed) >> 7;
	if (controller.dce.p_error > (3200))
		controller.dce.p_error = (3200);
	if (controller.dce.p_error < (-3200))
		controller.dce.p_error = (-3200);
	if (controller.dce.v_error > (4000))
		controller.dce.v_error = (4000);
	if (controller.dce.v_error < (-4000))
		controller.dce.v_error = (-4000);

	controller.dce.output_kp = ((controller.dce.kp) * (controller.dce.p_error));

	controller.dce.i_mut += ((controller.dce.ki) * (controller.dce.p_error));
	controller.dce.i_mut += ((controller.dce.kv) * (controller.dce.v_error));
	controller.dce.i_dec = (controller.dce.i_mut >> 7);
	controller.dce.i_mut -= (controller.dce.i_dec << 7);
	controller.dce.output_ki += (controller.dce.i_dec);
	if (controller.dce.output_ki > (CURRENE_RATED_CURRENT << 10))
		controller.dce.output_ki = (CURRENE_RATED_CURRENT << 10);
	else if (controller.dce.output_ki < (-(CURRENE_RATED_CURRENT << 10)))
		controller.dce.output_ki = (-(CURRENE_RATED_CURRENT << 10));

	controller.dce.output_kd = ((controller.dce.kd) * (controller.dce.v_error));

	controller.dce.output = (controller.dce.output_kp + controller.dce.output_ki + controller.dce.output_kd) >> 10;
	if (controller.dce.output > CURRENE_RATED_CURRENT)
		controller.dce.output = CURRENE_RATED_CURRENT;
	else if (controller.dce.output < -CURRENE_RATED_CURRENT)
		controller.dce.output = -CURRENE_RATED_CURRENT;

	motor_ctrl.foc_current = controller.dce.output;

	if (motor_ctrl.foc_current > 0)
		motor_ctrl.foc_location = motor_ctrl.est_position + MOVE_DIVEDE_NUM;
	else if (motor_ctrl.foc_current < 0)
		motor_ctrl.foc_location = motor_ctrl.est_position - MOVE_DIVEDE_NUM;
	else
		motor_ctrl.foc_location = motor_ctrl.est_position;

	tb67h450_set_foc_current_vector(motor_ctrl.foc_location, motor_ctrl.foc_current);
}

void motor_ctrl_set_motor_mode(motor_ctrl_mode_e _mode)
{
	motor_ctrl.requested_mode_ = _mode;
	motor_ctrl.valid_mode = true;
}
void motor_ctrl_set_stall_switch(bool _switch)
{
	motor_ctrl.stall_switch = _switch;
	motor_ctrl.valid_stall_switch = true;
}
void motor_ctrl_set_default(void)
{
	motor_ctrl_set_motor_mode(motor_ctrl.default_confit_.ctrl_mode);
	motor_ctrl_set_stall_switch(motor_ctrl.default_confit_.is_enable_stall);
}

void motor_ctrl_write_goal_position(int32_t value)
{
	motor_ctrl.goal_position = value;
}

void motor_ctrl_write_goal_speed(int32_t value)
{
	if ((value >= -MOVE_RATED_SPEED) && (value <= MOVE_RATED_SPEED))
	{
		motor_ctrl.goal_speed = value;
	}
}

void motor_ctrl_write_goal_current(int16_t value)
{
	if ((value >= -CURRENE_RATED_CURRENT) && (value <= CURRENE_RATED_CURRENT))
	{
		motor_ctrl.goal_current = value;
	}
}

void motor_ctrl_write_goal_disable(uint16_t value)
{
	motor_ctrl.goal_disable = (bool)value;
}

void motor_ctrl_write_goal_brake(uint16_t value)
{
	motor_ctrl.goal_brake = (bool)value;
}

void motor_ctrl_init(void)
{
#ifdef LIFT_MOTOR
	motor_ctrl.default_confit_.ctrl_mode = CTRL_MODE_POSITION;
#elif defined(DOOR_MOTOR)
	motor_ctrl.default_confit_.ctrl_mode = CTRL_MODE_SPEED;
#endif

	motor_ctrl.default_confit_.is_enable_stall = true;
	if (!motor_ctrl.valid_mode)
		motor_ctrl_set_motor_mode(motor_ctrl.default_confit_.ctrl_mode);
	if (!motor_ctrl.valid_stall_switch)
		motor_ctrl_set_stall_switch(motor_ctrl.default_confit_.is_enable_stall);

	motor_ctrl.current_mode_ = CTRL_MODE_STOP;

	motor_ctrl.real_lap_position = 0;
	motor_ctrl.real_lap_position_last = 0;
	motor_ctrl.real_postion = 0;
	motor_ctrl.real_postion_last = 0;

	motor_ctrl.est_speed_mut = 0;
	motor_ctrl.est_speed = 0;
	motor_ctrl.est_lead_position = 0;
	motor_ctrl.est_position = 0;
	motor_ctrl.est_error = 0;

	motor_ctrl.goal_position = 0;
	motor_ctrl.goal_speed = 0;
	motor_ctrl.goal_current = 0;
	motor_ctrl.goal_disable = false;
	motor_ctrl.goal_brake = false;

	motor_ctrl.soft_position = 0;
	motor_ctrl.soft_speed = 0;
	motor_ctrl.soft_current = 0;
	motor_ctrl.soft_disable = false;
	motor_ctrl.soft_brake = false;
	motor_ctrl.soft_new_curve = false;

	motor_ctrl.foc_location = 0;
	motor_ctrl.foc_current = 0;

	motor_ctrl.stall_time_us = 0;
	motor_ctrl.stall_flag = false;

	motor_ctrl.overload_time_us = 0;
	motor_ctrl.overload_flag = false;

	motor_ctrl.state_ = CTRL_STATE_STOP;

	controller_init();
}

void motor_ctrl_clear_integral(void)
{
	controller_clear_integral();
}


void motor_ctrl_callback(void)
{
	// 第一次进入控制回调，主要是为了将编码器的值赋给电机控制器中
	static bool _first_entry_call = true;
	if (_first_entry_call)
	{
		motor_ctrl.real_lap_position = encoder.rectify_angle;
		motor_ctrl.real_lap_position_last = encoder.rectify_angle;
		motor_ctrl.real_postion = encoder.rectify_angle;
		motor_ctrl.real_lap_position_last = encoder.rectify_angle;
		_first_entry_call = false;
		return;
	}

	int32_t sub_data;

	motor_ctrl.real_lap_position_last = motor_ctrl.real_lap_position;
	motor_ctrl.real_lap_position = encoder.rectify_angle;

	sub_data = motor_ctrl.real_lap_position - motor_ctrl.real_lap_position_last;
	if (sub_data > (MOVE_PULSE_NUM >> 1))
		sub_data -= MOVE_PULSE_NUM;
	else if (sub_data < -(MOVE_PULSE_NUM >> 1))
		sub_data += MOVE_PULSE_NUM;

	motor_ctrl.real_postion_last = motor_ctrl.real_postion;
	motor_ctrl.real_postion += sub_data;

	motor_ctrl.est_speed_mut += (((motor_ctrl.real_postion - motor_ctrl.real_postion_last) * (CONTROL_FREQ_HZ)) + ((int32_t)(motor_ctrl.est_speed << 5) - (int32_t)(motor_ctrl.est_speed)));
	motor_ctrl.est_speed = (motor_ctrl.est_speed_mut >> 5);
	motor_ctrl.est_speed_mut = ((motor_ctrl.est_speed_mut) - ((motor_ctrl.est_speed << 5)));

	motor_ctrl.est_lead_position = motor_control_advance_compen(motor_ctrl.est_speed);
	motor_ctrl.est_position = motor_ctrl.real_postion + motor_ctrl.est_lead_position;

	motor_ctrl.est_error = motor_ctrl.soft_position - motor_ctrl.est_position;

	if ((motor_ctrl.stall_flag) || (motor_ctrl.soft_disable) || (!encoder.rectify_valid))
	{
		motor_ctrl_clear_integral();
		motor_ctrl.foc_location = 0;
		motor_ctrl.foc_current = 0;
		tb67h450_sleep();
	}
	else if (motor_ctrl.soft_brake)
	{
		motor_ctrl_clear_integral();
		motor_ctrl.foc_location = 0;
		motor_ctrl.foc_current = 0;
		tb67h450_break();
	}
	else
	{
		switch (motor_ctrl.current_mode_)
		{
		case CTRL_MODE_STOP:
		{
			tb67h450_sleep();
			break;
		}

		case CTRL_MODE_CURRENT:
		{
			controller_cur_2_electric(motor_ctrl.soft_current);
			break;
		}

		case CTRL_MODE_SPEED:
		{
			controller_pid_2_electric(motor_ctrl.soft_speed);
			break;
		}

		case CTRL_MODE_POSITION:
		{
			controller_dce_2_electric(motor_ctrl.soft_position, motor_ctrl.soft_speed);
			break;
		}

		case CTRL_MODE_TRAJ:
		{
			controller_dce_2_electric(motor_ctrl.soft_position, motor_ctrl.soft_speed);
			break;
		}

		default:
			break;
		}
	}

	if (motor_ctrl.current_mode_ != motor_ctrl.requested_mode_)
	{
		motor_ctrl.current_mode_ = motor_ctrl.requested_mode_;
		motor_ctrl.soft_new_curve = true;
	}

	if (motor_ctrl.goal_speed > MOVE_RATED_SPEED)
		motor_ctrl.goal_speed = MOVE_RATED_SPEED;
	else if (motor_ctrl.goal_speed < -MOVE_RATED_SPEED)
		motor_ctrl.goal_speed = -MOVE_RATED_SPEED;
	if (motor_ctrl.goal_current > CURRENE_RATED_CURRENT)
		motor_ctrl.goal_current = CURRENE_RATED_CURRENT;
	else if (motor_ctrl.goal_current < -CURRENE_RATED_CURRENT)
		motor_ctrl.goal_current = -CURRENE_RATED_CURRENT;

	if (((motor_ctrl.soft_disable) && (!motor_ctrl.goal_disable)) || ((motor_ctrl.soft_brake) && (!motor_ctrl.goal_brake)))
	{
		motor_ctrl.soft_new_curve = true;
	}

	if (motor_ctrl.soft_new_curve)
	{
		motor_ctrl.soft_new_curve = false;

		motor_ctrl_clear_integral();
		motor_ctrl_clear_stall();
		switch (motor_ctrl.current_mode_)
		{
		case CTRL_MODE_STOP:
		{
			break;
		}

		case CTRL_MODE_CURRENT:
		{
			current_tracker_new_task(motor_ctrl.foc_current);
			break;
		}

		case CTRL_MODE_SPEED:
		{
			speed_tracker_new_task(motor_ctrl.est_speed);
			break;
		}

		case CTRL_MODE_POSITION:
		{
			postion_tracker_new_task(motor_ctrl.est_position, motor_ctrl.est_speed);
			break;
		}

		case CTRL_MODE_TRAJ:
		{
			traj_tracker_new_task(motor_ctrl.est_position, motor_ctrl.est_speed);
			break;
		}

		default:
			break;
		}
	}

	switch (motor_ctrl.current_mode_)
	{
	case CTRL_MODE_STOP:
	{
		break;
	}

	case CTRL_MODE_CURRENT:
	{
		current_tracker_capture_goal(motor_ctrl.goal_current);
		motor_ctrl.soft_current = controller.current_tracker.go_current;
		break;
	}

	case CTRL_MODE_SPEED:
	{
		speed_tracker_capture_goal(motor_ctrl.goal_speed);
		motor_ctrl.soft_speed = controller.speed_tracker.go_speed;
		break;
	}

	case CTRL_MODE_POSITION:
	{
		postion_tracker_capture_goal(motor_ctrl.goal_position);
		motor_ctrl.soft_position = controller.position_tracker.go_location;
		motor_ctrl.soft_speed = controller.position_tracker.go_speed;
		break;
	}
	case CTRL_MODE_TRAJ:
	{
		postion_tracker_capture_goal(motor_ctrl.goal_position);
		motor_ctrl.soft_position = controller.traj_tracker.go_location;
		motor_ctrl.soft_speed = controller.traj_tracker.go_speed;
		break;
	}

	default:
		break;
	}

	motor_ctrl.soft_disable = motor_ctrl.goal_disable;
	motor_ctrl.soft_brake = motor_ctrl.goal_brake;

	int32_t abs_out_electric = abs(motor_ctrl.foc_current);

	if (((motor_ctrl.current_mode_ == CTRL_MODE_CURRENT)) && (abs_out_electric != 0) && (abs(motor_ctrl.est_speed) < (MOVE_PULSE_NUM / 5)))
	{
		if (motor_ctrl.stall_time_us >= (1000 * 1000))
			motor_ctrl.stall_flag = true;
		else
			motor_ctrl.stall_time_us += CONTROL_PERIOD_US;
	}
	else if ((abs_out_electric == CURRENE_RATED_CURRENT) && (abs(motor_ctrl.est_speed) < (MOVE_PULSE_NUM / 5)))
	{
		if (motor_ctrl.stall_time_us >= (1000 * 1000))
			motor_ctrl.stall_flag = true;
		else
			motor_ctrl.stall_time_us += CONTROL_PERIOD_US;
	}
	else
	{
		motor_ctrl.stall_time_us = 0;
	}

	if (abs_out_electric == CURRENE_RATED_CURRENT)
	{
		if (motor_ctrl.overload_time_us >= (1000 * 1000))
			motor_ctrl.overload_flag = true;
		else
			motor_ctrl.overload_time_us += CONTROL_PERIOD_US;
	}
	else
	{
		motor_ctrl.overload_time_us = 0;
		motor_ctrl.overload_flag = false;
	}

	if (motor_ctrl.current_mode_ == CTRL_MODE_STOP)
		motor_ctrl.state_ = CTRL_STATE_STOP;
	else if (motor_ctrl.stall_flag)
		motor_ctrl.state_ = CTRL_STATE_STALL;
	else if (motor_ctrl.overload_flag)
		motor_ctrl.state_ = CTRL_STATE_OVERLOAD;
	else
	{
		if (motor_ctrl.current_mode_ == CTRL_MODE_CURRENT)
		{
			if (motor_ctrl.soft_current == motor_ctrl.goal_current)
				motor_ctrl.state_ = CTRL_STATE_FINISH;
			else
				motor_ctrl.state_ = CTRL_STATE_RUNNING;
		}
		else if (motor_ctrl.current_mode_ == CTRL_MODE_SPEED)
		{
			if (motor_ctrl.soft_speed == motor_ctrl.goal_speed)
				motor_ctrl.state_ = CTRL_STATE_FINISH;
			else
				motor_ctrl.state_ = CTRL_STATE_RUNNING;
		}
		else if (motor_ctrl.current_mode_ == CTRL_MODE_POSITION)
		{
			if ((motor_ctrl.soft_position == motor_ctrl.goal_position) && (motor_ctrl.soft_speed == 0))
				motor_ctrl.state_ = CTRL_STATE_FINISH;
			else
				motor_ctrl.state_ = CTRL_STATE_RUNNING;
		}
		else
		{
			motor_ctrl.state_ = CTRL_STATE_FINISH;
		}
	}

}

int32_t motor_control_advance_compen(int32_t _speed)
{
	int32_t compen;
	if (_speed < 0)
	{
		if (_speed > -100000)
			compen = 0;
		else if (_speed > -1300000)
			compen = (((_speed + 100000) * 262) >> 20) - 0;
		else if (_speed > -2200000)
			compen = (((_speed + 1300000) * 105) >> 20) - 300;
		else
			compen = (((_speed + 2200000) * 52) >> 20) - 390;
		if (compen < -430)
			compen = -430;
	}
	else
	{
		if (_speed < 100000)
			compen = 0;
		else if (_speed < 1300000)
			compen = (((_speed - 100000) * 262) >> 20) + 0;
		else if (_speed < 2200000)
			compen = (((_speed - 1300000) * 105) >> 20) + 300;
		else
			compen = (((_speed - 2200000) * 52) >> 20) + 390;
		if (compen > 430)
			compen = 430;
	}
	return compen;
}

bool motor_get_is_calibration()
{
	return encoder.rectify_valid;
}

int32_t motor_get_pos()
{
	return motor_ctrl.real_postion;
}

int32_t motor_get_spd()
{
	return motor_ctrl.est_speed;
}

void motor_set_pos(int32_t _pos, int32_t _spd)
{
	motor_ctrl_write_goal_speed(_spd);
	motor_ctrl_write_goal_position(_pos);
}

void motor_set_spd(int32_t _spd)
{
	motor_ctrl_write_goal_speed(_spd);
}

bool motor_get_stall_flag()
{
	return motor_ctrl.stall_flag;
}

void motor_ctrl_clear_stall(void)
{
	motor_ctrl.stall_time_us = 0;
	motor_ctrl.stall_flag = false;
}


void motor_reset_encoder()
{
	motor_ctrl.real_postion = 0;
	motor_ctrl.real_postion_last = 0;
}

void motor_set_pid_parm(uint16_t _kp, uint16_t _ki, uint16_t _kd)
{
	pid_set_kp(_kp);
	pid_set_ki(_ki);
	pid_set_kd(_kd);
}

void motor_set_dce_parm(uint16_t _kp, uint16_t _ki, uint16_t _kv, uint16_t _kd)
{
	dce_set_kp(_kp);
	dce_set_ki(_ki);
	dce_set_kv(_kv);
	dce_set_kd(_kd);
}

void motor_run_break()
{
	motor_ctrl.goal_brake = true;
	motor_ctrl.goal_position = motor_ctrl.real_postion;
	motor_ctrl.goal_speed = 0;
	motor_ctrl.goal_brake = false;
}

motor_ctrl_mode_e motor_get_curr_ctrl_mode()
{
	return motor_ctrl.current_mode_;
}