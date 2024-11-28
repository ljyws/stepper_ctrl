#include "motor.h"

motor_ctrl_t motor_ctrl;

void motor_ctrl_init(void)
{
    motor_ctrl.default_confit_.ctrl_mode = CTRL_MODE_SPEED;
    motor_ctrl.default_confit_.is_enable_stall = true;
	if(!motor_ctrl.valid_mode)					
		motor_ctrl_set_motor_mode(motor_ctrl.default_confit_.ctrl_mode);
	if(!motor_ctrl.valid_stall_switch)
		motor_ctrl_set_stall_switch(motor_ctrl.default_confit_.is_enable_stall);

	motor_ctrl.current_mode_ = CTRL_MODE_STOP;

	motor_ctrl.real_lap_location = 0;
	motor_ctrl.real_lap_location_last = 0;
	motor_ctrl.real_location = 0;
	motor_ctrl.real_location_last = 0;
	//估计
	motor_ctrl.est_speed_mut = 0;
	motor_ctrl.est_speed = 0;
	motor_ctrl.est_lead_location = 0;
	motor_ctrl.est_location = 0;
	motor_ctrl.est_error = 0;
	//硬目标
	motor_ctrl.goal_location = 0;
	motor_ctrl.goal_speed = 0;
	motor_ctrl.goal_current = 0;
	motor_ctrl.goal_disable = false;
	motor_ctrl.goal_brake = false;
	//软目标
	motor_ctrl.soft_location = 0;
	motor_ctrl.soft_speed = 0;
	motor_ctrl.soft_current = 0;
	motor_ctrl.soft_disable = false;
	motor_ctrl.soft_brake = false;
	motor_ctrl.soft_new_curve = false;
	//输出
	motor_ctrl.foc_location = 0;
	motor_ctrl.foc_current = 0;
	//堵转识别
	motor_ctrl.stall_time_us = 0;
	motor_ctrl.stall_flag = false;
	//过载识别
	motor_ctrl.overload_time_us = 0;
	motor_ctrl.overload_flag = false;
	//状态
	motor_ctrl.state_ = CTRL_STATE_STOP;		
}