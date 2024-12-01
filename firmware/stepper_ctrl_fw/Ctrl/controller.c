#include "controller.h"

controller_t controller;


void controller_init(void)
{
	pid_init();
	dce_init();
	current_tracker_init();
	speed_tracker_init();
	postion_tracker_init();
	position_interp_init();
	traj_tracker_init();
}

void controller_clear_integral(void)
{
	controller.pid.i_mut = 0;
	controller.pid.i_dec = 0;
	controller.pid.output_ki = 0;
	
	controller.dce.i_mut = 0;
	controller.dce.i_dec = 0;
	controller.dce.output_ki = 0;
}

/************************************************************************************************************/

void pid_set_kp(uint16_t _k)
{
	if(_k <= 1024)
	{		
		controller.pid.kp = _k;		
		controller.pid.config_.valid_kp = true;		
	}else
	{
		controller.pid.config_.valid_kp = false;	
	}
}	


void pid_set_ki(uint16_t _k)
{
	if(_k <= 1024)
	{		
		controller.pid.ki = _k;		
		controller.pid.config_.valid_ki = true;		
	}else
	{
		controller.pid.config_.valid_ki = false;	
	}
}


void pid_set_kd(uint16_t _k)
{
	if(_k <= 1024)
	{		
		controller.pid.kd = _k;		
		controller.pid.config_.valid_kd = true;		
	}else
	{
		controller.pid.config_.valid_kd = false;	
	}
}


void pid_set_default(void)
{
	pid_set_kp(controller.pid.config_.default_kp);
	pid_set_ki(controller.pid.config_.default_ki);
	pid_set_kd(controller.pid.config_.default_kd);
}



void pid_init(void)
{
	controller.pid.config_.default_kp = 5.0f;
	controller.pid.config_.default_ki = 30.0f;
	controller.pid.config_.default_kd = 0;

	if(!controller.pid.config_.valid_kp)	
		pid_set_kp(controller.pid.config_.default_kp);	
	if(!controller.pid.config_.valid_ki)		
		pid_set_ki(controller.pid.config_.default_ki);		
	if(!controller.pid.config_.valid_kd)				
		pid_set_kd(controller.pid.config_.default_kd);
	
	controller.pid.v_error = 0;	
	controller.pid.v_error_last = 0;
	controller.pid.output_kp = 0;		
	controller.pid.output_ki = 0;		
	controller.pid.output_kd = 0;	
	controller.pid.i_mut = 0;
	controller.pid.i_dec = 0;
	controller.pid.output = 0;
}

void dce_set_kp(uint16_t _k)
{
	if(_k <= 1024)
	{	
		controller.dce.kp = _k;		
		controller.dce.config_.valid_kp = true;		
	}else
	{
		controller.dce.config_.valid_kp = false;
	}
}

void dce_set_ki(uint16_t _k)
{
    if(_k <= 1024)
	{	
		controller.dce.ki = _k;		
		controller.dce.config_.valid_ki = true;		
	}else
	{
		controller.dce.config_.valid_ki = false;
	}
}

void dce_set_kv(uint16_t _k)
{
    if(_k <= 1024)
	{	
		controller.dce.kv = _k;		
		controller.dce.config_.valid_kv = true;		
	}else
	{
		controller.dce.config_.valid_kv = false;
	}
}

void dce_set_kd(uint16_t _k)
{
    if(_k <= 1024)
	{	
		controller.dce.kd = _k;		
		controller.dce.config_.valid_kd = true;		
	}else
	{
		controller.dce.config_.valid_kd = false;
	}
}

void dce_set_default(void)
{
    dce_set_kp(controller.dce.config_.default_kp);
    dce_set_ki(controller.dce.config_.default_ki);
    dce_set_kv(controller.dce.config_.default_kv);
    dce_set_kd(controller.dce.config_.default_kd);
}

void dce_init(void)
{
    controller.dce.config_.default_kp = 500.0f;
    controller.dce.config_.default_ki = 80.0f;
    controller.dce.config_.default_kv = 300.0f;
    controller.dce.config_.default_kd = 250.0f;

	if(!controller.dce.config_.valid_kp)				
		dce_set_kp(controller.dce.config_.default_kp);
	if(!controller.dce.config_.valid_ki)				
		dce_set_ki(controller.dce.config_.default_ki);
	if(!controller.dce.config_.valid_kv)				
		dce_set_kv(controller.dce.config_.default_kv);
	if(!controller.dce.config_.valid_kd)				
		dce_set_kd(controller.dce.config_.default_kd);
	
	controller.dce.p_error = 0;
	controller.dce.v_error = 0;
	controller.dce.output_kp = 0;				
	controller.dce.output_ki = 0;	
	controller.dce.output_kd = 0;	
	controller.dce.i_mut = 0;	
	controller.dce.i_dec = 0;
	controller.dce.output = 0;   
}


/************************************************************************************************************/


void current_tracker_init(void)
{
    controller.current_tracker.config_.default_up_rate = MOVE_RATED_UP_CURRENT_RATE / 10;
    controller.current_tracker.config_.default_down_rate = MOVE_RATED_DOWN_CURRENT_RATE / 10;
    if(!controller.current_tracker.config_.valid_up_rate)			
	{
		current_tracker_set_up_rate(controller.current_tracker.config_.default_up_rate);	
	}
	if(!controller.current_tracker.config_.valid_down_rate)	
	{	
		current_tracker_set_down_rate(controller.current_tracker.config_.default_down_rate);	
	}

    controller.current_tracker.course_mut = 0;
    controller.current_tracker.course = 0;
    controller.current_tracker.go_current = 0;
}										

void current_tracker_new_task(int16_t real_current)
{
    controller.current_tracker.course_mut = 0;
    controller.current_tracker.course = real_current;
}			

void current_tracker_capture_goal(int32_t goal_current)
{
    int32_t electric_sub = goal_current - controller.current_tracker.course;

    if(electric_sub == 0)
	{
		controller.current_tracker.course = goal_current;
	}else if(electric_sub > 0)
	{
		if(controller.current_tracker.course >= 0)
		{
            controller.current_tracker.course_mut += controller.current_tracker.config_.up_rate;		
	        controller.current_tracker.course += controller.current_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.current_tracker.course_mut =  controller.current_tracker.course_mut % CONTROL_FREQ_HZ;

			if(controller.current_tracker.course >= goal_current)
			{
				controller.current_tracker.course_mut = 0;
				controller.current_tracker.course = goal_current;
			}
		}else
		{
            controller.current_tracker.course_mut += controller.current_tracker.config_.down_rate;	
	        controller.current_tracker.course += controller.current_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.current_tracker.course_mut =  controller.current_tracker.course_mut % CONTROL_FREQ_HZ;
			if((int32_t)controller.current_tracker.course >= 0)
			{
				controller.current_tracker.course_mut = 0;
				controller.current_tracker.course = 0;
			}
		}
    }else if(electric_sub < 0)
	{
		if(controller.current_tracker.course <= 0)
		{
            controller.current_tracker.course_mut += (-controller.current_tracker.config_.up_rate);
	        controller.current_tracker.course += controller.current_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.current_tracker.course_mut =  controller.current_tracker.course_mut % CONTROL_FREQ_HZ;
			if((int32_t)controller.current_tracker.course <= (int32_t)goal_current)
			{
				controller.current_tracker.course_mut = 0;
				controller.current_tracker.course = goal_current;
			}
		}
		else
		{
            controller.current_tracker.course_mut += (-controller.current_tracker.config_.down_rate);
	        controller.current_tracker.course += controller.current_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.current_tracker.course_mut =  controller.current_tracker.course_mut % CONTROL_FREQ_HZ;
			if((int32_t)controller.current_tracker.course <= 0)
			{
				controller.current_tracker.course_mut = 0;
				controller.current_tracker.course = 0;
			}
		}
	}

    controller.current_tracker.go_current = (int32_t)controller.current_tracker.course;
}

void current_tracker_set_up_rate(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_UP_CURRENT_RATE))
	{
		controller.current_tracker.config_.up_rate = value;
		controller.current_tracker.config_.valid_up_rate = true;
	}
	else
	{
		controller.current_tracker.config_.valid_up_rate = false;
	}
}
void current_tracker_set_down_rate(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_DOWN_CURRENT_RATE))
	{
		controller.current_tracker.config_.down_rate = value;
		controller.current_tracker.config_.valid_down_rate = true;
	}
	else
	{
		controller.current_tracker.config_.valid_down_rate = false;
	}
}

void current_tracker_set_default(void)
{
	current_tracker_set_up_rate(controller.current_tracker.config_.default_up_rate);
	current_tracker_set_down_rate(controller.current_tracker.config_.default_down_rate);
}

/************************************************************************************************************/

void speed_tracker_init(void)
{
    controller.speed_tracker.config_.default_up_acc = MOVE_RATED_UP_ACC / 10;
    controller.speed_tracker.config_.default_down_acc = MOVE_RATED_DOWN_ACC / 10;
	if(!controller.speed_tracker.config_.valid_up_acc)			
		speed_tracker_set_up_acc(controller.speed_tracker.config_.default_up_acc);		
	if(!controller.speed_tracker.config_.valid_down_acc)			
		speed_tracker_set_down_acc(controller.speed_tracker.config_.default_down_acc);
	
	controller.speed_tracker.course_mut = 0;
	controller.speed_tracker.course = 0;
	controller.speed_tracker.go_speed = 0;
}

void speed_tracker_new_task(int32_t real_speed)
{
    controller.speed_tracker.course_mut = 0;		
	controller.speed_tracker.course = real_speed;
}	

void speed_tracker_capture_goal(int32_t goal_speed)
{
	int32_t speed_sub = goal_speed - controller.speed_tracker.course;

	if(speed_sub == 0)
	{
		controller.speed_tracker.course = goal_speed;		
	}else if(speed_sub > 0)
	{
		if(controller.speed_tracker.course >= 0)
		{
            controller.speed_tracker.course_mut += (controller.speed_tracker.config_.up_acc);											
	        controller.speed_tracker.course += controller.speed_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.speed_tracker.course_mut = controller.speed_tracker.course_mut % CONTROL_FREQ_HZ;	
			if(controller.speed_tracker.course >= goal_speed)
			{
				controller.speed_tracker.course_mut = 0;
				controller.speed_tracker.course = goal_speed;
			}
		}else
		{
            controller.speed_tracker.course_mut += (controller.speed_tracker.config_.down_acc);											
	        controller.speed_tracker.course += controller.speed_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.speed_tracker.course_mut = controller.speed_tracker.course_mut % CONTROL_FREQ_HZ;
			if(controller.speed_tracker.course >= 0)
			{
				controller.speed_tracker.course_mut = 0;
				controller.speed_tracker.course = 0;
			}
		}
	}else if(speed_sub < 0)
	{
		if(controller.speed_tracker.course <= 0)
		{
            controller.speed_tracker.course_mut += (-controller.speed_tracker.config_.up_acc);									
	        controller.speed_tracker.course += controller.speed_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.speed_tracker.course_mut = controller.speed_tracker.course_mut % CONTROL_FREQ_HZ;
			if(controller.speed_tracker.course <= goal_speed)
			{
				controller.speed_tracker.course_mut = 0;
				controller.speed_tracker.course = goal_speed;
			}
		}else
		{
            controller.speed_tracker.course_mut += (-controller.speed_tracker.config_.down_acc);								
	        controller.speed_tracker.course += controller.speed_tracker.course_mut / CONTROL_FREQ_HZ;		
	        controller.speed_tracker.course_mut = controller.speed_tracker.course_mut % CONTROL_FREQ_HZ;
			if(controller.speed_tracker.course <= 0)
			{
				controller.speed_tracker.course_mut = 0;
				controller.speed_tracker.course = 0;
			}
		}
	}

	controller.speed_tracker.go_speed = (int32_t)controller.speed_tracker.course;
}

void speed_tracker_set_up_acc(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_UP_ACC))
	{
		controller.speed_tracker.config_.up_acc = value;
		controller.speed_tracker.config_.valid_up_acc = true;
	}
	else{
		controller.speed_tracker.config_.valid_up_acc = false;
	}
}
void speed_tracker_set_down_acc(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_DOWN_ACC))
	{
		controller.speed_tracker.config_.down_acc = value;
		controller.speed_tracker.config_.valid_down_acc = true;
	}
	else
	{
		controller.speed_tracker.config_.valid_down_acc = true;
	}
}

void speed_tracker_set_default(void)
{
	speed_tracker_set_up_acc(controller.speed_tracker.config_.default_up_acc);
	speed_tracker_set_down_acc(controller.speed_tracker.config_.default_down_acc);
}

/************************************************************************************************************/


void postion_tracker_set_max_speed(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_SPEED))
	{
		controller.position_tracker.config_.max_speed = value;
		controller.position_tracker.config_.valid_max_speed = true;
	}
	else
	{
		controller.position_tracker.config_.valid_max_speed = false;
	}
}

void postion_tracker_set_up_acc(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_UP_ACC))
	{
		controller.position_tracker.config_.up_acc = value;
		controller.position_tracker.config_.valid_up_acc = true;
	}
	else
	{
		controller.position_tracker.config_.valid_up_acc = false;
	}
}
void postion_tracker_set_down_acc(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_DOWN_ACC))
	{
		controller.position_tracker.config_.down_acc = value;
		controller.position_tracker.config_.down_acc_quick = 0.5f / (float)controller.position_tracker.config_.down_acc;
		controller.position_tracker.config_.valid_down_acc = true;
	}
	else
	{
		controller.position_tracker.config_.valid_down_acc = false;
	}
}

void postion_tracker_set_default(void)
{
    postion_tracker_set_max_speed(controller.position_tracker.config_.default_max_speed);
    postion_tracker_set_up_acc(controller.position_tracker.config_.default_up_acc);
    postion_tracker_set_down_acc(controller.position_tracker.config_.default_down_acc);
}

void postion_tracker_init(void)
{
	controller.position_tracker.config_.default_max_speed = MOVE_RATED_SPEED;
	controller.position_tracker.config_.default_up_acc = MOVE_RATED_UP_ACC / 10;
	controller.position_tracker.config_.default_down_acc = MOVE_RATED_DOWN_ACC / 10;

	if(!controller.position_tracker.config_.valid_max_speed)	
		postion_tracker_set_max_speed(controller.position_tracker.config_.default_max_speed);		
	if(!controller.position_tracker.config_.valid_up_acc)
		postion_tracker_set_up_acc(controller.position_tracker.config_.default_up_acc);
	if(!controller.position_tracker.config_.valid_down_acc)	
		postion_tracker_set_down_acc(controller.position_tracker.config_.default_down_acc);
	
	controller.position_tracker.config_.speed_locking_stop = MOVE_PULSE_NUM;	
																							
	controller.position_tracker.course_acc_integral = 0;
	controller.position_tracker.course_speed = 0;
	controller.position_tracker.course_speed_integral = 0;
	controller.position_tracker.course_location = 0;

	controller.position_tracker.go_location = 0;
	controller.position_tracker.go_speed = 0;
}

void postion_tracker_new_task(int32_t real_postion, int32_t real_speed)
{
	controller.position_tracker.course_acc_integral = 0;			
	controller.position_tracker.course_speed = real_speed;			
	controller.position_tracker.course_speed_integral = 0;		
	controller.position_tracker.course_location = real_postion;	
}		

void postion_tracker_capture_goal(int32_t goal_position)
{
	int32_t location_sub = goal_position - controller.position_tracker.course_location;

	if(location_sub == 0)
	{
		if((controller.position_tracker.course_speed >= -controller.position_tracker.config_.speed_locking_stop) && (controller.position_tracker.course_speed <= controller.position_tracker.config_.speed_locking_stop))
		{
			controller.position_tracker.course_acc_integral = 0;
			controller.position_tracker.course_speed = 0;
			controller.position_tracker.course_speed_integral = 0;
		}
		else if(controller.position_tracker.course_speed > 0)
		{
			controller.position_tracker.course_acc_integral += (-controller.position_tracker.config_.down_acc);									
			controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;
			controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
			if(controller.position_tracker.course_speed <= 0)
			{
				controller.position_tracker.course_acc_integral = 0;
				controller.position_tracker.course_speed = 0;
			}
		}
		else if(controller.position_tracker.course_speed < 0)
		{
			controller.position_tracker.course_acc_integral += controller.position_tracker.config_.down_acc;										
			controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;
			controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
			if(controller.position_tracker.course_speed >= 0)
			{
				controller.position_tracker.course_acc_integral = 0;
				controller.position_tracker.course_speed = 0;
			}
		}
	}
	else
	{
		if(controller.position_tracker.course_speed == 0)
		{
			if(location_sub > 0)
			{	
				controller.position_tracker.course_acc_integral += controller.position_tracker.config_.up_acc;					
				controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
				controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
			}
			else
			{
				controller.position_tracker.course_acc_integral += (-controller.position_tracker.config_.up_acc);					
				controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
				controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
			}
		}
		else if((location_sub > 0) && (controller.position_tracker.course_speed > 0))
		{
			if(controller.position_tracker.course_speed <= controller.position_tracker.config_.max_speed)
			{
				int32_t need_down_location = (int32_t)((float)controller.position_tracker.course_speed * (float)controller.position_tracker.course_speed * (float)controller.position_tracker.config_.down_acc_quick);
				if(abs(location_sub) > need_down_location)
				{
					if(controller.position_tracker.course_speed < controller.position_tracker.config_.max_speed)
					{
						controller.position_tracker.course_acc_integral += controller.position_tracker.config_.up_acc;					
						controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
						controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
						if(controller.position_tracker.course_speed >= controller.position_tracker.config_.max_speed)
						{
							controller.position_tracker.course_acc_integral = 0;
							controller.position_tracker.course_speed = controller.position_tracker.config_.max_speed;
						}
					}
					else if(controller.position_tracker.course_speed > controller.position_tracker.config_.max_speed)
					{
						controller.position_tracker.course_acc_integral += (-controller.position_tracker.config_.down_acc);					
						controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
						controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
					}
				}
				else
				{
					controller.position_tracker.course_acc_integral += (-controller.position_tracker.config_.down_acc);					
					controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
					controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
					if(controller.position_tracker.course_speed <= 0)
					{
						controller.position_tracker.course_acc_integral = 0;
						controller.position_tracker.course_speed = 0;
					}
				}
			}
			else
			{
				controller.position_tracker.course_acc_integral += controller.position_tracker.config_.down_acc;					
				controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
				controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
				if(controller.position_tracker.course_speed <= 0)
				{
					controller.position_tracker.course_acc_integral = 0;
					controller.position_tracker.course_speed = 0;
				}
			}
		}
		else if((location_sub < 0) && (controller.position_tracker.course_speed < 0))
		{
			if(controller.position_tracker.course_speed >= -controller.position_tracker.config_.max_speed)
			{
				int32_t need_down_location = (int32_t)((float)controller.position_tracker.course_speed * (float)controller.position_tracker.course_speed * (float)controller.position_tracker.config_.down_acc_quick);
				if(abs(location_sub) > need_down_location)
				{
					if(controller.position_tracker.course_speed > -controller.position_tracker.config_.max_speed)
					{
						controller.position_tracker.course_acc_integral += (-controller.position_tracker.config_.up_acc);					
						controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
						controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
						if(controller.position_tracker.course_speed <= -controller.position_tracker.config_.max_speed)
						{
							controller.position_tracker.course_acc_integral = 0;
							controller.position_tracker.course_speed = -controller.position_tracker.config_.max_speed;
						}
					}
					else if(controller.position_tracker.course_speed < -controller.position_tracker.config_.max_speed)
					{
						controller.position_tracker.course_acc_integral += controller.position_tracker.config_.down_acc;					
						controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
						controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
					}
				}
				else
				{
					controller.position_tracker.course_acc_integral += controller.position_tracker.config_.down_acc;					
					controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
					controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
					if(controller.position_tracker.course_speed >= 0)
					{
						controller.position_tracker.course_acc_integral = 0;
						controller.position_tracker.course_speed = 0;
					}
				}
			}
			else
			{
				controller.position_tracker.course_acc_integral += controller.position_tracker.config_.down_acc;					
				controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
				controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
				if(controller.position_tracker.course_speed >= 0)
				{
					controller.position_tracker.course_acc_integral = 0;
					controller.position_tracker.course_speed = 0;
				}
			}
		}
		else if((location_sub < 0) && (controller.position_tracker.course_speed > 0))
		{
			controller.position_tracker.course_acc_integral += (-controller.position_tracker.config_.down_acc);					
			controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
			controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
			if(controller.position_tracker.course_speed <= 0)
			{
				controller.position_tracker.course_acc_integral = 0;
				controller.position_tracker.course_speed = 0;
			}
		}
		else if(((location_sub > 0) && (controller.position_tracker.course_speed < 0)))
		{
			controller.position_tracker.course_acc_integral += controller.position_tracker.config_.down_acc;					
			controller.position_tracker.course_speed += controller.position_tracker.course_acc_integral / CONTROL_FREQ_HZ;	
			controller.position_tracker.course_acc_integral = controller.position_tracker.course_acc_integral % CONTROL_FREQ_HZ;
			if(controller.position_tracker.course_speed >= 0)
			{
				controller.position_tracker.course_acc_integral = 0;
				controller.position_tracker.course_speed = 0;
			}
		}		
	}

	controller.position_tracker.course_speed_integral += controller.position_tracker.course_speed;										
	controller.position_tracker.course_location += controller.position_tracker.course_speed_integral / CONTROL_FREQ_HZ;		
	controller.position_tracker.course_speed_integral = controller.position_tracker.course_speed_integral % CONTROL_FREQ_HZ;	

	controller.position_tracker.go_location = (int32_t)controller.position_tracker.course_location;
	controller.position_tracker.go_speed = (int32_t)controller.position_tracker.course_speed;
}


/************************************************************************************************************/

void position_interp_init(void)
{
	controller.position_interp.record_location = 0;
	controller.position_interp.record_location_last = 0;
	controller.position_interp.est_position = 0;
	controller.position_interp.est_speed_mut = 0;
	controller.position_interp.est_speed = 0;

	controller.position_interp.go_location = 0;
	controller.position_interp.go_speed = 0;
}

void position_interp_new_task(int32_t real_postion, int32_t real_speed)
{
	controller.position_interp.record_location = real_postion;
	controller.position_interp.record_location_last = real_postion;
	controller.position_interp.est_position = real_postion;
	controller.position_interp.est_speed = real_speed;
}

void position_interp_capture_goal(int32_t goal_position)
{
	controller.position_interp.record_location_last = controller.position_interp.record_location;
	controller.position_interp.record_location = goal_position;

	controller.position_interp.est_speed_mut += (((controller.position_interp.record_location - controller.position_interp.record_location_last) * CONTROL_FREQ_HZ)
							  + ((int32_t)(controller.position_interp.est_speed  << 6) - (int32_t)(controller.position_interp.est_speed)));
	controller.position_interp.est_speed = (controller.position_interp.est_speed_mut >> 6);							//(对64取整)(向0取整)(保留符号位)
	controller.position_interp.est_speed_mut = (controller.position_interp.est_speed_mut - (controller.position_interp.est_speed << 6));	//(对64取余)(向0取整)(保留符号位)

	controller.position_interp.est_position = controller.position_interp.record_location;

	controller.position_interp.go_location = controller.position_interp.est_position;
	controller.position_interp.go_speed = controller.position_interp.est_speed;
}


/************************************************************************************************************/


void traj_tracker_set_down_acc(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= MOVE_RATED_DOWN_ACC))
	{
		controller.traj_tracker.config_.down_acc = value;
		controller.traj_tracker.config_.valid_down_acc = true;
	}
	else{
		controller.traj_tracker.config_.valid_down_acc = false;
	}
}	

void traj_tracker_set_over_time(uint16_t value)
{
	if((value >= controller.traj_tracker.config_.min_overtime) && (value <= controller.traj_tracker.config_.max_overtime))
	{
		controller.traj_tracker.config_.overtime = value;
		controller.traj_tracker.config_.valid_overtime = true;
	}
	else
	{
		controller.traj_tracker.config_.valid_overtime = false;
	}
}
void traj_tracker_set_default(void)
{
	traj_tracker_set_down_acc(controller.traj_tracker.config_.default_down_acc);
	traj_tracker_set_over_time(controller.traj_tracker.config_.default_overtime);
}					
		    
void traj_tracker_init(void)
{
	controller.traj_tracker.config_.default_down_acc = MOVE_RATED_DOWN_ACC / 10;
	controller.traj_tracker.config_.default_overtime = 200;

	if(!controller.traj_tracker.config_.valid_down_acc)	
		traj_tracker_set_down_acc(controller.traj_tracker.config_.default_down_acc);
	if(!controller.traj_tracker.config_.valid_overtime)
		traj_tracker_set_over_time(controller.traj_tracker.config_.default_overtime);
	
	controller.traj_tracker.dyn_speed_acc = 0;

	controller.traj_tracker.record_timer = 0;
	controller.traj_tracker.overtime_flag = false;
	controller.traj_tracker.record_speed = 0;
	controller.traj_tracker.record_location = 0;

	controller.traj_tracker.speed_course_dec = 0;
	controller.traj_tracker.speed_course = 0;
	controller.traj_tracker.location_course_dec = 0;
	controller.traj_tracker.location_course = 0;

	controller.traj_tracker.go_location = 0;
	controller.traj_tracker.go_speed = 0;
}																		

void traj_tracker_new_task(int32_t real_postion, int32_t real_speed)
{
	controller.traj_tracker.record_timer = 0;								
	controller.traj_tracker.overtime_flag = false;			

	controller.traj_tracker.speed_course_dec = 0;						
	controller.traj_tracker.speed_course = real_speed;
	controller.traj_tracker.location_course_dec = 0;
	controller.traj_tracker.location_course = real_postion;	
}	


void traj_tracker_capture_goal(int32_t goal_position, int32_t goal_speed)
{
	if( (goal_speed != controller.traj_tracker.record_speed) || (goal_position != controller.traj_tracker.record_location))
	{
		controller.traj_tracker.record_timer = 0;
		controller.traj_tracker.record_speed = goal_speed;
		controller.traj_tracker.record_location = goal_position;

		controller.traj_tracker.dyn_speed_acc = (int32_t)((float)(goal_speed + controller.traj_tracker.speed_course) * (float)(goal_speed - controller.traj_tracker.speed_course)
											    / (float)(2 * (goal_position - controller.traj_tracker.location_course)));

		controller.traj_tracker.overtime_flag = false;
	}
	else
	{
		if(controller.traj_tracker.record_timer >= (200 * 1000))
			controller.traj_tracker.overtime_flag = true;
		else
			controller.traj_tracker.record_timer += CONTROL_PERIOD_US;
	}
	if(controller.traj_tracker.overtime_flag)
	{
		if(controller.traj_tracker.speed_course == 0)
		{
			controller.traj_tracker.speed_course_dec = 0;
			controller.traj_tracker.speed_course = 0;
		}
		else if(controller.traj_tracker.speed_course > 0)
		{
			controller.traj_tracker.speed_course_dec += (-controller.traj_tracker.config_.down_acc);	
			controller.traj_tracker.speed_course += controller.traj_tracker.speed_course_dec / CONTROL_FREQ_HZ;	
			controller.traj_tracker.speed_course_dec = controller.traj_tracker.speed_course_dec % CONTROL_FREQ_HZ;
			if(controller.traj_tracker.speed_course <= 0)
			{
				controller.traj_tracker.speed_course_dec = 0;
				controller.traj_tracker.speed_course = 0;
			}
		}
		else
		{
			controller.traj_tracker.speed_course_dec += (controller.traj_tracker.config_.down_acc);	
			controller.traj_tracker.speed_course += controller.traj_tracker.speed_course_dec / CONTROL_FREQ_HZ;	
			controller.traj_tracker.speed_course_dec = controller.traj_tracker.speed_course_dec % CONTROL_FREQ_HZ;
			if(controller.traj_tracker.speed_course >= 0)
			{
				controller.traj_tracker.speed_course_dec = 0;
				controller.traj_tracker.speed_course = 0;
			}
		}
	}
	else
	{
		controller.traj_tracker.speed_course_dec += (controller.traj_tracker.dyn_speed_acc);	
		controller.traj_tracker.speed_course += controller.traj_tracker.speed_course_dec / CONTROL_FREQ_HZ;	
		controller.traj_tracker.speed_course_dec = controller.traj_tracker.speed_course_dec % CONTROL_FREQ_HZ;
	}

	controller.traj_tracker.location_course_dec += (controller.traj_tracker.speed_course);									
	controller.traj_tracker.location_course += controller.traj_tracker.location_course_dec / CONTROL_FREQ_HZ;	
	controller.traj_tracker.location_course_dec = controller.traj_tracker.location_course_dec % CONTROL_FREQ_HZ;

	controller.traj_tracker.go_location = (int32_t)controller.traj_tracker.location_course;
	controller.traj_tracker.go_speed = (int32_t)controller.traj_tracker.speed_course;
}

