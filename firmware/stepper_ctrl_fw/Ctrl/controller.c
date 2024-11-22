#include "controller.h"

controller_t controller;


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


void postion_tracker_set_maxSpeed(int32_t value)
{
	value = abs(value);
	if((value > 0) && (value <= Move_Rated_Speed))
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
    postion_tracker_set_maxSpeed(controller.position_tracker.config_.default_max_speed);
    postion_tracker_set_up_acc(controller.position_tracker.config_.default_up_acc);
    postion_tracker_set_down_acc(controller.position_tracker.config_.default_down_acc);
}


