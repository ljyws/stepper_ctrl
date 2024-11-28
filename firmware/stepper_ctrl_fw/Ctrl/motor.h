#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "board_cfg.h"

typedef enum
{
	CTRL_STATE_STOP	= 0,
	CTRL_STATE_FINISH,
	CTRL_STATE_RUNNING,
	CTRL_STATE_OVERLOAD,
	CTRL_STATE_STALL,
}motor_state_e;

typedef enum
{
	CTRL_MODE_STOP      = 0x10,		
	CTRL_MODE_POSITION	= 0x20,
	CTRL_MODE_SPEED     = 0x21,
	CTRL_MODE_CURRENT   = 0x22,
	CTRL_MODE_TRAJ      = 0x23
}motor_ctrl_mode_e;

typedef struct
{
    motor_ctrl_mode_e ctrl_mode;
    bool is_enable_stall;
}motor_default_config_t;

typedef struct
{
    motor_default_config_t default_confit_;
	motor_state_e state_;		

	bool valid_mode;																										
	motor_ctrl_mode_e requested_mode_;		
    motor_ctrl_mode_e current_mode_;	

	bool valid_stall_switch;																									
	bool stall_switch;																											

	int32_t	real_lap_location;																											
	int32_t	real_lap_location_last;																										
	int32_t	real_location;																													
	int32_t	real_location_last;																											

	int32_t	est_speed_mut;																														
	int32_t	est_speed;																																
	int32_t	est_lead_location;																												
	int32_t	est_lead_location_debug;																								
	int32_t	est_location;																														
	int32_t	est_error;																															

	int32_t	goal_location;																												
	int32_t	goal_speed;																															
	int16_t	goal_current;																														
	bool goal_disable;																														
	bool goal_brake;																																

	int32_t	soft_location;																														
	int32_t	soft_speed;																																
	int16_t	soft_current;																															
	bool soft_disable;																														
	bool soft_brake;																																
	bool soft_new_curve;																													
																										
	int32_t	foc_location;																															
	int32_t	foc_current;																														
																											
	uint32_t stall_time_us;																													
	bool stall_flag;																															

	uint32_t overload_time_us;																												
	bool overload_flag;																		
}motor_ctrl_t;
extern motor_ctrl_t motor_ctrl;




void controller_cur_2_electric(int16_t current);
void controller_pid_2_electric(int32_t _speed);
void controller_dce_2_electric(int32_t _location, int32_t _speed);

void motor_ctrl_set_motor_mode(motor_ctrl_mode_e _mode);	
void motor_ctrl_set_stall_switch(bool _switch);	
void motor_ctrl_set_default(void);						

void motor_ctrl_write_goal_location(int32_t value);								
void motor_ctrl_write_goal_speed(int32_t value);						
void motor_ctrl_write_goal_current(int16_t value);			
void motor_ctrl_write_goal_disable(uint16_t value);				
void motor_ctrl_write_goal_brake(uint16_t value);										
int32_t motor_control_advance_compen(int32_t _speed);	

void motor_ctrl_init(void);																			
void motor_ctrl_callback(void);																		
void motor_ctrl_clear_integral(void);										
void motor_ctrl_clear_stall(void);		

#endif // !__MOTOR_CTRL_H__
