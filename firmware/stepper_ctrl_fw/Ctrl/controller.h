#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "board_cfg.h"

typedef struct
{
    int32_t default_kp;
    int32_t default_ki;
    int32_t default_kd;

    bool valid_kp;
    bool valid_ki;
    bool valid_kd;
}pid_config_t;

typedef struct
{
    pid_config_t config_;

	int32_t	kp;
    int32_t ki;
    int32_t kd;						
	int32_t	v_error;
    int32_t v_error_last;	
	int32_t	output_kp;
    int32_t output_ki;
    int32_t output_kd;							
	int32_t	i_mut; 
    int32_t i_dec;							
	int32_t	output;										
}pid_t;

void pid_set_kp(uint16_t _k);		
void pid_set_ki(uint16_t _k);	
void pid_set_kd(uint16_t _k);		
void pid_set_default(void);	

void pid_init(void);

typedef struct
{
    int32_t default_kp;
    int32_t default_ki;
    int32_t default_kv;
    int32_t default_kd;

    bool valid_kp;
    bool valid_ki;
    bool valid_kv;
    bool valid_kd;
}dec_config_t;

typedef struct
{
    dec_config_t config_;
    
	int32_t	kp;
    int32_t ki;
    int32_t kv;
    int32_t kd;			
	int32_t	p_error;
    int32_t v_error;											
	int32_t	output_kp;
    int32_t output_ki;
    int32_t output_kd;													
	int32_t	i_mut;
    int32_t i_dec;													
	int32_t	output;															
}dce_t;

void dce_set_kp(uint16_t _k);
void dce_set_ki(uint16_t _k);
void dce_set_kv(uint16_t _k);
void dce_set_kd(uint16_t _k);
void dce_set_default(void);
void dce_init(void);

typedef struct 
{
    float default_up_rate;
    float default_down_rate;
    bool valid_up_rate;
    bool valid_down_rate;
    int32_t up_rate;
    int32_t down_rate;
}current_config_t;

typedef struct
{
    current_config_t config_;
    int32_t course_mut;
    int32_t course;
    int32_t go_current;
}current_ctrl_t;

void current_tracker_set_up_rate(int32_t value);					
void current_tracker_set_down_rate(int32_t value);					
void current_tracker_set_default(void);											

void current_tracker_init(void);														
void current_tracker_new_task(int16_t real_current);				
void current_tracker_capture_goal(int32_t goal_current);		


typedef struct 
{
    float default_up_acc;
    float default_down_acc;
    bool valid_up_acc;
    bool valid_down_acc;
    int32_t up_acc;
    int32_t down_acc;
}speed_config_t;

typedef struct
{
    speed_config_t config_;
    int32_t course_mut;
    int32_t course;
    int32_t go_speed;
}speed_ctrl_t;

void speed_tracker_set_up_acc(int32_t value);	
void speed_tracker_set_down_acc(int32_t value);	
void speed_tracker_set_default(void);					
	
void speed_tracker_init(void);								
void speed_tracker_new_task(int32_t real_speed);		
void speed_tracker_capture_goal(int32_t goal_speed);	

typedef struct 
{
    float default_max_speed;
    float default_up_acc;
    float default_down_acc;
    bool valid_max_speed;
    bool valid_up_acc;
    bool valid_down_acc;
    int32_t	max_speed;
    int32_t up_acc;
    int32_t down_acc;
    float down_acc_quick;
    int32_t	speed_locking_stop;
}position_config_t;

typedef struct
{
    position_config_t config_;
	int32_t	course_acc_integral;		
	int32_t	course_speed;					
	int32_t	course_speed_integral;	
	int32_t	course_location;		
	int32_t	go_location;
	int32_t	go_speed;		
}position_ctrl_t;

void postion_tracker_set_maxSpeed(int32_t value);	
void postion_tracker_set_up_acc(int32_t value);		
void postion_tracker_set_down_acc(int32_t value);	
void postion_tracker_set_default(void);					

void postion_tracker_init(void);																				
void postion_tracker_new_task(int32_t real_postion, int32_t real_speed);
void postion_tracker_capture_goal(int32_t goal_position);						

typedef struct
{
	int32_t		record_location;				
	int32_t		record_location_last;		
	int32_t		est_position;					
	int32_t		est_speed_mut;				
	int32_t		est_speed;							

	int32_t		go_location;	
	int32_t		go_speed;		
}position_interp_t;

void position_interp_init(void);				
void position_interp_new_task(int32_t real_postion, int32_t real_speed);
void position_interp_capture_goal(int32_t goal_position);								


typedef struct
{
    float max_overtime;
    float min_overtime;

    float default_down_acc;
    float default_overtime;
    
    bool valid_down_acc;
    bool valid_overtime;

    int32_t down_acc;
    uint16_t overtime; 
}traj_tracker_config_t;

typedef struct
{
    traj_tracker_config_t config_;
    int32_t	dyn_speed_acc;
    int32_t	record_timer;
    bool overtime_flag;		
	int32_t	record_speed;			
	int32_t	record_location;	
	int32_t	speed_course_dec;		
	int32_t	speed_course;		
	int32_t	location_course_dec;	
	int32_t	location_course;	
	int32_t	go_location;		
	int32_t	go_speed;			
}traj_tracker_ctrl_t;

void traj_tracker_set_down_acc(int32_t value);		
void traj_tracker_set_over_time(uint16_t value);	
void traj_tracker_set_default(void);					
		    
void traj_tracker_init(void);																					
void traj_tracker_new_task(int32_t real_postion, int32_t real_speed);			
void traj_tracker_capture_goal(int32_t goal_position, int32_t goal_speed);


typedef struct
{
    pid_t pid;
    dce_t dce;
    current_ctrl_t current_tracker;
    speed_ctrl_t	speed_tracker;
    position_ctrl_t	position_tracker;
    position_interp_t position_interp;
    traj_tracker_ctrl_t traj_tracker;
}controller_t;
extern controller_t controller;

void controller_init(void);
void controller_clear_integral(void);
#endif