#ifndef __ENCODER_CALIB_H__
#define __ENCODER_CALIB_H__

#include "board_cfg.h"

#define ENC_CALI_GATHER_QUANTITY	16	            //每个采集点采集数量

typedef enum
{
	CALI_NO_ERROR = 0x00,				
	CALI_ERROR_AVERAGE_DIR,				//平均值方向错误
	CALI_ERROR_AVERAGE_CONTINUITY,	    //平均值连续性错误
	CALI_ERROR_PHASESTEP,				//阶跃次数错误
	CALI_ERROR_ANALYSIS_QUANTITY,		//解析数据量错误
}cali_error_e;

typedef enum
{
	CALI_DISABLE = 0x00,				//失能
	CALI_FORWARD_ENCLDER_AUTOCALI,	    //编码器正转自动校准
	CALI_FORWARE_MEASURE,			    //正向测量
	CALI_REVERSE_RET,					//反向回退
	CALI_REVERSE_GAP,					//反向消差
	CALI_REVERSE_MEASURE,			    //反向测量
	CALI_OPERATION,						//解算
}cali_state_e;



typedef struct
{
	bool trigger;			                                        //触发校准
	cali_error_e error_code;		                                //校准错误代码
	uint32_t error_data;		                                    //校准错误数据
	cali_state_e state;					                            //校准状态
	uint32_t out_location;		                                    //输出位置
	uint16_t gather_count;							                //采集计数
	uint16_t coder_data_gather[ENC_CALI_GATHER_QUANTITY];	        //采集点每次数据
	uint16_t coder_data_f[200+1];	                                //校准读取的数据
	uint16_t coder_data_r[200+1];	                                //校准读取的数据
	bool dir;		                                                //校准数据方向
	int32_t rcd_x;			                                        //寻找区间下标及阶跃差值
    int32_t rcd_y;	
	uint32_t result_num;				                            //统计数量
}encoder_cali_t;
extern encoder_cali_t encode_cali;

/**
 * @brief 取余
 * 
 * @param a 
 * @param b 
 * @return uint32_t 
 */
static inline uint32_t cycle_rem(uint32_t a,uint32_t b)
{
    return (a+b)%b;
}		

/**
 * @brief 取循环差
 * 
 * @param a 
 * @param b 
 * @param cyc 
 * @return int32_t 
 */
static inline int32_t cycle_sub(int32_t a, int32_t b, int32_t cyc)
{
	int32_t sub_data;

	sub_data = a - b;
	if(sub_data > (cyc >> 1))		sub_data -= cyc;
	if(sub_data < (-cyc >> 1))	sub_data += cyc;
	return sub_data;
}

/**
 * @brief 取循环平均值
 * 
 * @param a 
 * @param b 
 * @param cyc 
 * @return int32_t 
 */
static inline int32_t cycle_average(int32_t a, int32_t b, int32_t cyc)
{
    int32_t sub_data;
	int32_t ave_data;

	sub_data = a - b;
	ave_data = (a + b) >> 1;

	if(abs(sub_data) > (cyc >> 1))
	{
		if(ave_data >= (cyc >> 1))	ave_data -= (cyc >> 1);
		else												ave_data += (cyc >> 1);
	}
	return ave_data;
}

/**
 * @brief 取循环平均值
 * 
 * @param data 
 * @param length 
 * @param cyc 
 * @return int32_t 
 */
static inline int32_t cycle_data_average(uint16_t *data, uint16_t length, int32_t cyc)
{
	int32_t sum_data = 0;	//积分和
	int32_t sub_data;		//差
	int32_t	diff_data;		//微分

	//加载第一值
	sum_data += (int32_t)data[0];
	//加载后面的值
	for(uint16_t i=1; i<length; i++)
	{
		diff_data = (int32_t)data[i];
		sub_data = (int32_t)data[i] - (int32_t)data[0];
		if(sub_data > (cyc >> 1))		diff_data = (int32_t)data[i] - cyc;
		if(sub_data < (-cyc >> 1))	diff_data = (int32_t)data[i] + cyc;
		sum_data += diff_data;
	}
	//计算平均值
	sum_data = sum_data / length;
	//平均值标准化
	if(sum_data < 0)		sum_data += cyc;
	if(sum_data > cyc)	sum_data -= cyc;
	return sum_data;
}


void encoder_calibration_init(void);				//校准器初始化
void encoder_calibration_interrupt_callback(void);	//校准器中断回调(稳定中断调用)
void encoder_calibration_loop_callback(void);	    //校准器主程序回调(非中断调用)


#endif
