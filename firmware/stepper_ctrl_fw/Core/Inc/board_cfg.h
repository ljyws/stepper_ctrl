#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__



#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <spi.h>
#include <tim.h>
#include <main.h>
#include <gpio.h>
#include <usart.h>
// #include <dma.h>



#include "sin_map.h"
#include "tb67h450.h"
// #include "motor.h"
#include "controller.h"
#include "encoder.h"
#include "encoder_calibration.h"

// #include "position_interp.h"
// #include "controller.traj_tracker.h"
// #include "main_ctrl.h"

#include "bsp_flash.h"

/********************  硬件配置区  ********************/
#define CURRENE_RATED_CURRENT		(3000)		        //额定电流(mA)
#define CURRENT_CALI_CURRENT		(2000)		        //校准电流(mA)
/********************  运动参数配置区  ********************/
#define MOVE_STEP_NUM					((int32_t)(200))											//(使用的电机单圈步数)(每步磁场旋转90°)
#define MOVE_DIVEDE_NUM					((int32_t)(256))											//(每步柔性件控制细分量)
#define MOVE_PULSE_NUM					((int32_t)(MOVE_STEP_NUM * MOVE_DIVEDE_NUM))		        //(电机单圈脉冲数)
#define Move_Rated_Speed				((int32_t)(50 * MOVE_PULSE_NUM))							//(额定转速)(50转每秒)
#define MOVE_RATED_UP_ACC				((int32_t)(1000 * MOVE_PULSE_NUM))							//(固件额定加速加速度)(1000r/ss)
#define MOVE_RATED_DOWN_ACC				((int32_t)(1000 * MOVE_PULSE_NUM))							//(固件额定减速加速度)(1000r/ss)
#define MOVE_RATED_UP_CURRENT_RATE	    ((int32_t)(20 * CURRENE_RATED_CURRENT))					    //(固件额定增流梯度)(20倍额定/s)
#define MOVE_RATED_DOWN_CURRENT_RATE		((int32_t)(20 * CURRENE_RATED_CURRENT))					    //(固件额定减流梯度)(20倍额定/s)



#define CALI_ENCODER_BIT				((int32_t)(14))									//(编码器位宽)(14位输出精度)
#define CALI_ENCODER_RES				((int32_t)((0x00000001U) << CALI_ENCODER_BIT))	//(编码器分辨率)(2^14 = 16384)(16k分辨率)(占用32k校准空间)
#define CALI_GATHER_ENCODER_RES         ((int32_t)(CALI_ENCODER_RES / MOVE_STEP_NUM))	//(校准每采集步编码器分辨率)



#define CONTROL_FREQ_HZ			(20000)							//控制频率_hz
#define CONTROL_PERIOD_US		(1000000 / CONTROL_FREQ_HZ)		//控制周期_us


/*******************************************************FLASH 分区**************************************************************************/

#define	STOCKPILE_APP_FIRMWARE_ADDR	    (0x08008000)		    //起始地址
#define	STOCKPILE_APP_FIRMWARE_SIZE		(0x00010000)		//Flash容量    47K    XDrive(APP_FIRMWARE)
//APP_CALI
#define	STOCKPILE_APP_CALI_ADDR			(0x08018000)		//起始地址
#define	STOCKPILE_APP_CALI_SIZE			(0x00020000)		//Flash容量    32K    XDrive(APP_CALI)(可容纳16K-2byte校准数据-即最大支持14位编码器的校准数据)
//APP_DATA
#define	STOCKPILE_APP_DATA_ADDR			(0x0801FC00)		//起始地址
#define	STOCKPILE_APP_DATA_SIZE			(0x00000400)		//Flash容量     1K    XDrive(APP_DATA)

#define STOCKPILE_RAM_APP_START         (0x20000000)
#define STOCKPILE_RAM_APP_SIZE          (0x00004F00)		//19K768字节

#define STOCKPILE_RAM_SHARED_START      (0x20004F00)
#define STOCKPILE_RAM_SHARED_SIZE       (0x00000100)		//256字节




typedef enum configStatus_t
{
    CONFIG_RESTORE = 0,
    CONFIG_OK,
    CONFIG_COMMIT
} configStatus_t;


typedef struct
{
	configStatus_t configStatus;
	uint32_t canNodeId;
	int32_t encoderHomeOffset;
	uint32_t defaultMode;
	int32_t currentLimit;
	int32_t velocityLimit;
	int32_t velocityAcc;
	int32_t calibrationCurrent;
	int32_t dce_kp;
	int32_t dce_kv;
	int32_t dce_ki;
	int32_t dce_kd;
	bool enableMotorOnBoot;
	bool enableStallProtect;
} board_config_t;

extern board_config_t board_config;


#endif
