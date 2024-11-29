#ifndef __GLAZER_API_H__
#define __GLAZER_API_H__

#include "board_cfg.h"


/*********************************************************/
/**********在此定义由glazer主控与步进闭环板通信协议**********/
/**
 * 该协议规定了由主板发送来的串口数据：由以下几部分构成
 * SOF(前导码)(2字节)+ADDR(目标地址)(1字节)+FC(功能码)(1字节)+DATA(数据包)(8字节)+CRC(校验码)(1字节)+TAIL(尾帧)(1字节)
 * 
 * 1.其中SOF：
 * 主板向抬升电机步进驱动发送前导码：0xAA 0xAF
 * 主板向冰舱电机步进驱动发送前导码：0xAA 0xBF
 * 
 * 2.目标地址
 * 抬升电机步进驱动addr: 0x01
 * 冰仓电机步进驱动addr: 0x02
 * 
 * 3.功能码：
 *   0x01: 发送控制量
 *   0x02: 编码器清零
 *   0x03: 清除堵转标志位
 *   0x04: 设置控制pid值
 *   0x05: 设置控制dce值
 *   0x06: 电机进入校准并重启
 *   0x07: 紧急刹车
 * 
 * 4.数据包占用8个字节
 * 
 * 5.CRC校验
 * 
 * 6.尾帧：0xFF
 */

void glazer_uart_rx_data_unpack(uint8_t *buff);

void glazer_entry_encoder_calibration(void);
void glazer_set_motor_pos(int32_t _pos, int32_t _spd);
void glazer_set_motor_spd(int32_t _spd);
void glazer_reset_encoder_pos(void);
void glazer_reset_stall_flag(void);
void glazer_set_pid_parm(uint16_t _kp, uint16_t _ki, uint16_t _kd);
void glazer_set_dce_parm(uint16_t _kp, uint16_t _ki, uint16_t _kv, uint16_t _kd);
void glazer_set_break();
int glazer_get_curr_ctrl_mode();


int32_t glazer_get_encoder_pos();
int32_t glazer_get_encoder_spd();
bool glazer_get_stail_flag(void);
bool glazer_get_is_calibration();


#endif // !__GLAZER_API_H__
