#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "board_cfg.h"

#define STOCKPILE_PAGE_SIZE		0x400U

/**
 * @brief flash分区表
 * 
 */
typedef struct
{
    uint32_t begin_addr;            //起始地址
    uint32_t area_size;             //区域大小
    uint32_t page_num;              //页数量
    uint32_t asce_write_addr;       //写地址
}stockpile_flash_t;


extern stockpile_flash_t stockpile_app_firmware;        //APP分区
extern stockpile_flash_t stockpile_quick_cali;          //编码器校准数据分区
extern stockpile_flash_t stockpile_data;

void flash_erase(stockpile_flash_t *stockpile);

void flash_write_begin(stockpile_flash_t *stockpile);

void flash_write_end(stockpile_flash_t *stockpile);

void flash_set_write_addr(stockpile_flash_t *stockpile, uint32_t write_addr);

void flash_write_halfword(stockpile_flash_t *stockpile, uint16_t *data, uint32_t num);

void flash_write_word(stockpile_flash_t *stockpile, uint32_t *data, uint32_t num);

#endif
