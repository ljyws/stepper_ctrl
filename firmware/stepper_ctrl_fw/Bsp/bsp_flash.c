#include "bsp_flash.h"


stockpile_flash_t stockpile_app_firmware = {
    .begin_addr = STOCKPILE_APP_FIRMWARE_ADDR,
    .area_size  = STOCKPILE_APP_FIRMWARE_SIZE,
    .page_num   = (STOCKPILE_APP_FIRMWARE_SIZE / STOCKPILE_PAGE_SIZE),
    .asce_write_addr = 0,
};

stockpile_flash_t stockpile_quick_cali = {
    .begin_addr = STOCKPILE_APP_CALI_ADDR,
    .area_size  = STOCKPILE_APP_CALI_SIZE,
    .page_num   = (STOCKPILE_APP_CALI_SIZE / STOCKPILE_PAGE_SIZE),
    .asce_write_addr = 0,
};

stockpile_flash_t stockpile_data = {
    .begin_addr = STOCKPILE_APP_DATA_ADDR,
    .area_size  = STOCKPILE_APP_DATA_SIZE,
    .page_num   = (STOCKPILE_APP_DATA_SIZE / STOCKPILE_PAGE_SIZE),
    .asce_write_addr = 0,
};


/**
 * @brief flase 擦除
 * 
 * @param stockpile 
 */
void flash_erase(stockpile_flash_t *stockpile)
{
	uint32_t count;
	HAL_FLASH_Unlock();	
	for(count = 0; count < stockpile->page_num; count++)
	{
		FLASH_EraseInitTypeDef erase_config;
		uint32_t page_error;
		erase_config.TypeErase   = FLASH_TYPEERASE_PAGES;															
		erase_config.PageAddress = stockpile->begin_addr + (count * STOCKPILE_PAGE_SIZE);
		erase_config.NbPages     = 1;																						
		HAL_FLASHEx_Erase(&erase_config, &page_error);
		FLASH_WaitForLastOperation(HAL_MAX_DELAY);
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}
	HAL_FLASH_Lock();
}

/**
 * @brief  flash写入开始
 * 
 * @param  stockpile
 */
void flash_write_begin(stockpile_flash_t *stockpile)
{
	HAL_FLASH_Unlock();	
	stockpile->asce_write_addr = stockpile->begin_addr;
}

/**
 * @brief flash写入结束
 * 
 * @param stockpile 
 */
void flash_write_end(stockpile_flash_t *stockpile)
{
	HAL_FLASH_Lock();	
}

/**
 * @brief flash设置写入地址
 * 
 * @param stockpile 
 * @param write_add 
 */
void flash_set_write_addr(stockpile_flash_t *stockpile, uint32_t write_addr)
{
	if(write_addr < stockpile->begin_addr)						
        return;
	if(write_addr > stockpile->begin_addr + stockpile->area_size)	
        return;
	stockpile->asce_write_addr = write_addr;
}

/**
 * @brief  flash半字写入
 * @param  stockpile
 * @param  data		
 * @param  num		半字数量
 * @retval NULL
 */
void flash_write_halfword(stockpile_flash_t *stockpile, uint16_t *data, uint32_t num)
{
	if(stockpile->asce_write_addr < stockpile->begin_addr)									
        return;
	if((stockpile->asce_write_addr + num * 2) > stockpile->begin_addr + stockpile->area_size)	
        return;
	
	for(uint32_t i=0; i<num; i++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, stockpile->asce_write_addr, (uint64_t)data[i]) == HAL_OK)
			stockpile->asce_write_addr += 2;
	}
}

/**
 * @brief  flash 按字写入
 * @param  stockpile	
 * @param  data		
 * @param  num	
 * @retval NULL
 */
void flash_write_word(stockpile_flash_t *stockpile, uint32_t *data, uint32_t num)
{
	if(stockpile->asce_write_addr < stockpile->begin_addr)									
        return;
	if((stockpile->asce_write_addr + num * 4) > stockpile->begin_addr + stockpile->area_size)	
        return;
	
	for(uint32_t i=0; i<num; i++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, stockpile->asce_write_addr, (uint64_t)data[i]) == HAL_OK)
			stockpile->asce_write_addr += 4;
	}
}
