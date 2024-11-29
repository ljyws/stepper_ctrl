#include "bsp_mcu.h"


void mcu_software_reset(void)
{
    /* set FAULTMASK */
    __set_FAULTMASK(1);     //关掉所有中断
    NVIC_SystemReset();     //软复位
}
