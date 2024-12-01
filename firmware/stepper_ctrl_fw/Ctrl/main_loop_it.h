#ifndef __MAIN_LOOP_IT_H__
#define __MAIN_LOOP_IT_H__

#include "board_cfg.h"

typedef struct
{
    // 中断计数器
    uint32_t systick_count;
    uint32_t dma1_ch3_count;
    uint32_t uart3_count;
} loop_it_t;
extern loop_it_t loop_it;

void loop_it_sysTick_20khz(void);
void loop_it_priority_overlay(void);

#endif
