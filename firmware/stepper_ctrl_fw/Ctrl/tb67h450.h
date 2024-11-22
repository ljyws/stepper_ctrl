#ifndef __TB67H450_H__
#define __TB67H450_H__

#include <stdbool.h>
#include <stdlib.h>

#include "board_cfg.h"


typedef struct
{
	uint16_t sinMapPtr;
	int16_t  sinMapData;
	uint16_t dacValue12Bits;
}fast_sin_2_dac_t;


extern fast_sin_2_dac_t phaseB;
extern fast_sin_2_dac_t phaseA;
		
bool tb67h450_init();

void tb67h450_sleep();

void tb67h450_break();

void tb67h450_set_input_A(bool _statusAp, bool _statusAm);

void tb67h450_set_input_B(bool _statusBp, bool _statusBm);

void tb67h450_set_foc_current_vector(uint32_t _directionInCount, int32_t _current_mA);

void tb67h450_set_two_coils_current(uint16_t _currentA_3300mAIn12Bits, uint16_t _currentB_3300mAIn12Bits);

void tb67h450_dac_output_voltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits);







#endif
