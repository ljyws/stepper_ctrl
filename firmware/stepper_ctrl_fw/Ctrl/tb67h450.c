#include "tb67h450.h"

fast_sin_2_dac_t phaseB;
fast_sin_2_dac_t phaseA;

bool tb67h450_init()
{
	return true;
}
void tb67h450_sleep()
{
	phaseA.dacValue12Bits = 0;
	phaseB.dacValue12Bits = 0;

	tb67h450_set_two_coils_current(phaseA.dacValue12Bits, phaseB.dacValue12Bits);

	tb67h450_set_input_A(false, false);
	tb67h450_set_input_B(false, false);
}

void tb67h450_break()
{
	phaseA.dacValue12Bits = 0;
	phaseB.dacValue12Bits = 0;

	tb67h450_set_two_coils_current(phaseA.dacValue12Bits, phaseB.dacValue12Bits);

	tb67h450_set_input_A(true, true);
	tb67h450_set_input_B(true, true);
}

void tb67h450_dac_output_voltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, _voltageA_3300mVIn12bits >> 2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, _voltageB_3300mVIn12bits >> 2);
}


void tb67h450_set_foc_current_vector(uint32_t _directionInCount, int32_t _current_mA)
{
	phaseB.sinMapPtr = (_directionInCount) & (0x000003FF);
    phaseA.sinMapPtr = (phaseB.sinMapPtr + (256)) & (0x000003FF);

    phaseA.sinMapData = sin_pi_m2[phaseA.sinMapPtr];
    phaseB.sinMapData = sin_pi_m2[phaseB.sinMapPtr];

    uint32_t dac_reg = abs(_current_mA);
    dac_reg = (uint32_t) (dac_reg * 5083) >> 12;
    dac_reg = dac_reg & (0x00000FFF);
    phaseA.dacValue12Bits = (uint32_t) (dac_reg * abs(phaseA.sinMapData)) >> sin_pi_m2_dpiybit;
    phaseB.dacValue12Bits = (uint32_t) (dac_reg * abs(phaseB.sinMapData)) >> sin_pi_m2_dpiybit;

    tb67h450_set_two_coils_current(phaseA.dacValue12Bits, phaseB.dacValue12Bits);

    if (phaseA.sinMapData > 0)
        tb67h450_set_input_A(true, false);
    else if (phaseA.sinMapData < 0)
        tb67h450_set_input_A(false, true);
    else
        tb67h450_set_input_A(true, true);

    if (phaseB.sinMapData > 0)
        tb67h450_set_input_B(true, false);
    else if (phaseB.sinMapData < 0)
        tb67h450_set_input_B(false, true);
    else
        tb67h450_set_input_B(true, true);
}

void tb67h450_set_two_coils_current(uint16_t _currentA_3300mAIn12Bits, uint16_t _currentB_3300mAIn12Bits)
{
	 tb67h450_dac_output_voltage(_currentA_3300mAIn12Bits, _currentB_3300mAIn12Bits);
}

void tb67h450_set_input_A(bool _statusAp, bool _statusAm)
{
    _statusAp ? (IN_AP_GPIO_Port->BSRR = IN_AP_Pin) : (IN_AP_GPIO_Port->BRR = IN_AP_Pin);
    _statusAm ? (IN_AM_GPIO_Port->BSRR = IN_AM_Pin) : (IN_AM_GPIO_Port->BRR = IN_AM_Pin);
}

void tb67h450_set_input_B(bool _statusBp, bool _statusBm)
{
    _statusBp ? (IN_BP_GPIO_Port->BSRR = IN_BP_Pin) : (IN_BP_GPIO_Port->BRR = IN_BP_Pin);
    _statusBm ? (IN_BM_GPIO_Port->BSRR = IN_BM_Pin) : (IN_BM_GPIO_Port->BRR = IN_BM_Pin);
}



