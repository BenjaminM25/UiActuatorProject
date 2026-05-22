/*
 * BLDCDriver3PWM.cpp
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */


/**
 * @file    BLDCDriver3PWM.cpp
 */

#include "BLDCDriver3PWM.h"
#include "plot.h"

const uint32_t BLDCDriver3PWM::CH[3] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
};

BLDCDriver3PWM::BLDCDriver3PWM(
    SPI_HandleTypeDef* hspi,
    GPIO_TypeDef* csPort,     uint16_t csPin,
    GPIO_TypeDef* drvoffPort, uint16_t drvoffPin,
    GPIO_TypeDef* sleepPort,  uint16_t sleepPin,
    GPIO_TypeDef* inlaPort,   uint16_t inlaPin,
    GPIO_TypeDef* inlbPort,   uint16_t inlbPin,
    GPIO_TypeDef* inlcPort,   uint16_t inlcPin,
    TIM_HandleTypeDef* htim
)
    : _drv(hspi, csPort, csPin, drvoffPort, drvoffPin, sleepPort, sleepPin),
      _htim(htim),
      _inlaPort(inlaPort), _inlaPin(inlaPin),
      _inlbPort(inlbPort), _inlbPin(inlbPin),
      _inlcPort(inlcPort), _inlcPin(inlcPin),
      period(0)
{}

void BLDCDriver3PWM::init()
{
    /* Full DRV8316 init - wake, configure, enable */
    _drv.init();

    /* INLx must be HIGH in 3PWM mode for DRV8316 to respond to INHx */
    HAL_GPIO_WritePin(_inlaPort, _inlaPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_inlbPort, _inlbPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_inlcPort, _inlcPin, GPIO_PIN_SET);

    /* Cache period */
    period = _htim->Init.Period;

    /* Start TIM1 PWM at 0% */
    for (int i = 0; i < 3; i++) {
        __HAL_TIM_SET_COMPARE(_htim, CH[i], 0);
        HAL_TIM_PWM_Start(_htim, CH[i]);
    }

    /* Start channel 4 to trigger ADC sampling in the valley */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4);
}

void BLDCDriver3PWM::enable()
{
    _drv.enable();
}

void BLDCDriver3PWM::disable()
{
    _drv.disable();
}

void BLDCDriver3PWM::setPwm(float a, float b, float c)
{
    setChannelDuty(0, a);
    setChannelDuty(1, b);
    setChannelDuty(2, c);
}

void BLDCDriver3PWM::setPhase(uint8_t phase, int state, float duty)
{
    if (phase > 2) return;

    switch (state)
    {
        case 1: // HIGH
            setChannelDuty(phase, duty);
            break;

        case 0: // LOW
            setLow(phase);
            break;

        case -1: // FLOAT
            phaseFloat(phase);
            break;
    }
}

void BLDCDriver3PWM::phaseFloat(uint8_t phase)
{
	if (phase > 2) return;
	    __HAL_TIM_SET_COMPARE(_htim, CH[phase], 0);

	    /* Pull INLx low to Hi-Z the phase */
	    switch (phase) {
	        case 0: HAL_GPIO_WritePin(_inlaPort, _inlaPin, GPIO_PIN_RESET); break;
	        case 1: HAL_GPIO_WritePin(_inlbPort, _inlbPin, GPIO_PIN_RESET); break;
	        case 2: HAL_GPIO_WritePin(_inlcPort, _inlcPin, GPIO_PIN_RESET); break;
	    }
}


bool BLDCDriver3PWM::isFault()
{
    return HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin) == GPIO_PIN_RESET;
}

void BLDCDriver3PWM::clearFault()
{
    _drv.clearFault();
}

IC_Status BLDCDriver3PWM::getStatus()
{
    return _drv.getStatus0();
}

Status__1 BLDCDriver3PWM::getStatus1()
{
    return _drv.getStatus1();
}

Status__2 BLDCDriver3PWM::getStatus2()
{
    return _drv.getStatus2();
}

void BLDCDriver3PWM::setChannelDuty(uint8_t phase, float duty)
{
    if (phase > 2) return;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    switch (phase) {
        case 0: HAL_GPIO_WritePin(_inlaPort, _inlaPin, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(_inlbPort, _inlbPin, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(_inlcPort, _inlcPin, GPIO_PIN_SET); break;
    }

    uint32_t ccr = (uint32_t)(duty * period);
    __HAL_TIM_SET_COMPARE(_htim, CH[phase], ccr);
}

void BLDCDriver3PWM::setLow(uint8_t phase)
{
    if (phase > 2) return;

    /* In 3PWM mode INLx must remain HIGH for the DRV8316 to control
     * the bridge. With INHx duty = 0 the high-side is off all period;
     * the DRV's body diode (or auto-rectification if enabled) handles
     * the freewheel path. */
    switch (phase) {
        case 0: HAL_GPIO_WritePin(_inlaPort, _inlaPin, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(_inlbPort, _inlbPin, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(_inlcPort, _inlcPin, GPIO_PIN_SET); break;
    }
    __HAL_TIM_SET_COMPARE(_htim, CH[phase], 0);
}

