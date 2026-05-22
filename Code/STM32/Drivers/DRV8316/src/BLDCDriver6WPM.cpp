/*
 * BLDCDriver6WPM.cpp
 *
 *  Created on: 8. mai 2026
 *      Author: Benjamin
 */


/**
 * @file    BLDCDriver6PWM.cpp
 */

#include "BLDCDriver6PWM.h"
#include "plot.h"

const uint32_t BLDCDriver6PWM::CH[3] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3
};

BLDCDriver6PWM::BLDCDriver6PWM(
    SPI_HandleTypeDef* hspi,
    GPIO_TypeDef* csPort,     uint16_t csPin,
    GPIO_TypeDef* drvoffPort, uint16_t drvoffPin,
    GPIO_TypeDef* sleepPort,  uint16_t sleepPin,
    TIM_HandleTypeDef* htim
)
    : _drv(hspi, csPort, csPin, drvoffPort, drvoffPin, sleepPort, sleepPin),
      _htim(htim),
      _pairActive{false, false, false},
      period(0)
{}

void BLDCDriver6PWM::init()
{
    /* Configure DRV8316 for 6x PWM mode with synchronous rectification.
     *
     * In 6PWM the timer drives both INHx and INLx as a complementary
     * pair with hardware dead-time, so EN_ASR is safe and beneficial:
     * during commutation the low-side FET conducts in place of the body
     * diode, reducing freewheel losses. (This is the conflict that
     * required EN_ASR=disabled in the 3PWM driver — there the manual
     * INLx GPIO control fought the DRV's automatic rectification.) */
    DRV8316Config cfg;
    cfg.pwmMode = PWM6_Mode;
    _drv.init(cfg);

    /* Cache period */
    period = _htim->Init.Period;

    /* Start TIM1 main and complementary PWM at 0% duty.
     * With CCR=0 the high side is held low all period and the
     * complement (INLx) is held high → low-side FET fully conducting.
     * That is a safe brake state for an unenergised motor. */
    for (int i = 0; i < 3; i++) {
        __HAL_TIM_SET_COMPARE(_htim, CH[i], 0);
        HAL_TIM_PWM_Start(_htim, CH[i]);
        HAL_TIMEx_PWMN_Start(_htim, CH[i]);
        _pairActive[i] = true;
    }

    /* CH4 drives ADC trigger via TRGO2 (OC4REF). */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4);
}

void BLDCDriver6PWM::enable()
{
    _drv.enable();
}

void BLDCDriver6PWM::disable()
{
    _drv.disable();
}

void BLDCDriver6PWM::setPwm(float a, float b, float c)
{
    setChannelDuty(0, a);
    setChannelDuty(1, b);
    setChannelDuty(2, c);
}

void BLDCDriver6PWM::setPhase(uint8_t phase, int state, float duty)
{
    if (phase > 2) return;

    switch (state)
    {
        case 1: // HIGH (PWM)
            setChannelDuty(phase, duty);
            break;

        case 0: // LOW (low-side FET on)
            setLow(phase);
            break;

        case -1: // FLOAT (Hi-Z)
            phaseFloat(phase);
            break;
    }
}

void BLDCDriver6PWM::setLow(uint8_t phase)
{
    if (phase > 2) return;
    /* Same enable as setChannelDuty, with CCR=0. */
    switch (phase) {
        case 0: TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE); break;
        case 1: TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE); break;
        case 2: TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE); break;
    }
    __HAL_TIM_SET_COMPARE(_htim, CH[phase], 0);
}

void BLDCDriver6PWM::phaseFloat(uint8_t phase)
{
    if (phase > 2) return;
    /* Clear CC_E and CC_NE for this channel. Pin output is disconnected
     * from the timer and falls to its idle state (RESET → low). With
     * both pins low: INH=0, INL=0 → DRV truth table → Hi-Z. */
    switch (phase) {
        case 0: TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE); break;
        case 1: TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE); break;
        case 2: TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE); break;
    }
}

void BLDCDriver6PWM::enablePair(uint8_t phase)
{
    if (phase > 2) return;
    if (_pairActive[phase]) return;

    /* Make sure the duty starts at 0 — the caller will set the real
     * duty immediately after this returns. */
    __HAL_TIM_SET_COMPARE(_htim, CH[phase], 0);

    HAL_TIM_PWM_Start  (_htim, CH[phase]);
    HAL_TIMEx_PWMN_Start(_htim, CH[phase]);

    _pairActive[phase] = true;
}

bool BLDCDriver6PWM::isFault()
{
    return HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin) == GPIO_PIN_RESET;
}

void BLDCDriver6PWM::clearFault()
{
    _drv.clearFault();
}

IC_Status BLDCDriver6PWM::getStatus()
{
    return _drv.getStatus0();
}

Status__1 BLDCDriver6PWM::getStatus1()
{
    return _drv.getStatus1();
}

Status__2 BLDCDriver6PWM::getStatus2()
{
    return _drv.getStatus2();
}

void BLDCDriver6PWM::setChannelDuty(uint8_t phase, float duty)
{
    if (phase > 2) return;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    /* Re-enable channel outputs in case they were disabled by phaseFloat. */
    switch (phase) {
        case 0: TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE); break;
        case 1: TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE); break;
        case 2: TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE); break;
    }
    uint32_t ccr = (uint32_t)(duty * period);
    __HAL_TIM_SET_COMPARE(_htim, CH[phase], ccr);
}

