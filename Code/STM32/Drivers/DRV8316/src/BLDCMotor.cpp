/*
 * BLDCMotor.cpp
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */

#include "BLDCMotor.h"
#include "BLDCDriver.h"

/* =========================================================================
 * Lookup tables
 * ========================================================================= */

const int8_t BLDCMotor::HALL_TO_SECTOR[8] = {
    -1,  /* 000 invalid */
     2,  /* Hall=1 → sector 1 */
     0,  /* Hall=2 → sector 5 */
     1,  /* Hall=3 → sector 4 */
     4,  /* Hall=4 → sector 3 */
     3,  /* Hall=5 → sector 2 */
     5,  /* Hall=6 → sector 0 */
    -1
};


//const int8_t BLDCMotor::HALL_TO_SECTOR[8] = {
//    -1,  /* 000 invalid */
//     2,  /* Hall=1 */
//     0,  /* Hall=2 */
//     1,  /* Hall=3 */
//     3,  /* Hall=4 — was 4 */
//     4,  /* Hall=5 — was 3 */
//     5,  /* Hall=6 */
//    -1
//};

const BLDCMotor::TrapStep BLDCMotor::TRAP_TABLE[6] = {
    /* sector 0: A high, C low */ { 1, 0, 0,  0, 0, 1 },
    /* sector 1: B high, C low */ { 0, 1, 0,  0, 0, 1 },
    /* sector 2: B high, A low */ { 0, 1, 0,  1, 0, 0 },
    /* sector 3: C high, A low */ { 0, 0, 1,  1, 0, 0 },
    /* sector 4: C high, B low */ { 0, 0, 1,  0, 1, 0 },
    /* sector 5: A high, B low */ { 1, 0, 0,  0, 1, 0 }
};

/* =========================================================================
 * Constructor
 * ========================================================================= */

BLDCMotor::BLDCMotor(
    GPIO_TypeDef* hallUPort, uint16_t hallUPin,
    GPIO_TypeDef* hallVPort, uint16_t hallVPin,
    GPIO_TypeDef* hallWPort, uint16_t hallWPin)
    : _driver(nullptr),
      _hallUPort(hallUPort), _hallUPin(hallUPin),
      _hallVPort(hallVPort), _hallVPin(hallVPin),
      _hallWPort(hallWPort), _hallWPin(hallWPin),
      _vbus(12.0f),
      _duty(0.0f),
	  _activeLowPhase(-1)
{}

/* =========================================================================
 * Public API
 * ========================================================================= */

void BLDCMotor::linkDriver(BLDCDriver* driver)
{
    _driver = driver;
}

void BLDCMotor::init()
{
    /* Nothing to do beyond linking — Hall pins are already configured
     * by CubeMX as GPIO inputs with pull-ups. */
}

void BLDCMotor::setVoltage(float volts)
{
    if (volts < 0.0f)  volts = 0.0f;
    if (volts > _vbus) volts = _vbus;
    _duty = volts / _vbus;
}


static const int8_t FORWARD_OFFSET = 2;
static const int8_t REVERSE_OFFSET = 5;

void BLDCMotor::run(uint8_t hallState, int8_t direction)
{
    if (!_driver) return;

    uint8_t hall = hallState & 0x07;
    int8_t  sec  = HALL_TO_SECTOR[hall];

    if (sec < 0) {
        _driver->phaseFloat(0);
        _driver->phaseFloat(1);
        _driver->phaseFloat(2);
        return;
    }

    sec = (direction >= 0)
		? (sec + FORWARD_OFFSET) % 6
		: (sec + REVERSE_OFFSET) % 6;


    const TrapStep& step = TRAP_TABLE[(uint8_t)sec];


    if      (step.aH) _driver->setChannelDuty(0, _duty);
    else if (step.aL) _driver->setLow(0);
    else              _driver->phaseFloat(0);

    if      (step.bH) _driver->setChannelDuty(1, _duty);
    else if (step.bL) _driver->setLow(1);
    else              _driver->phaseFloat(1);

    if      (step.cH) _driver->setChannelDuty(2, _duty);
    else if (step.cL) _driver->setLow(2);
    else              _driver->phaseFloat(2);

    if		(step.aL) _activeLowPhase = 0;
    else if (step.bL) _activeLowPhase = 1;
    else if (step.cL) _activeLowPhase = 2;
    else              _activeLowPhase = -1;
}




