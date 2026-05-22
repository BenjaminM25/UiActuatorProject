/*
 * BLDCMotor.h
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */

#ifndef DRV8316_INC_BLDCMOTOR_H_
#define DRV8316_INC_BLDCMOTOR_H_

#include "stm32g4xx_hal.h"
#include "BLDCDriver3PWM.h"

class BLDCMotor {
public:
    BLDCMotor(
        GPIO_TypeDef* hallUPort, uint16_t hallUPin,
        GPIO_TypeDef* hallVPort, uint16_t hallVPin,
        GPIO_TypeDef* hallWPort, uint16_t hallWPin
    );

    /* Commutation table */
	struct TrapStep { int8_t aH, bH, cH, aL, bL, cL; };
	static const TrapStep TRAP_TABLE[6];

    /**
     * @brief  Link the hardware driver. Call before init().
     */
    void linkDriver(BLDCDriver* driver);

    /**
     * @brief  Initialise motor — reads initial Hall state.
     *         Call after driver.init().
     */
    void init();

    /**
     * @brief  Set drive voltage in volts. Clamped to [0, vbus].
     */
    void setVoltage(float volts);


    /**
     * @brief  Set DC bus voltage for normalisation. Default 5.0 V.
     */
    void setSupplyVoltage(float vbus) { _vbus = vbus; }

    /**
     * @brief  Execute one commutation step. Call from main loop.
     */
    void run(uint8_t hallState, int8_t direction);  /* use pre-read Hall state */

    /**
     * @brief  Raw Hall state 0–7 (0 and 7 are invalid fault states).
     */

    static int8_t hallToSector(uint8_t hall) { return HALL_TO_SECTOR[hall & 0x07]; }

    int8_t getActiveLowPhase() const { return _activeLowPhase; };

private:
    BLDCDriver* _driver;

    GPIO_TypeDef* _hallUPort; uint16_t _hallUPin;
    GPIO_TypeDef* _hallVPort; uint16_t _hallVPin;
    GPIO_TypeDef* _hallWPort; uint16_t _hallWPin;

    float _vbus;
    float _duty;
    int8_t _activeLowPhase;  // 0=A, 1=B, 2=C, -1=none

    /* Hall → sector lookup */
    static const int8_t HALL_TO_SECTOR[8];



};


#endif /* DRV8316_INC_BLDCMOTOR_H_ */
