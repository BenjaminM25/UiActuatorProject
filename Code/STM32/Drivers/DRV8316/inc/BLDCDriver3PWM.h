/*
 * BLDCDriver3PWM.h
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */

#ifndef DRV8316_INC_BLDCDRIVER3PWM_H_
#define DRV8316_INC_BLDCDRIVER3PWM_H_


/**
 * @file    BLDCDriver3PWM.h
 * @brief   Hardware driver for DRV8316 in 3PWM mode.
 *
 * Owns:
 *  - DRV8316 SPI object (wake, sleep, enable, register config)
 *  - TIM1 PWM channels (INHx - high side)
 *  - INLx GPIO pins    (held LOW so DRV8316 controls low side)
 *
 * Usage:
 *   BLDCDriver3PWM driver(
 *       &hspi3,
 *       DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
 *       DRVOFF_GPIO_Port,      DRVOFF_Pin,
 *       nSLEEP_GPIO_Port,      nSLEEP_Pin,
 *       PWMAL_GPIO_Port,       PWMAL_Pin,    // INL_A
 *       PWMBL_GPIO_Port,       PWMBL_Pin,    // INL_B
 *       PWMCL_GPIO_Port,       PWMCL_Pin,    // INL_C
 *       &htim1
 *   );
 *   driver.init();
 */


#include "main.h"
#include "drv8316.h"
#include "drv8316_registers.h"
#include "BLDCDriver.h"

class BLDCDriver3PWM : public BLDCDriver {
public:
    BLDCDriver3PWM(
        SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* csPort,     uint16_t csPin,
        GPIO_TypeDef* drvoffPort, uint16_t drvoffPin,
        GPIO_TypeDef* sleepPort,  uint16_t sleepPin,
        GPIO_TypeDef* inlaPort,   uint16_t inlaPin,
        GPIO_TypeDef* inlbPort,   uint16_t inlbPin,
        GPIO_TypeDef* inlcPort,   uint16_t inlcPin,
        TIM_HandleTypeDef* htim
    );

    void init()  override;
    void enable() override;
    void disable() override;

    /* Convenience: set all three high-side duties at once. */
	void setPwm(float a, float b, float c);

    void setChannelDuty(uint8_t phase, float duty) override;
	void setLow       (uint8_t phase)              override;
	void phaseFloat   (uint8_t phase)              override;

	void setPhase(uint8_t phase, int state, float duty);

    bool isFault();
    void clearFault();

    /**
     * @brief  Read IC status register.
     */
    IC_Status getStatus();
    Status__1 getStatus1();
	Status__2 getStatus2();


private:
    DRV8316            _drv;
    TIM_HandleTypeDef* _htim;

    GPIO_TypeDef* _inlaPort; uint16_t _inlaPin;
    GPIO_TypeDef* _inlbPort; uint16_t _inlbPin;
    GPIO_TypeDef* _inlcPort; uint16_t _inlcPin;

    static const uint32_t CH[3];  /* TIM_CHANNEL_1/2/3 */

public:
    uint32_t period;   ///< TIM1 ARR — used by BLDCMotor for CCR scaling
};


#endif /* DRV8316_INC_BLDCDRIVER3PWM_H_ */
