/*
 * BLDCDriver6PWM.h
 *
 *  Created on: 8. mai 2026
 *      Author: Benjamin
 */

#ifndef DRV8316_INC_BLDCDRIVER6PWM_H_
#define DRV8316_INC_BLDCDRIVER6PWM_H_

#include "main.h"
#include "drv8316.h"
#include "drv8316_registers.h"
#include "BLDCDriver.h"

class BLDCDriver6PWM : public BLDCDriver{
public:
    BLDCDriver6PWM(
        SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* csPort,     uint16_t csPin,
        GPIO_TypeDef* drvoffPort, uint16_t drvoffPin,
        GPIO_TypeDef* sleepPort,  uint16_t sleepPin,
        TIM_HandleTypeDef* htim
    );

    void init() override;
    void enable() override;
    void disable() override;

    void setPwm(float a, float b, float c);


    void setChannelDuty(uint8_t phase, float duty) override;
    void setLow(uint8_t phase);
    void phaseFloat(uint8_t phase) override;

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

    static const uint32_t CH[3];   /* TIM_CHANNEL_1/2/3 */

    /**
     * @brief  Re-enable the complementary pair for a phase that was
     *         previously floated. No-op if already active.
     */
    void enablePair(uint8_t phase);

    /* Tracks whether each pair is currently enabled (vs. floated). */
    bool _pairActive[3];

public:
    uint32_t period;   ///< TIM1 ARR — used by BLDCMotor for CCR scaling
};



#endif /* DRV8316_INC_BLDCDRIVER6PWM_H_ */
