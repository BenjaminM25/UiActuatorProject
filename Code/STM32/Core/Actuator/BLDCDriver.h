/*
 * BLDCDriver.h
 *
 *  Created on: 15. mai 2026
 *      Author: Benjamin
 */

#ifndef INC_BLDCDRIVER_H_
#define INC_BLDCDRIVER_H_



/*
 * BLDCDriver.h
 *
 *  Abstract interface for a 3-phase BLDC gate driver, so BLDCMotor
 *  can drive either the 3PWM or the 6PWM implementation.
 */


#include "drv8316_registers.h"   /* for IC_Status / Status__1 / Status__2 */

class BLDCDriver {
public:
    virtual ~BLDCDriver() = default;

    virtual void init()    = 0;
    virtual void enable()  = 0;
    virtual void disable() = 0;

    /* High side PWM with low side handled by the implementation
     * (3PWM: DRV automatic rectification; 6PWM: complementary CHxN). */
    virtual void setChannelDuty(uint8_t phase, float duty) = 0;

    /* Force phase LOW (low-side FET fully conducting). */
    virtual void setLow(uint8_t phase) = 0;

    /* Float a phase to Hi-Z. */
    virtual void phaseFloat(uint8_t phase) = 0;

    virtual bool isFault()    = 0;
    virtual void clearFault() = 0;

    virtual IC_Status getStatus()  = 0;
    virtual Status__1 getStatus1() = 0;
    virtual Status__2 getStatus2() = 0;
};



#endif /* INC_BLDCDRIVER_H_ */
