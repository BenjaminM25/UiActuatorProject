/*
 * LimitSwitch.h
 *
 *  Created on: 21. apr. 2026
 *      Author: Benjamin
 */

#ifndef SRC_LIMITSWITCH_H_
#define SRC_LIMITSWITCH_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>


class LimitSwitch {
public:
    LimitSwitch(GPIO_TypeDef* port, uint16_t pin)
        : _port(port), _pin(pin) {}

    bool isPressed() const {
        return HAL_GPIO_ReadPin(_port, _pin) == GPIO_PIN_RESET;
    }

private:
    GPIO_TypeDef* _port;
    uint16_t      _pin;
};

#endif /* SRC_LIMITSWITCH_H_ */
