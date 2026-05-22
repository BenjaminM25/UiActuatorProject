/*
 * TestHALL.h
 *
 *  Created on: 17. apr. 2026
 *      Author: Benjamin
 */

#ifndef INC_TESTHALL_H_
#define INC_TESTHALL_H_

#include "main.h"
#include <stdint.h>

/* How often the RPM is printed (milliseconds) */
#define HALL_PRINT_INTERVAL_MS  200U

typedef struct {
    uint8_t u;
    uint8_t v;
    uint8_t w;
} HallState_t;

/**
 * @brief  Read all three Hall sensor outputs into a HallState_t struct.
 */
void Hall_Read(HallState_t *state);

/**
 * @brief  Print RPM and direction over USB CDC.
 *         Call periodically from the main loop.
 *         Prints 0.0 RPM if no transition seen for >500 ms.
 */
void Hall_PrintStatus(void);

/**
 * @brief  EXTI callback — updates RPM and direction on every Hall transition.
 *         Call from HAL_GPIO_EXTI_Callback() in stm32g4xx_it.c.
 */
void Hall_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_TESTHALL_H_ */
