/*
 * TESTUSB.cpp
 *
 *  Created on: 15. apr. 2026
 *      Author: Benjamin
 */

#include "TESTUSB.h"

/*
 * TESTUSB.cpp
 *
 *  Created on: 15. apr. 2026
 *      Author: Benjamin
 */

#include "TESTUSB.h"

#define BLINK_INTERVAL_MS  500

static uint32_t last_blink = 0;

void TESTUSB_Update(void)
{
    uint8_t vbus = (HAL_GPIO_ReadPin(VBUS_GPIO_Port, VBUS_Pin) == GPIO_PIN_SET) ? 1 : 0;

    if (vbus)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    else
    {
        uint32_t now = HAL_GetTick();
        if (now - last_blink >= BLINK_INTERVAL_MS)
        {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            last_blink = now;
        }
    }
}

