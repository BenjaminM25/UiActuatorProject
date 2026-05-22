/*
 * TestHALL.cpp
 *
 *  Created on: 17. apr. 2026
 *      Author: Benjamin
 */


#include "testhall.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS() */
#include <stdio.h>
#include <string.h>
#include "plot.h"

/*
 * Using the internal TIM2 for us timer
 */

extern TIM_HandleTypeDef htim2;

/* ---------------------------------------------------------------
 * With 1 pole pair there are 6 Hall transitions per revolution
 * ---------------------------------------------------------------*/
#define HALL_TRANSITIONS_PER_REV    6

/* Valid Hall states and their sequence for forward rotation.
 * One electrical cycle: 1->3->2->6->4->5->1...
 * Reverse is the mirror: 5->4->6->2->3->1->5...
 * Encoded as U<<2 | V<<1 | W<<0                          */
static const uint8_t HALL_FORWARD_SEQ[6] = {1, 3, 2, 6, 4, 5};

static uint32_t s_lastTransitionTick = 0U;
static uint32_t s_lastTransitionTick_ms = 0U;
static uint32_t s_lastPrintTick      = 0U;
static int8_t   s_direction          = 0;   /* +1 forward, -1 reverse, 0 unknown */
static float    s_rpm                = 0.0f;
static uint8_t  s_lastHallState      = 0U;

/* ---------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------*/

static uint8_t Hall_Encode(HallState_t *state)
{
    return (uint8_t)((state->u << 2) | (state->v << 1) | state->w);
}

static int8_t Hall_GetDirection(uint8_t prev, uint8_t curr)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        if (HALL_FORWARD_SEQ[i] == prev)
        {
            uint8_t next_fwd = HALL_FORWARD_SEQ[(i + 1) % 6];
            uint8_t next_rev = HALL_FORWARD_SEQ[(i + 5) % 6];
            if (curr == next_fwd) return  1;
            if (curr == next_rev) return -1;
        }
    }
    return 0; /* invalid transition */
}

/* ---------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------*/

void Hall_Read(HallState_t *state)
{
    state->u = (HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    state->v = (HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    state->w = (HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void Hall_PrintStatus(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_lastPrintTick) < HALL_PRINT_INTERVAL_MS)
    {
        return;
    }
    s_lastPrintTick = now;

    /* If no transition for >500 ms assume stopped */
    if ((now - s_lastTransitionTick_ms) > 500U)
    {
        s_rpm = 0.0f;
    }

    Plot_Float("rpm", s_direction >=0 ? s_rpm : -s_rpm);
}

void Hall_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != HALL_U_Pin &&
        GPIO_Pin != HALL_V_Pin &&
        GPIO_Pin != HALL_W_Pin)
    {
        return;
    }

    HallState_t state;
    Hall_Read(&state);
    uint8_t encoded = Hall_Encode(&state);

    /* Plot the encoded state (1-6) so you can see the sequence */
    Plot_Float("hall_state", (float)encoded);
}
