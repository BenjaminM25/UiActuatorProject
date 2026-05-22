/*
 * MotorApp.h
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */

/**
 * @file    MotorApp.h
 * @brief   Top-level motor application. Call MotorApp_Init() once after
 *          all MX peripherals are initialised, then call MotorApp_Run()
 *          from the main while(1) loop.
 */

#ifndef INC_MOTORAPP_H_
#define INC_MOTORAPP_H_

#include "main.h"
#include <stdio.h>          /* for snprintf */
#include <stdbool.h>
#include "usbd_cdc_if.h"    /* for CDC_Transmit_FS */


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise DRV8316, TIM1 PWM, ADC DMA and set initial drive voltage.
 *         Call once after all MX_xxx_Init() and ADC calibration in main.c.
 */
void MotorApp_Init(void);

void test6pwm_static(void);

/**
 * @brief  Run one iteration: commutate motor, check faults, plot telemetry,
 *         service homing state machine, dispatch USB commands.
 *         Call from while(1) in main.c.
 */
void MotorApp_Run(void);

void MotorApp_Start(void);
void MotorApp_SimpleRun(void);

int8_t MotorApp_GetActiveLowPhase(void);

void MotorApp_HallCallback(uint16_t GPIO_Pin);

/* ---------------- Homing ---------------- */

/** Begin a homing cycle. Idempotent — safe to call repeatedly. */
void MotorApp_StartHoming(void);

/** True once the homing state machine has reached Done. */
bool MotorApp_HomingDone(void);

/** True while a homing cycle is in progress. */
bool MotorApp_HomingActive(void);

void SinglePhase_SimpleRun(void);

/* ---------------- Position commands ---------------- */
void MotorApp_GoTo(float targetRev);
void MotorApp_StartAutoCycle(void);

/* ---------------- USB command dispatch ---------------- */

/**
 * @brief  Pass a chunk of USB CDC RX data into the command parser.
 *         Safe to call from ISR context: only copies into a small buffer
 *         and sets a flag. The actual command is dispatched in
 *         MotorApp_Run().
 */
void MotorApp_USB_Receive(const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif


#endif /* INC_MOTORAPP_H_ */
