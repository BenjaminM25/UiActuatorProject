/*
 * SPITest.h
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */


/**
 * @file    SPITest.h
 * @brief   Standalone SPI communication test for the DRV8316.
 *
 * Tests SPI by reading back known registers and checking values.
 * Results are sent via Teleplot over USB CDC.
 *
 * Usage in main.c:
 *   #include "SPITest.h"
 *   // USER CODE BEGIN 2:
 *   SPITest_Run();
 *   // Do NOT call MotorApp_Init() at the same time
 */

#ifndef SPITEST_H
#define SPITEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Wake DRV8316, write known register values, read them back,
 *         and plot results via Teleplot. Call once from USER CODE BEGIN 2.
 */
void SPITest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* SPITEST_H */
