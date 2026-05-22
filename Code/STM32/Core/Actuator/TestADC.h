/*
 * TestADC.h
 *
 *  Created on: 20. apr. 2026
 *      Author: Benjamin
 */

#ifndef TESTADC_H
#define TESTADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Raw ADC values — can be read externally if needed */
extern volatile uint16_t adc_raw[3];

void TestADC_Init(void);
void TestADC_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTADC_H */
