/*
 * TempTest.h
 *
 *  Created on: 14. mai 2026
 *      Author: Benjamin
 */

#ifndef INC_TEMPTEST_H_
#define INC_TEMPTEST_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEMP_SAMPLE_INTERVAL_MS		1000U
#define TEMP_PLOT_INTERVAL_MS		1000U

void TEMPTEST_Init(void);
void TEMPTEST_Run(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_TEMPTEST_H_ */
