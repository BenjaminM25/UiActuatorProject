/*
 * TestCAN.h
 *
 *  Created on: 17. apr. 2026
 *      Author: Benjamin
 */

#ifndef INC_TESTCAN_H_
#define INC_TESTCAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <string.h>
#include <stdio.h>

/* Call once before the main loop to start the peripheral */
void TESTCAN_Init(void);

/* Call from the main loop to periodically transmit a frame */
void TESTCAN_Update(void);

/* Call every main loop iteration to flush pending RX prints */
void TESTCAN_Flush(void);

void TESTCAN_RxCallback(FDCAN_HandleTypeDef *hfdcan);

#ifdef __cplusplus
}
#endif


#endif /* INC_TESTCAN_H_ */
