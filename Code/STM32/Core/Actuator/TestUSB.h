/*
 * TestUSB.h
 *
 *  Created on: 15. apr. 2026
 *      Author: Benjamin
 */

#ifndef INC_TESTUSB_H_
#define INC_TESTUSB_H_


#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

void TESTUSB_Receive(uint8_t *buf, uint32_t len);  /* called from CDC_Receive_FS */
void TESTUSB_Update(void);


#endif /* INC_TESTUSB_H_ */
