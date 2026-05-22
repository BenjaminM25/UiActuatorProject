/*
 * Diagnostics.h
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */


/**
 * @file    Diagnostics.h
 * @brief   System diagnostics - tests each component in isolation.
 *
 * In main.c:
 *   #include "Diagnostics.h"
 *
 *   // In while(1) after USB delay:
 *   if (!diagDone && now > 3000) {
 *       diagDone = 1;
 *       Diagnostics_Run();
 *   }
 */


#ifndef INC_DIAGNOSTICS_H_
#define INC_DIAGNOSTICS_H_


#ifdef __cplusplus
extern "C" {
#endif

void Diagnostics_Run(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_DIAGNOSTICS_H_ */
