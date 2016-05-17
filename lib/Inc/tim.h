#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f7xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */
