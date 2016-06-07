/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"
#include "cmsis_os.h"

extern volatile uint32_t millis_since_reset;


/* External variables --------------------------------------------------------*/
extern void xPortSysTickHandler(void);
extern volatile void _hack_sadvance();

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void) {
  millis_since_reset++;
  _hack_sadvance();
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/*
*
*/
void WWDG_IRQHandler(void) {
}


/*
*
*/
void RTC_WKUP_IRQHandler(void) {
}


/*
*
*/
void RTC_Alarm_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream0_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream1_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream2_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream4_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream5_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream6_IRQHandler(void) {
}


/*
*
*/
void DMA1_Stream7_IRQHandler(void) {
}


/*
*
*/
void DMA2_Stream0_IRQHandler(void) {
}


/*
*
*/
void DMA2_Stream1_IRQHandler(void) {
}


/*
*
*/
void DMA2_Stream4_IRQHandler(void) {
}


/*
*
*/
void DMA2_Stream5_IRQHandler(void) {
}


/*
*
*/
void DMA2_Stream6_IRQHandler(void) {
}


/*
*
*/
void DMA2_Stream7_IRQHandler(void) {
}


/*
*
*/
void USART2_IRQHandler(void) {
}


/*
*
*/
void SDMMC1_IRQHandler(void) {
}


/*
*
*/
void RNG_IRQHandler(void) {
}
