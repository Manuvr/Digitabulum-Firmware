#include "stm32f7xx_hal.h"

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
     PA12   ------> USB_OTG_FS_DP
     PA11   ------> USB_OTG_FS_DM
     PA10   ------> USB_OTG_FS_ID
*/
void unused_gpio() {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOI_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();

  // LED Driver Reset pin
  GPIO_InitStruct.Pin   = GPIO_PIN_10;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // LED Driver Interrupt pin
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // The USB driver takes care of this for us.
  ///*Configure GPIO pins : PA12 PA11 PA10 */
  //GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  //GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* These Port C pins are push-pull outputs:
  *
  * #  Default   Purpose
  * -----------------------------------------------
  * 6     1      Expansion reset pin
  */
  GPIO_InitStruct.Pin   = GPIO_PIN_6;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);



  /* Everything below represents a pin that is unused. */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin  = GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_12
                          |GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8
                          |GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4
                          |GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PD7 PD0 PD5 PD1
                           PD4 PD15 PD14 PD12
                           PD11 PD9 PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_1
                          |GPIO_PIN_4|GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_12
                          |GPIO_PIN_11|GPIO_PIN_9|GPIO_PIN_8;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PI7 PI6 PI5 PI3
                           PI2 PI8 PI9 PI4
                           PI1 PI10 PI11 PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_3
                          |GPIO_PIN_2|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_4
                          |GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_0;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PH15 PH13 PH14 PH2
                           PH3 PH4 PH5 PH12
                           PH11 PH10 PH6 PH8
                           PH9 PH7 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12
                          |GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_7;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE13 PE8 PE9 PE7 PE10 PE12 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}
