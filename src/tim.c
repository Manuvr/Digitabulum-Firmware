/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "stm32f7xx_hal.h"

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  HAL_TIM_OC_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM2) {
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

    /**TIM2 GPIO Configuration
    PA15     ------> TIM2_CH1
    PB10     ------> TIM2_CH3
    PB11     ------> TIM2_CH4
    */
    // GPIO for haptic vibrators...
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance==TIM2) {
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA15     ------> TIM2_CH1
    PB10     ------> TIM2_CH3
    PB11     ------> TIM2_CH4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
  }
}
