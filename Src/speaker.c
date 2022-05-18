/*
 * speaker.c
 *
 *  Created on: 8 juil. 2020
 *      Author: flassabe
 */

#include <speaker.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_tim.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;

void setHWBUZZER(unsigned short usFreq, unsigned short usDuration) {
	TIM_OC_InitTypeDef sConfigOC;
	TIM_MasterConfigTypeDef sMasterConfig;

	// usFreq not over 20000Hz
	if (usFreq > 20000) {
		usFreq = 20000;
	}
	// usFreq not below 20Hz
	if (usFreq < 20) {
		usFreq = 20;
	}

	// configure la fr�quence
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 240;		// 48MHZ/240 = 200KHz
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 200000/usFreq;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(&htim1);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = htim1.Init.Period/2;	//50% duty cycle
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

	// configure le timer pour la dur�e
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 48000;	// 1ms
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = usDuration;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim6);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

	/* Start PWM */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	/* Start the timer */
	__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim6);
}
