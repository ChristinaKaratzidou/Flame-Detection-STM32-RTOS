/*
 * hcsr04.c
 *
 *  Created on: Mar 2, 2025
 *      Author: Neko
 */

#include "hcsr04.h"
#include "main.h"

void HCSR04_Trigger() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

float HCSR04_ReadDistance() {
    HCSR04_Trigger();
    uint32_t tickStart = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - tickStart > 50) return -1;  // Timeout
    }
    uint32_t startTime = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET);
    uint32_t endTime = DWT->CYCCNT;

    uint32_t pulseWidth = endTime - startTime;
    float distance = (pulseWidth / (SystemCoreClock / 1000000.0)) / 58.0;
    return distance;
}
