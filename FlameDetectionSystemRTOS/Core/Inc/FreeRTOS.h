/*
 * FreeRTOS.h
 *
 *  Created on: Mar 13, 2025
 *      Author: Neko
 */

#ifndef INC_FREERTOS_H_
#define INC_FREERTOS_H_

#pragma once

#define configUSE_PREEMPTION            1
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configCPU_CLOCK_HZ              SystemCoreClock
#define configTICK_RATE_HZ              1000            // 1ms tick
#define configMAX_PRIORITIES            5               // Adjust based on needs
#define configMINIMAL_STACK_SIZE        128             // Words (not bytes)
#define configTOTAL_HEAP_SIZE           (20 * 1024)     // 20KB for STM32F401RE
#define configUSE_QUEUE_SETS            0
#define configCHECK_FOR_STACK_OVERFLOW  2               // Enable stack overflow checks
#include "stm32f4xx_hal.h"



#endif /* INC_FREERTOS_H_ */
