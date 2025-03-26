/*
 * task.h
 *
 *  Created on: Mar 13, 2025
 *      Author: Neko
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_

#pragma once

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os2.h"

// Shared RTOS objects (extern for access across files)
extern QueueHandle_t xFlameQueue;
extern SemaphoreHandle_t xSensorMutex;

// Task prototypes
void vFlameSensorTask(void *pvParameters);
void vAlertTask(void *pvParameters);
void vSerialTask(void *pvParameters);

// Helper function prototypes
uint16_t readFlameSensor(void);



#endif /* INC_TASK_H_ */
