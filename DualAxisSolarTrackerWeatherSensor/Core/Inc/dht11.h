/*
 * dht11.h
 *
 *  Created on: Dec 13, 2024
 *      Author: Neko
 */

#ifndef DHT11_H
#define DHT11_H

#include "main.h"

#define DHT11_OK        0
#define DHT11_ERROR     1

uint8_t DHT11_Read(float* temperature, float* humidity);

#endif
