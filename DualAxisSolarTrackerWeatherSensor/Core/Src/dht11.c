/*
 * dht11.c
 *
 *  Created on: Dec 13, 2024
 *      Author: Neko
 */

#include "dht11.h"

/* Define the GPIO port and pin for DHT11 */
#define DHT11_PORT     GPIOB
#define DHT11_PIN      GPIO_PIN_0

/* Private function prototypes */
static void DHT11_DelayUs(uint32_t us);
static void DHT11_SetPinOutput(void);
static void DHT11_SetPinInput(void);
static uint8_t DHT11_ReadPin(void);

uint8_t DHT11_Read(float* temperature, float* humidity)
{
    uint8_t data[5] = {0}; // Array to store the received data
    uint8_t checksum = 0;  // For checksum verification

    /* Start communication */
    DHT11_SetPinOutput();      // Set pin as output
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);  // Pull low
    HAL_Delay(18);             // Wait 18ms (>18ms trigger signal)
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);    // Pull high
    DHT11_DelayUs(30);         // Wait 30us

    /* Set pin as input to read data */
    DHT11_SetPinInput();

    /* Check DHT11 response */
    if (DHT11_ReadPin() == 0)  // DHT11 pulls low for 80us
    {
        DHT11_DelayUs(80);
        if (DHT11_ReadPin() == 1)  // DHT11 pulls high for 80us
        {
            DHT11_DelayUs(80);

            /* Read 40 bits (5 bytes) of data */
            for (uint8_t i = 0; i < 5; i++)
            {
                for (uint8_t j = 0; j < 8; j++)
                {
                    /* Wait for data bit start */
                    while (DHT11_ReadPin() == 0);  // 50us low
                    DHT11_DelayUs(30);             // Wait 30us

                    /* Read bit value */
                    if (DHT11_ReadPin() == 1)      // If still high after 30us
                    {
                        data[i] |= (1 << (7 - j)); // It's a '1' bit
                        while (DHT11_ReadPin() == 1); // Wait for bit end
                    }
                }
            }

            /* Verify checksum */
            checksum = data[0] + data[1] + data[2] + data[3];
            if (checksum == data[4])
            {
                /* Data is valid, convert to temperature and humidity */
                *humidity = (float)data[0];
                *temperature = (float)data[2];
                return DHT11_OK;
            }
        }
    }

    return DHT11_ERROR;
}

/* Microsecond delay function */
static void DHT11_DelayUs(uint32_t us)
{
    uint32_t delay = us * (SystemCoreClock / 1000000U);
    while (delay--)
    {
        __NOP();
    }
}

/* Set DHT11 pin as output */
static void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* Set DHT11 pin as input */
static void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* Read DHT11 pin state */
static uint8_t DHT11_ReadPin(void)
{
    return HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN);
}


