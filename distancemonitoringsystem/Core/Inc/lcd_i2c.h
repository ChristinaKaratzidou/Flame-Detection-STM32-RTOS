/*
 * lcd_i2c.h
 *
 *  Created on: Mar 2, 2025
 *      Author: Neko
 */

// lcd_i2c.h - I2C LCD Driver Header for STM32F401RE

#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f4xx_hal.h"

#define LCD_BACKLIGHT   0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_EN          0x04
#define LCD_RW          0x02
#define LCD_RS          0x01

void LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t cols, uint8_t rows);
void LCD_Clear();
void LCD_SetCursor(uint8_t col, uint8_t row);
void LCD_Print(const char *str);
void LCD_Backlight(uint8_t state);

#endif // LCD_I2C_H
