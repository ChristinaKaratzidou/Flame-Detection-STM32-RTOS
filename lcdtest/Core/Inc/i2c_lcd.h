/*
 * i2c_lcd.h
 *
 *  Created on: Dec 10, 2024
 *      Author: Neko
 */

#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "main.h"

/* LCD Commands */
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

/* LCD Display Control */
#define LCD_DISPLAYON  0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON   0x02
#define LCD_CURSOROFF  0x00
#define LCD_BLINKON    0x01
#define LCD_BLINKOFF   0x00

/* PCF8574 I2C Address */
#define LCD_I2C_ADDR 0x27 << 1  // Adjust this if your LCD uses a different address

/* Function Prototypes */
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);
void LCD_PrintChar(char c);

#endif
