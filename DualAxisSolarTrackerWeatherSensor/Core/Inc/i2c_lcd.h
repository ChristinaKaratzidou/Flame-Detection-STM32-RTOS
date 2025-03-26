/*
 * i2c_lcd.h
 *
 *  Created on: Dec 10, 2024
 *      Author: Neko
 */

#ifndef I2C_LCD_H
#define I2C_LCD_H
// These lines prevent the header file from being included multiple times

#include "main.h"
// Includes the main header file which has basic STM32 definitions

/* LCD Commands - these are standard commands for HD44780 LCD */
#define LCD_CLEARDISPLAY    0x01    // Command to clear the LCD screen
#define LCD_RETURNHOME      0x02    // Command to return cursor to start
#define LCD_ENTRYMODESET    0x04    // Command to set entry mode
#define LCD_DISPLAYCONTROL  0x08    // Command to control display
#define LCD_FUNCTIONSET     0x20    // Command to set LCD function
#define LCD_SETCGRAMADDR    0x40    // Command to set custom character
#define LCD_SETDDRAMADDR    0x80    // Command to set cursor position

/* LCD Display Control - options for display settings */
#define LCD_DISPLAYON  0x04    // Turn display on
#define LCD_DISPLAYOFF 0x00    // Turn display off
#define LCD_CURSORON   0x02    // Show cursor
#define LCD_CURSOROFF  0x00    // Hide cursor
#define LCD_BLINKON    0x01    // Make cursor blink
#define LCD_BLINKOFF   0x00    // Stop cursor blinking

/* PCF8574 I2C Address */
#define LCD_I2C_ADDR 0x27 << 1    // I2C address of LCD module (shifted left by 1)

/* Function Prototypes - list of functions we'll create */
void LCD_Init(void);              // Initialize the LCD
void LCD_Clear(void);             // Clear the LCD screen
void LCD_SetCursor(uint8_t row, uint8_t col);    // Move cursor to position
void LCD_Print(const char *str);  // Print a string on LCD
void LCD_PrintChar(char c);       // Print a single character

#endif
