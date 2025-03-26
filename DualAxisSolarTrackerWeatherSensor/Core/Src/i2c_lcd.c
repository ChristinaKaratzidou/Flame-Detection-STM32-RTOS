/*
 * i2c_lcd.c
 *
 *  Created on: Dec 10, 2024
 *      Author: Neko
 */

#include "i2c_lcd.h"
// Include our header file

extern I2C_HandleTypeDef hi2c1;
// Use the I2C1 settings from main.c

/* Define pins on the PCF8574 I2C expander */
#define LCD_RS  (1 << 0)    // Register Select pin: 0=command, 1=data
#define LCD_RW  (1 << 1)    // Read/Write pin: 0=write, 1=read
#define LCD_EN  (1 << 2)    // Enable pin: pulse to process data
#define LCD_BL  (1 << 3)    // Backlight control
#define LCD_D4  (1 << 4)    // Data pin 4
#define LCD_D5  (1 << 5)    // Data pin 5
#define LCD_D6  (1 << 6)    // Data pin 6
#define LCD_D7  (1 << 7)    // Data pin 7

// Declare private functions (only used in this file)
static void LCD_WriteCommand(uint8_t cmd);
static void LCD_WriteData(uint8_t data);
static void LCD_Write(uint8_t data, uint8_t rs);
static void LCD_PulseEnable(uint8_t data);

void LCD_Init(void)
{
    HAL_Delay(50);  // Wait for LCD to power up

    // Initialize 4-bit mode (standard procedure for HD44780)
    LCD_WriteCommand(0x33);
    HAL_Delay(5);
    LCD_WriteCommand(0x32);
    HAL_Delay(5);

    // Set up the display
    LCD_WriteCommand(LCD_FUNCTIONSET | 0x08);  // 4-bit mode, 2 lines
    LCD_WriteCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON); // Display on
    LCD_Clear();    // Clear the display
    LCD_WriteCommand(LCD_ENTRYMODESET | 0x02); // Cursor moves right
}

void LCD_Clear(void)
{
    LCD_WriteCommand(LCD_CLEARDISPLAY);  // Send clear command
    HAL_Delay(2);  // Wait for LCD to process
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    // Calculate position in LCD memory
    uint8_t row_offsets[] = {0x00, 0x40};  // Start of each line
    LCD_WriteCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_Print(const char *str)
{
    // Print each character in the string
    while(*str) LCD_WriteData(*str++);
}

void LCD_PrintChar(char c)
{
    LCD_WriteData(c);  // Print a single character
}

static void LCD_Write(uint8_t data, uint8_t rs)
{
    // Split byte into two 4-bit parts (4-bit mode)
    uint8_t high_nibble = (data & 0xF0) | LCD_BL;  // Upper 4 bits
    uint8_t low_nibble = ((data << 4) & 0xF0) | LCD_BL;  // Lower 4 bits

    // Set RS pin based on whether it's data or command
    if (rs) {
        high_nibble |= LCD_RS;
        low_nibble |= LCD_RS;
    }

    // Send both nibbles
    LCD_PulseEnable(high_nibble);
    LCD_PulseEnable(low_nibble);
}

static void LCD_PulseEnable(uint8_t data)
{
    // Send data to LCD
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
    HAL_Delay(1);

    // Pulse the enable pin
    data |= LCD_EN;  // Set Enable high
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
    HAL_Delay(1);

    data &= ~LCD_EN;  // Set Enable low
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
    HAL_Delay(1);
}

static void LCD_WriteCommand(uint8_t cmd)
{
    LCD_Write(cmd, 0);  // Write command (rs=0)
}

static void LCD_WriteData(uint8_t data)
{
    LCD_Write(data, 1);  // Write data (rs=1)
}


