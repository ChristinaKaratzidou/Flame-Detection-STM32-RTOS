/*
 * i2c_lcd.c
 *
 *  Created on: Dec 10, 2024
 *      Author: Neko
 */

#include "i2c_lcd.h"

extern I2C_HandleTypeDef hi2c1;  // I2C handle from main.c

/* PCF8574 pins */
#define LCD_RS  (1 << 0)
#define LCD_RW  (1 << 1)
#define LCD_EN  (1 << 2)
#define LCD_BL  (1 << 3)
#define LCD_D4  (1 << 4)
#define LCD_D5  (1 << 5)
#define LCD_D6  (1 << 6)
#define LCD_D7  (1 << 7)

static void LCD_WriteCommand(uint8_t cmd);
static void LCD_WriteData(uint8_t data);
static void LCD_Write(uint8_t data, uint8_t rs);
static void LCD_PulseEnable(uint8_t data);

void LCD_Init(void)
{
    HAL_Delay(50);  // Wait for LCD to power up

    // Initialize 4-bit mode
    LCD_WriteCommand(0x33);
    HAL_Delay(5);
    LCD_WriteCommand(0x32);
    HAL_Delay(5);

    // Function Set
    LCD_WriteCommand(LCD_FUNCTIONSET | 0x08);  // 4-bit mode, 2 lines, 5x8 font

    // Display Control
    LCD_WriteCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);

    // Clear Display
    LCD_Clear();

    // Entry Mode Set
    LCD_WriteCommand(LCD_ENTRYMODESET | 0x02);  // Increment cursor, no display shift
}

void LCD_Clear(void)
{
    LCD_WriteCommand(LCD_CLEARDISPLAY);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_WriteCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_Print(const char *str)
{
    while(*str) LCD_WriteData(*str++);
}

void LCD_PrintChar(char c)
{
    LCD_WriteData(c);
}

static void LCD_WriteCommand(uint8_t cmd)
{
    LCD_Write(cmd, 0);
}

static void LCD_WriteData(uint8_t data)
{
    LCD_Write(data, 1);
}

static void LCD_Write(uint8_t data, uint8_t rs)
{
    uint8_t high_nibble = (data & 0xF0) | LCD_BL;
    uint8_t low_nibble = ((data << 4) & 0xF0) | LCD_BL;

    if (rs) {
        high_nibble |= LCD_RS;
        low_nibble |= LCD_RS;
    }

    LCD_PulseEnable(high_nibble);
    LCD_PulseEnable(low_nibble);
}

static void LCD_PulseEnable(uint8_t data)
{
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
    HAL_Delay(1);
    data |= LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
    HAL_Delay(1);
    data &= ~LCD_EN;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
    HAL_Delay(1);
}



