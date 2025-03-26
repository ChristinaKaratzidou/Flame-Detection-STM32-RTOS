/*
 * lcd_2ic.c
 *
 *  Created on: Mar 2, 2025
 *      Author: Neko
 */

#include "lcd_i2c.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;  // Use global I2C handle (usually from main.c)

static uint8_t lcdAddress;
static uint8_t lcdBacklight = LCD_BACKLIGHT;

// Send command to LCD
void LCD_SendCommand(uint8_t cmd) {
    uint8_t data_u = (cmd & 0xF0);
    uint8_t data_l = ((cmd << 4) & 0xF0);
    uint8_t data_t[4] = {data_u | lcdBacklight | LCD_EN, data_u | lcdBacklight, data_l | lcdBacklight | LCD_EN, data_l | lcdBacklight};
    HAL_I2C_Master_Transmit(&hi2c1, lcdAddress, data_t, 4, HAL_MAX_DELAY);
}

// Send data to LCD
void LCD_SendData(uint8_t data) {
    uint8_t data_u = (data & 0xF0);
    uint8_t data_l = ((data << 4) & 0xF0);
    uint8_t data_t[4] = {data_u | lcdBacklight | LCD_EN | LCD_RS, data_u | lcdBacklight | LCD_RS, data_l | lcdBacklight | LCD_EN | LCD_RS, data_l | lcdBacklight | LCD_RS};
    HAL_I2C_Master_Transmit(&hi2c1, lcdAddress, data_t, 4, HAL_MAX_DELAY);
}

void LCD_Init(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t cols, uint8_t rows) {
    lcdAddress = address << 1;  // Shift to match HAL I2C 7-bit addressing

    HAL_Delay(50);
    LCD_SendCommand(0x03);
    HAL_Delay(5);
    LCD_SendCommand(0x03);
    HAL_Delay(5);
    LCD_SendCommand(0x03);
    HAL_Delay(5);
    LCD_SendCommand(0x02);

    LCD_SendCommand(0x28);  // 4-bit, 2 lines, 5x8 dots
    LCD_SendCommand(0x08);  // Display off
    LCD_SendCommand(0x01);  // Clear display
    HAL_Delay(2);
    LCD_SendCommand(0x06);  // Entry mode
    LCD_SendCommand(0x0C);  // Display on, cursor off
}

void LCD_Clear() {
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row) {
    uint8_t rowOffsets[] = {0x00, 0x40};
    LCD_SendCommand(0x80 | (col + rowOffsets[row]));
}

void LCD_Print(const char *str) {
    while (*str) {
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}

void LCD_Backlight(uint8_t state) {
    lcdBacklight = state ? LCD_BACKLIGHT : LCD_NOBACKLIGHT;
    uint8_t buf = lcdBacklight;
    HAL_I2C_Master_Transmit(&hi2c1, lcdAddress, &buf, 1, HAL_MAX_DELAY);
}


