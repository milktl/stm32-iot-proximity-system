/*
 * lcd.c
 *
 *  Created on: Jun 9, 2025
 *      Author: alexa
 */

#include "lcd.h"
#include "stm32l5xx_hal.h"
#include "string.h"

void LCD_EnablePulse(void) {
    HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_Port, LCD_E_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Send4Bits(uint8_t data) {
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data >> 3) & 0x01);
    LCD_EnablePulse();
}

void LCD_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_Send4Bits(cmd >> 4);
    LCD_Send4Bits(cmd & 0x0F);
    HAL_Delay(2);
}

void LCD_SendData(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_Send4Bits(data >> 4);
    LCD_Send4Bits(data & 0x0F);
    HAL_Delay(2);
}

void LCD_Init(void) {
    HAL_Delay(50);  // Wait for LCD to power up properly
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    HAL_Delay(5);
    LCD_Send4Bits(0x02);
    HAL_Delay(1);

    LCD_SendCommand(0x28); // 4-bit, 2 line, 5x8 font
    LCD_SendCommand(0x0C); // Display ON, cursor OFF, blink OFF
    LCD_SendCommand(0x06); // Entry mode set: Increment cursor
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(2);
}

void LCD_SendString(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    LCD_SendCommand(0x80 | (address + col));
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01); // Clear display command
    HAL_Delay(2); // Wait for the command to execute
}
