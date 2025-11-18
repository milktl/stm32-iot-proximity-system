/*
 * lcd.h
 *
 *  Created on: Jun 9, 2025
 *      Author: alexa
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

// Pin assignments
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_Port GPIOB

#define LCD_E_Pin GPIO_PIN_1
#define LCD_E_Port GPIOB

#define LCD_D4_Pin GPIO_PIN_2
#define LCD_D4_Port GPIOB

#define LCD_D5_Pin GPIO_PIN_3
#define LCD_D5_Port GPIOB

#define LCD_D6_Pin GPIO_PIN_4
#define LCD_D6_Port GPIOB

#define LCD_D7_Pin GPIO_PIN_5
#define LCD_D7_Port GPIOB

void LCD_Init(void);
void LCD_SendCommand(uint8_t);
void LCD_SendData(uint8_t);
void LCD_SendString(char*);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_EnablePulse(void);
void LCD_Send4Bits(uint8_t);
void LCD_Clear(void);


#endif /* INC_LCD_H_ */
