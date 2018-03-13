/*
 * ssd1306.h
 *
 *  Created on: 1 janv. 2018
 *      Author: hoel
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fonts.h"
#ifndef ssd1306
#define ssd1306
#define SSD1306_I2C             hi2c1
#define SSD1306_I2C_ADDR        (0x3C<< 1)
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
extern I2C_HandleTypeDef        hi2c1;

typedef enum {
  Black = 0x00, /*!< Black color, no pixel */
  White = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR;

typedef struct {
  uint16_t CurrentX;
  uint16_t CurrentY;
  uint8_t Inverted;
  uint8_t Initialized;
} SSD1306_t;

extern I2C_HandleTypeDef SSD1306_I2C_PORT;

uint8_t ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_wstr(char* str,uint8_t x, uint8_t y);
//static void ssd1306_WriteCommand(uint8_t command);

#endif
#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H_ */
