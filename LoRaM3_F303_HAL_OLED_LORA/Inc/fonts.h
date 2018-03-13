/*
 * fonts.h
 *
 *  Created on: 1 janv. 2018
 *      Author: hoel
 */

#ifndef FONTS_H_
#define FONTS_H_

#include "stm32f3xx_hal.h"
typedef struct {
  const uint8_t FontWidth;
  uint8_t FontHeight;
  const uint16_t *data;
} FontDef;


extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;


#endif /* FONTS_H_ */
