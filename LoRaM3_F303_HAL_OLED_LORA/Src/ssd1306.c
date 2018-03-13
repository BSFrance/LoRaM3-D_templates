/*
 * ssd1306.c
 *
 *  Created on: 1 janv. 2018
 *      Author: hoel
 */

#include"ssd1306.h"
#define SSD1306_WRITECOMMAND  ssd1306_WriteCommand

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static SSD1306_t SSD1306;

static void ssd1306_WriteCommand(uint8_t command){
	HAL_I2C_Mem_Write(&hi2c1,SSD1306_I2C_ADDR,0x00,1,&command,1,10);
	//HAL_I2C_Mem_Write_DMA(&hi2c1,SSD1306_I2C_ADDR,0x00,1,&command,1);
}
void WriteDat(unsigned char I2C_Data) {

	HAL_I2C_Mem_Write(&hi2c1,SSD1306_I2C_ADDR,0x40,1,&I2C_Data,1,10);
	//HAL_I2C_Mem_Write_DMA(&hi2c1,SSD1306_I2C_ADDR,0x40,1,&I2C_Data,1);
}
uint8_t ssd1306_Init(void){
  // Even wachten zodat het scherm zeker opgestart is
//  HAL_Delay(100);
//
//  /* Init LCD */
//  ssd1306_WriteCommand(0xAE); //display off
//  ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode
//  ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
//  ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
//  ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
//  ssd1306_WriteCommand(0x00); //---set low column address
//  ssd1306_WriteCommand(0x10); //---set high column address
//  ssd1306_WriteCommand(0x40 | 0x0 ); //--set start line address
//  ssd1306_WriteCommand(0x81); //--set contrast control register
//  ssd1306_WriteCommand(0xFF);
//  ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
//  ssd1306_WriteCommand(0xA6); //--set normal display
//  ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
//  ssd1306_WriteCommand(0x3F); //
//  ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
//  ssd1306_WriteCommand(0xD3); //-set display offset
//  ssd1306_WriteCommand(0x00); //-not offset
//  ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
//  ssd1306_WriteCommand(0xF0); //--set divide ratio
//  ssd1306_WriteCommand(0xD9); //--set pre-charge period
//  ssd1306_WriteCommand(0x22); //
//  ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
//  ssd1306_WriteCommand(0x12);
//  ssd1306_WriteCommand(0xDB); //--set vcomh
//  ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
//  ssd1306_WriteCommand(0x8D); //--set DC-DC enable
//  ssd1306_WriteCommand(0x14); //
//  ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel
//
//  /* Clearen scherm */
//  ssd1306_Fill(Black);
//
//  /* Update screen */
//  ssd1306_UpdateScreen();
//
//  /* Set default values */
//  SSD1306.CurrentX = 0;
//  SSD1306.CurrentY = 0;
//
//  /* Initialized OK */
//  SSD1306.Initialized = 1;
//
//
//  ssd1306_I2C_Init();

  /* Check if LCD connected to I2C */
  if (HAL_I2C_IsDeviceReady(&SSD1306_I2C, SSD1306_I2C_ADDR, 5, 1000) != HAL_OK) {
    /* Return false */

    return 8;
  }

  /* A little delay */
  uint32_t p = 2500;
  while(p>0) p--;

  /* Init LCD */
  SSD1306_WRITECOMMAND(0xAE); //display off
  SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
  SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
  SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
  SSD1306_WRITECOMMAND(0x02); //---set low column address
  SSD1306_WRITECOMMAND(0x10); //---set high column address
  SSD1306_WRITECOMMAND(0x40); //--set start line address
  SSD1306_WRITECOMMAND(0x81); //--set contrast control register
  SSD1306_WRITECOMMAND(0xFF);
  SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
  SSD1306_WRITECOMMAND(0xA6); //--set normal display
  SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
  SSD1306_WRITECOMMAND(0x3F); //
  SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  SSD1306_WRITECOMMAND(0xD3); //-set display offset
  SSD1306_WRITECOMMAND(0x00); //-not offset
  SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
  SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
  SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
  SSD1306_WRITECOMMAND(0x22); //
  SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
  SSD1306_WRITECOMMAND(0x12);
  SSD1306_WRITECOMMAND(0xDB); //--set vcomh
  SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
  SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
  SSD1306_WRITECOMMAND(0x14); //
  SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel

  /* Clear screen */
  ssd1306_Fill(0x00);

  /* Update screen */
  ssd1306_UpdateScreen();

  /* Set default values */
  SSD1306.CurrentX = 0;
  SSD1306.CurrentY = 0;

  /* Initialized OK */
  SSD1306.Initialized = 1;


  /* Return OK */
  return 1;
}
void ssd1306_Fill(SSD1306_COLOR color){
  /* Set memory */
  uint32_t i;

  for(i = 0; i < sizeof(SSD1306_Buffer); i++)
  {
    SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
  }
}
void ssd1306_Fill2(unsigned char fill_Data) {

  unsigned char m, n;
  for (m = 0; m < 8; m++) {
    SSD1306_WRITECOMMAND(0xb0 + m);   //page0-page1
    SSD1306_WRITECOMMAND(0x02);   //low column start address
    SSD1306_WRITECOMMAND(0x10);   //high column start address
    for (n = 0; n < 128; n++) {
      WriteDat(fill_Data);
    }
  }
}
void ssd1306_UpdateScreen(void) {
  uint8_t i;

  for (i = 0; i < 8; i++) {
    ssd1306_WriteCommand(0xB0 + i);
    ssd1306_WriteCommand(0x02);
    ssd1306_WriteCommand(0x10);

    // We schrijven alles map per map weg
    HAL_I2C_Mem_Write(&SSD1306_I2C,SSD1306_I2C_ADDR,0x40,1,&SSD1306_Buffer[SSD1306_WIDTH * i],SSD1306_WIDTH,100);
  }
}
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color){
  if (x > SSD1306_WIDTH || y > SSD1306_HEIGHT)
  {
    // We gaan niet buiten het scherm schrijven
    return;
  }

  // Kijken of de pixel geinverteerd moet worden
  if (SSD1306.Inverted)
  {
    color = (SSD1306_COLOR)!color;
  }

  // We zetten de juiste kleur voor de pixel
  if (color == White)
  {
    SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
  }
  else
  {
    SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
  }
}
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color){
  uint32_t i, b, j;

  // Kijken of er nog plaats is op deze lijn
  if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
    SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
  {
    // Er is geen plaats meer
    return 0;
  }

  // We gaan door het font
  for (i = 0; i < Font.FontHeight; i++)
  {
    b = Font.data[(ch - 32) * Font.FontHeight + i];
    for (j = 0; j < Font.FontWidth; j++)
    {
      if ((b << j) & 0x8000)
      {
        ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
      }
      else
      {
        ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
      }
    }
  }

  // De huidige positie is nu verplaatst
  SSD1306.CurrentX += Font.FontWidth;

  // We geven het geschreven char terug voor validatie
  return ch;
}
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color){
  // We schrijven alle char tot een nulbyte
  while (*str)
  {
    if (ssd1306_WriteChar(*str, Font, color) != *str)
    {
      // Het karakter is niet juist weggeschreven
      return *str;
    }

    // Volgende char
    str++;
  }

  // Alles gelukt, we sturen dus 0 terug
  return *str;
}
void ssd1306_wstr(char* str,uint8_t x, uint8_t y){
  ssd1306_SetCursor(x, y);
  ssd1306_WriteString(str, Font_7x10, White);
  ssd1306_UpdateScreen();
}
void ssd1306_SetCursor(uint8_t x, uint8_t y) {
  /* Set write pointers */
  SSD1306.CurrentX = x;
  SSD1306.CurrentY = y;
}
