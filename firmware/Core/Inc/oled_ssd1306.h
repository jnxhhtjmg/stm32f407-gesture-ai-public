#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "stm32f4xx_hal.h"

/* Returns 1 when an SSD1306 answers at 0x3C or 0x3D. */
uint8_t OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_DrawString(uint8_t x, uint8_t page, const char *text);
void OLED_ShowGesture(const char *name, uint8_t confidence);
void OLED_Update(void);

#endif /* OLED_SSD1306_H */
