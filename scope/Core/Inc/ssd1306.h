#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "main.h"

#define SSD1306_I2C_ADDR 0x78 // 0x3C << 1

void ssd1306_init(I2C_HandleTypeDef *hi2c);
void ssd1306_send_cmd(uint8_t cmd);
void ssd1306_send_data(uint8_t *data, uint16_t size);
void ssd1306_set_cursor(uint8_t x, uint8_t page);
void ssd1306_clear(void);
void ssd1306_draw_char(uint8_t x, uint8_t page, char c);

#endif
