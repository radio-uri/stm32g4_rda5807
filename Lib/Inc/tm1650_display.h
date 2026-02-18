#ifndef TM1650_DISPLAY
#define TM1650_DISPLAY

/* Export function prototypes*/

void display_clear(uint8_t section);
void display_clear_all(void);
void display_digit(uint8_t section, uint8_t digit);
void display_number(float_t num);
void display_i2s_cfg(I2C_HandleTypeDef *ptr);
void set_display(uint8_t brightness, _Bool onoff);

#endif