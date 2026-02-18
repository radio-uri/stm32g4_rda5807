#include <stdint.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"
#include "tm1650_display.h"

I2C_HandleTypeDef *i2c_ptr;

void display_i2s_cfg(I2C_HandleTypeDef *ptr){
    i2c_ptr = ptr;
}

void display_clear(uint8_t section){
    uint8_t address[]= {0x68,0x6A,0x6C,0x6E};
    uint8_t data = 0x00;
    HAL_I2C_Master_Transmit(i2c_ptr, address[section], &data, 1, 500);

}

void display_clear_all(){
  for (uint8_t i=0;i<4;i++){
    display_clear(i);
  }
}

void display_digit(uint8_t section, uint8_t digit){
  if (section > 3) return;
  static uint8_t symbol[]={0x3f,0x06,0x5B,0x4F, 0x66, 0x6D, 0x7D,0x07, 0x7F, 0x6F};
  static uint8_t address[]= {0x68,0x6A,0x6C,0x6E};
  
  HAL_I2C_Master_Transmit(i2c_ptr, address[section], &symbol[digit], 1, 500);
}

void display_number(float_t num){
  int base = (int)(num*10);
  uint8_t digit[4];
  digit[3]=(int)(base % 10);
  digit[2]=(int)((base/10)%10);
  digit[1]=(int)((base/100)%10);
  digit[0]=(int)((base/1000)%10);
  for (uint8_t i=0;i<4;i++){
    if (digit[i]==0 && i==0) display_clear(i);
    else display_digit(i, digit[i]);
  }

}

/*
*   Sets display brightness and switches ON/OFF
*/
void set_display(uint8_t brightness, _Bool onoff){
  uint8_t data = brightness<<4|onoff;
  HAL_I2C_Master_Transmit(i2c_ptr, 0x48, &data, 1, 500);

}