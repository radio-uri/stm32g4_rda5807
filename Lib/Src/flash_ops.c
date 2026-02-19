#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <stdint.h>
#include <math.h>
#include "flash_ops.h"


/*
*   Flash write sequence:
*   1. Unlock flash
*   2. Erase flash blocks data to be stored in
*   3. Write data in to a block(s)
*   4. Lock the flash to prevent accodental write
*/

uint32_t Flash_Write_Data (uint32_t PageAddress, uint64_t *Data)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;
    uint32_t Error_msg = 0;

    /* 1. Unlock the Flash */
    HAL_FLASH_Unlock();

    /* EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; // Pages or Sections
    EraseInitStruct.Page = 127;  // Page number as per datasheet as far as possible from potential program location
    EraseInitStruct.NbPages = 1; // Number of pages to erase beginning from specified page nr.

    /* 2. Erase the flash */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        Error_msg = HAL_FLASH_GetError ();
    }

    /* 3. Program the Flash */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PageAddress, *Data) != HAL_OK) {
        Error_msg = HAL_FLASH_GetError ();
    }
    /* 4. Lock the Flash */
    HAL_FLASH_Lock();

    return Error_msg;
}

float_t Get_stored_float(uint64_t address){

  uint64_t val = *(__IO uint64_t*)address;
  return *(float_t*)&val;
}