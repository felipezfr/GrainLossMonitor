/*
 * flash_page.h
 *
 *  Created on: Apr 27, 2021
 *      Author: ipizf
 */

#ifndef INC_FLASH_PAGE_H_
#define INC_FLASH_PAGE_H_

#include "stm32f1xx_hal.h"


uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t * DATA_32);
void Flash_Read_Data (uint32_t StartPageAddress, __IO uint32_t * DATA_32);
void Convert_To_Str (uint32_t *data, char *str);

#endif /* INC_FLASH_PAGE_H_ */
