/*
 * flash_page.h
 *
 *  Created on: Apr 27, 2021
 *      Author: ipizf
 */

#ifndef INC_FLASH_PAGE_H_
#define INC_FLASH_PAGE_H_

#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"

// apaga uint8_t paginas a partir do uint32_t endereco
void FLASH_apaga(uint32_t endereco, uint8_t paginas);

// escreve uint16_t *dado no uint32_t endereco
void FLASH_escreve_16bits(uint32_t endereco, uint16_t *dado);

// escreve uint13_t *dado no uint32_t endereco
void FLASH_escreve_32bits(uint32_t endereco, uint32_t *dado);

// escreve int8_t *dado (string[]) a partir do uint32_t endereco
void FLASH_escreve_string(uint32_t endereco, int8_t *dado);

// le uint16_t *dado no uint32_t endereco
void FLASH_le_16bits(uint32_t endereco, uint16_t *dado);

// le uint32_t *dado no uint32_t endereco
void FLASH_le_32bits(uint32_t endereco, uint32_t *dado);

// escreve int8_t *dado (string[]) a partir do uint32_t endereco até que valor armazenado na memória seja 0xFFFF
void FLASH_le_string_0xFF(uint32_t endereco, int8_t *dado);

#endif /* INC_FLASH_PAGE_H_ */
