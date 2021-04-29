/*
 * flash_page.c
 *
 *  Created on: Apr 27, 2021
 *      Author: ipizf
 */

#include "flash_page.h"


/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Apaga a quantidade de páginas especificada a partir do endereço de memória FLASH especificado.
  *
  * @OBS    As funções HAL_FLASH_Unlock() e HAL_FLASH_OB_Unlock() devem ser chamadas previamente para habilitar a interface FLASH
  *         As funções HAL_FLASH_Lock() e HAL_FLASH_OB_Lock() devem ser chamadas  posteriormente para desabilitar a interface FLASH
  *
  * @param  Endereço:	  Endereço inicial da página que deseja ser apagada
  * @param  Paginas:      Quantidade de páginas que deseja apagar a partir do endereço inicial.
  *
  * @retval Void
  */
void FLASH_apaga(uint32_t endereco, uint8_t paginas){
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Allow Access to option bytes sector */
	HAL_FLASH_OB_Unlock();

	/* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;			    // cria estrutura de dados para armazenar informações de apagamento
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;  // definição padrão da biblioteca HAL
	EraseInitStruct.PageAddress = endereco;				// definição do usuário do endereço da página que deseja apagar
	EraseInitStruct.NbPages = paginas;					// número de páginas que serão apagadas

	uint32_t PageError;									// Valor do endereço caso ocorra erro no apagamento da função HAL_FLASHEx_Erase
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
		// rotina caso ocorra erro no processo de apagar
	}

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}
/*-----------------------------------------------------------------------------------------------------------------------------------*/



/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Escreve uma half-word (16-bit) no endereço de memória flash específico
  *
  * @OBS1   As funções HAL_FLASH_Unlock() e HAL_FLASH_OB_Unlock() devem ser chamadas previamente para habilitar a interface FLASH
  *         As funções HAL_FLASH_Lock() e HAL_FLASH_OB_Lock() devem ser chamadas  posteriormente para desabilitar a interface FLASH
  *
  * @OBS2   O endereço de memória FLASH deve ser apagado antes da inserção de um novo valor. Neste caso, a página deve ser apagada.
  *
  * @param  Endereço:	  Endereço da memória em que o dado de 16 bits deverá ser armazenado. Endereço deve ser múltiplo de 2)
  * @param  *dado:        Ponteiro para o valor a ser inserido no endereço de memória especificado
  *
  * @retval Void
  */
void FLASH_escreve_16bits(uint32_t endereco, uint16_t *dado){

	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	if( HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,(uint64_t)endereco, *dado) != HAL_OK){
		// rotina caso ocorra erro no processo de apagar
	}

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}
/*-----------------------------------------------------------------------------------------------------------------------------------*/



/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Programa uma word (32-bit) no endereço de memória flash específico
  *
  * @OBS1   As funções HAL_FLASH_Unlock() e HAL_FLASH_OB_Unlock() devem ser chamadas previamente para habilitar a interface FLASH
  *         As funções HAL_FLASH_Lock() e HAL_FLASH_OB_Lock() devem ser chamadas  posteriormente para desabilitar a interface FLASH
  *
  * @OBS2   O endereço de memória FLASH deve ser apagado antes da inserção de um novo valor. Neste caso, a página deve ser apagada.
  *
  * @param  Endereço:	  Endereço da memória em que o dado de 32 bits deverá ser armazenado. Endereço deve ser múltiplo de 2
  * @param  *dado:        Ponteiro para o valor a ser inserido no endereço de memória especificado
  *
  * @retval Void
  */
void FLASH_escreve_32bits(uint32_t endereco, uint32_t *dado){

	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	if( HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint64_t)endereco, *dado) != HAL_OK){
		// rotina caso ocorra erro no processo de apagar
	}

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}
/*-----------------------------------------------------------------------------------------------------------------------------------*/


/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Programa uma string a partir do endereço de memória flash específico
  *
  * @OBS1   A função escreve valores de 16bits. Caso a string tenha número ímpar, a própria função preenche com 0xFF.
  *
  * @OBS2 	A própria função calcula o comprimento de memória que será utilizado
  *
  * @OBS3   O endereço de memória FLASH deve ser apagado antes da inserção de um novo valor. Neste caso, a página deve ser apagada.
  *			Caso a string ocupe mais de uma página, o projetista deverá interpretar isso e apagar a qtd. de páginas necessárias
  *
  * @param  Endereço:	  Endereço da memória em que a string[0] deverá ser armazenado. Endereço deve ser múltiplo de 2
  * @param  *dado:        Ponteiro para a string[] a ser inserido a partir endereço de memória especificado
  *
  * @retval Void
  */
void FLASH_escreve_string(uint32_t endereco, int8_t *dado){

	uint8_t contagem_de_endereco = 0;
	uint8_t comprimento_dado_FLASH = 0;
	uint8_t i = 0;

	// Se comprimento de *dado for par  : data_to_flash[a] , *dado[b] => a = b
	// Se comprimento dw *dado for ímpar: data_to_flash[a] , *dado[b] => a = b + 1
	comprimento_dado_FLASH = (strlen((char*)dado)) + (int)((strlen((char*)dado) % 2) != 0);
	uint8_t data_to_FLASH[comprimento_dado_FLASH];

	// Escreve o valor 0xFF (valor que consiste em endereço não utilizado na FLASH) em todos os elementos do vetor
	memset((uint8_t*)data_to_FLASH, 0xFF, comprimento_dado_FLASH);

	// Copia dado para data_to_flash (mantem último caractere com 0xFF caso seja ímpar)
	memcpy(data_to_FLASH,dado,strlen((char*)dado));

	for(i = 0; i < comprimento_dado_FLASH; i+=2){
		FLASH_escreve_16bits(endereco + contagem_de_endereco, (uint16_t*)&data_to_FLASH[i]);
		contagem_de_endereco += 2;
	}

}
/*-----------------------------------------------------------------------------------------------------------------------------------*/




/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Realiza a leitura de uma half-word (16-bit) no endereço de memória flash específico
  *
  * @param  Endereço:	  Endereço da memória em que o dado de 16 bits deverá ser lido. Endereço deve ser múltiplo de 2
  * @param  *dado:        Ponteiro para armazenar o valor a ser lido no endereço de memória especificado
  *
  * @retval Void
  */
void FLASH_le_16bits(uint32_t endereco, uint16_t *dado){

	*dado = *(uint32_t*)(endereco);

}
/*-----------------------------------------------------------------------------------------------------------------------------------*/




/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Realiza a leitura de uma word (32-bit) no endereço de memória flash específico
  *
  * @param  Endereço:	  Endereço da memória em que o dado de 32 bits deverá ser lido. Endereço deve ser múltiplo de 2
  * @param  *dado:        Ponteiro para armazenar o valor a ser lido no endereço de memória especificado
  *
  * @retval Void
  */
void FLASH_le_32bits(uint32_t endereco, uint32_t *dado){

	*dado = *(uint32_t*)(endereco);

}
/*-----------------------------------------------------------------------------------------------------------------------------------*/


/**-----------------------------------------------------------------------------------------------------------------------------------
  * @resumo Realiza a leitura de uma string[] a partir do endereço de memória flash indicado
  *
  * @param  Endereço:	  Endereço da memória em que a string[0] inicia. Endereço deve ser múltiplo de 2
  * @param  *dado:        Ponteiro (string[]) para armazenar o valor a ser lido no endereço de memória especificado
  *
  * @retval Void
  */
void FLASH_le_string_0xFF(uint32_t endereco, int8_t *dado){
	uint32_t leitura_FLASH = 0;
	uint8_t cont_end = 0;

	while(leitura_FLASH != 0xFFFFFFFF){

		leitura_FLASH = *(uint32_t*)(endereco + cont_end);

		if(leitura_FLASH != 0xFFFFFFFF){
			dado[0 + cont_end] = (int8_t)leitura_FLASH;
			dado[1 + cont_end] = (int8_t)(leitura_FLASH >> 8);
			dado[2 + cont_end] = (int8_t)(leitura_FLASH >> 16);
			dado[3 + cont_end] = (int8_t)(leitura_FLASH >> 24);
			cont_end +=4;
		}
	}

}
/*-----------------------------------------------------------------------------------------------------------------------------------*/

