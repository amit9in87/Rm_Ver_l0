/**
  ******************************************************************************
  * @file    nrf.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NRF_H
#define __NRF_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "nRF24l01.h"

	 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

	 
/* USER CODE BEGIN Includes */
/* Chip Select macro definition */

#define SPI_CS_LOW       HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)
#define SPI_CS_HIGH      HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)

/* Chip Enable macro definition */
#define SPI_CE_LOW       HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
#define SPI_CE_HIGH      HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)

/* USER CODE END Includes */	 




/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void nrfinit(SPI_HandleTypeDef hspi1, uint8_t ch, uint8_t mode);

void nrfwritereg(SPI_HandleTypeDef hspi1, uint8_t regname, uint8_t value);
uint8_t nrfreadreg(SPI_HandleTypeDef hspi1, uint8_t regname);

void nrfrxaddr(SPI_HandleTypeDef hspi1, uint8_t pipe, uint8_t payload, uint32_t addr);
void nrftxaddr(SPI_HandleTypeDef hspi1, uint32_t addr); // Always TX_ADDR pipe

void nrftransmit(SPI_HandleTypeDef hspi1, uint8_t buff[32], uint8_t len);
void nrfreceive(SPI_HandleTypeDef hspi1, uint8_t buff[32], uint8_t len);

void nrflisten(SPI_HandleTypeDef hspi1);
uint8_t nrftranschk(SPI_HandleTypeDef hspi1);


void nrfregstatus(SPI_HandleTypeDef hspi1, uint8_t buff[32]);

void nrfrx_powerup(SPI_HandleTypeDef hspi1);
void nrftx_powerup(SPI_HandleTypeDef hspi1);
void nrfpowerdown(SPI_HandleTypeDef hspi1);

/* USER CODE END PFP */




#ifdef __cplusplus
}
#endif

#endif /* __NRF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
