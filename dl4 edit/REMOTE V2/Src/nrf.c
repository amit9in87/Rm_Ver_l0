/**
  ******************************************************************************
  * @file    nrf.c
  * @brief   nRF24l01.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 Vayve Technology
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
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "nRF24l01.h"
#include "nrf.h"

/* USER CODE BEGIN Includes */

/* Chip Select macro definition */

#define SPI_CS_LOW       HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)
#define SPI_CS_HIGH      HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)

/* Chip Enable macro definition */
#define SPI_CE_LOW       HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
#define SPI_CE_HIGH      HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t nrf_CONFIG;

/* USER CODE END PV */


/* USER CODE BEGIN 4 */

void nrfinit(SPI_HandleTypeDef hspi1, uint8_t ch, uint8_t re_transmit)
{ 
	SPI_CE_LOW;
	nrf_CONFIG = 0x00;
	nrfwritereg(hspi1, CONFIG, (nrf_CONFIG | (0<<EN_CRC)| (0<<CRCO))); // 0 for 1 byte crc scheme
	nrfwritereg(hspi1, EN_RXADDR, 0x3f); // enabled rx_address all	
	nrfwritereg(hspi1, SETUP_AW, 0x03); // 1 for 3 byte addr, 2 for 4 & 3 for 5
	nrfwritereg(hspi1, SETUP_RETR, re_transmit); // ARD | ARC
	nrfwritereg(hspi1, RF_CH, (ch&0x7e)); // Max 7 bit 
	nrfwritereg(hspi1, RF_SETUP, 0x26);
	nrfwritereg(hspi1, FLUSH_TX, 0x00);
	nrfwritereg(hspi1, FLUSH_RX, 0x00);
}
void nrfwritereg(SPI_HandleTypeDef hspi1, uint8_t regname, uint8_t value)
{
	regname = (0x20|regname);
	SPI_CS_LOW;
  HAL_SPI_Transmit(&hspi1, &regname, 1, 3000);
  HAL_SPI_Transmit(&hspi1, &value, 1, 3000);
  SPI_CS_HIGH;
}

uint8_t nrfreadreg(SPI_HandleTypeDef hspi1, uint8_t regname)
{
	uint8_t value;
	SPI_CS_LOW;
  HAL_SPI_Transmit(&hspi1, &regname, 1, 3000);
  HAL_SPI_Receive(&hspi1, &value, 1, 3000);
  SPI_CS_HIGH;
  return value;
}

void nrfregstatus(SPI_HandleTypeDef hspi1, uint8_t buff[32])
{
  for(int i=0; i<32; i++)
	 buff[i] = nrfreadreg(hspi1, i);
}	

void nrfrxaddr(SPI_HandleTypeDef hspi1, uint8_t pipe, uint8_t payload, uint32_t addr)
{
	uint8_t adr[4];
	switch(pipe)
	{
	  case 0: pipe = RX_ADDR_P0; 	nrfwritereg(hspi1, RX_PW_P0, payload); break; 
	  case 1: pipe = RX_ADDR_P1; 	nrfwritereg(hspi1, RX_PW_P1, payload); break; 
	  case 2: pipe = RX_ADDR_P2; 	nrfwritereg(hspi1, RX_PW_P2, payload); break; 
	  case 3: pipe = RX_ADDR_P3; 	nrfwritereg(hspi1, RX_PW_P3, payload); break; 
	  case 4: pipe = RX_ADDR_P4; 	nrfwritereg(hspi1, RX_PW_P4, payload); break; 
	  case 5: pipe = RX_ADDR_P5; 	nrfwritereg(hspi1, RX_PW_P5, payload); break; 
		
		default: break;
	}
	
	adr[0] = addr; adr[1] = addr>>8;
	adr[2] = addr>>16; adr[3] = addr>>24;
  pipe = (0x20|pipe);
	SPI_CS_LOW;
  HAL_SPI_Transmit(&hspi1, &pipe, 1, 3000);
	for(int j =0; j<4; j++)
   HAL_SPI_Transmit(&hspi1, &adr[j], 1, 3000);
  SPI_CS_HIGH;
}

void nrftxaddr(SPI_HandleTypeDef hspi1, uint32_t addr)
{
	uint8_t adr[4];
	uint8_t pipe; pipe = 0x10;
	adr[0] = addr; adr[1] = addr>>8;
	adr[2] = addr>>16; adr[3] = addr>>24;
  pipe = (0x20|pipe);
	SPI_CS_LOW;
  HAL_SPI_Transmit(&hspi1, &pipe, 1, 3000);
	for(int j =0; j<4; j++)
   HAL_SPI_Transmit(&hspi1, &adr[j], 1, 3000);
  SPI_CS_HIGH;
}
void nrftransmit(SPI_HandleTypeDef hspi1, uint8_t buff[32], uint8_t len)
{
  uint8_t cmd;
	cmd = W_TX_PAYLOAD;
	nrfwritereg(hspi1, FLUSH_TX, 0x00);
	nrfwritereg(hspi1, STATUS, 0x1e);
	SPI_CS_LOW;
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 3000);		
	for(int i=0; i<len; i++)
   HAL_SPI_Transmit(&hspi1, &buff[i], 1, 3000);
  SPI_CS_HIGH;
  SPI_CE_HIGH;
	HAL_Delay(10);
	nrfwritereg(hspi1, STATUS, 0x1e);
	nrfwritereg(hspi1, RF_CH, 22);

	HAL_Delay(5);
//  SPI_CE_LOW;

}

uint8_t nrftranschk(SPI_HandleTypeDef hspi1)
{
	nrfwritereg(hspi1, STATUS, 0x1e);
	if(0x11!=nrfreadreg(hspi1, FIFO_STATUS)) return 1; // repeat unless 0x11 is received
	else return 0;
}	

void nrflisten(SPI_HandleTypeDef hspi1)
{
	uint8_t status;
	SPI_CE_HIGH;
  while(0x11!=(nrfreadreg(hspi1, FIFO_STATUS)));
	SPI_CE_LOW;
}	

void nrfreceive(SPI_HandleTypeDef hspi1, uint8_t buff[32], uint8_t len)
{ 
  uint8_t cmd;
	cmd = R_RX_PAYLOAD;
	SPI_CS_LOW;
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 3000);		
	for(int i=0; i<len; i++)
   HAL_SPI_Receive(&hspi1, &buff[i], 1, 3000);
  SPI_CS_HIGH;
}	

void nrfrx_powerup(SPI_HandleTypeDef hspi1)
{
	nrf_CONFIG = nrfreadreg(hspi1, CONFIG);
	nrfwritereg(hspi1, CONFIG, nrf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
}

void nrftx_powerup(SPI_HandleTypeDef hspi1)
{
	nrf_CONFIG = nrfreadreg(hspi1, CONFIG);
	nrfwritereg(hspi1, CONFIG, nrf_CONFIG  | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}

void nrfpowerdown(SPI_HandleTypeDef hspi1)
{
 	nrf_CONFIG = nrfreadreg(hspi1, CONFIG);
	nrfwritereg(hspi1, CONFIG, nrf_CONFIG  |  (0<<PWR_UP)  );
}	

/* USER CODE END 4 */




/************************ (C) COPYRIGHT Vayve Technology *****END OF FILE****/
