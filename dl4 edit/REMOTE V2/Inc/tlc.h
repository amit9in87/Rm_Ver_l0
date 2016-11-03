/**
  ******************************************************************************
  * @file    stm32l0xx_it.h
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
#ifndef __TLC_H
#define __TLC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "led_position.h"
#include "key_position.h"
#include "ser_data.h"	
#include "ser_mask.h"	
#include "stm32l0xx_it.h"	 

	 
/* Exported types ------------------------------------------------------------*/
	 
extern int kp_prev;
extern int kp_next;
extern int kp_valid;

extern int stage;
extern int kpvalue;
extern int serdata;
extern int sermask;
	 
extern int userdata;	
extern int guestdata;	 
	 
extern int guest1;
extern int guest2;
extern int guest3;
extern int guest4;
extern int guest5;
	  
	  
extern int64_t	ledreg; 
	 
	 
/* Exported constants --------------------------------------------------------*/
#define Buzzc   TIM2->CR1 &= ~TIM_CR1_CEN;
#define Buzzs   TIM2->CR1 |= TIM_CR1_CEN;

#define LATs 		HAL_GPIO_WritePin(USER_LATCH_GPIO_Port, USER_LATCH_Pin, GPIO_PIN_SET)
#define LATc 		HAL_GPIO_WritePin(USER_LATCH_GPIO_Port, USER_LATCH_Pin, GPIO_PIN_RESET)

#define SCLKs   HAL_GPIO_WritePin(LED_SCLK_GPIO_Port, LED_SCLK_Pin, GPIO_PIN_SET)
#define SCLKc   HAL_GPIO_WritePin(LED_SCLK_GPIO_Port, LED_SCLK_Pin, GPIO_PIN_RESET)

#define SINs    HAL_GPIO_WritePin(LED_SIN_GPIO_Port, LED_SIN_Pin, GPIO_PIN_SET)
#define SINc    HAL_GPIO_WritePin(LED_SIN_GPIO_Port, LED_SIN_Pin, GPIO_PIN_RESET)

#define BLANKs  HAL_GPIO_WritePin(LED_SBLANK_GPIO_Port, LED_SBLANK_Pin, GPIO_PIN_SET)
#define BLANKc  HAL_GPIO_WritePin(LED_SBLANK_GPIO_Port, LED_SBLANK_Pin, GPIO_PIN_RESET)

#define KP_ROW1s  HAL_GPIO_WritePin(KP_ROW1_GPIO_Port, KP_ROW1_Pin, GPIO_PIN_SET)
#define KP_ROW1c  HAL_GPIO_WritePin(KP_ROW1_GPIO_Port, KP_ROW1_Pin, GPIO_PIN_RESET)
#define KP_ROW1t  HAL_GPIO_TogglePin(KP_ROW1_GPIO_Port, KP_ROW1_Pin)

#define KP_ROW2s  HAL_GPIO_WritePin(KP_ROW2_GPIO_Port, KP_ROW2_Pin, GPIO_PIN_SET)
#define KP_ROW2c  HAL_GPIO_WritePin(KP_ROW2_GPIO_Port, KP_ROW2_Pin, GPIO_PIN_RESET)
#define KP_ROW2t  HAL_GPIO_TogglePin(KP_ROW2_GPIO_Port, KP_ROW2_Pin)

#define KP_ROW3s  HAL_GPIO_WritePin(KP_ROW3_GPIO_Port, KP_ROW3_Pin, GPIO_PIN_SET)
#define KP_ROW3c  HAL_GPIO_WritePin(KP_ROW3_GPIO_Port, KP_ROW3_Pin, GPIO_PIN_RESET)
#define KP_ROW3t  HAL_GPIO_TogglePin(KP_ROW3_GPIO_Port, KP_ROW3_Pin)

#define KP_ROW4s  HAL_GPIO_WritePin(KP_ROW4_GPIO_Port, KP_ROW4_Pin, GPIO_PIN_SET)
#define KP_ROW4c  HAL_GPIO_WritePin(KP_ROW4_GPIO_Port, KP_ROW4_Pin, GPIO_PIN_RESET)
#define KP_ROW4t  HAL_GPIO_TogglePin(KP_ROW4_GPIO_Port, KP_ROW4_Pin)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void ledcmd(uint32_t BCR, uint32_t BCG, uint32_t BCB, uint64_t WRTCMD);  // Set the TLC5954 in writing mode
void leddata(uint64_t Data);								// Write data in TLC5954
void ledbrightness(uint8_t brightness);     // Adjust the brightness values accepted from 5 to 95 
void ledblink(int ontime, int offtime);     // Blink the leds as per TLC5954 data 

void buzzer_init(void);                     // Initiallizes the timer 3 for buzzer source of 3.571 KHz
void buzzer_beep(int ontime, int offtime);  // Buzzer beeps for as per timing

uint64_t setxtraled(uint64_t led_reg, uint8_t led_name);  // Used for setting xtra led as per name
uint64_t clrxtraled(uint64_t led_reg, uint8_t led_name);  // Used for clearing xtra led as per name
uint64_t togglextraled(uint64_t led_reg, uint8_t led_name); // Used for toggle xtra led as per name
uint64_t attn(uint64_t led_reg);            // Used for grabbing attention by blinking side led and status led
// uint64_t kp_ledpos(uint8_t index);          // Direct Key press mapping to Led mapping to be used in special case

uint32_t showuser(uint32_t entry);          // Show user data only as per entry
uint32_t showguest(uint32_t entry);         // Show guest data only as per entry
uint32_t showage(uint32_t entry);						// Show age data only as per entry
void showgender(uint32_t entry);						// Toggle gender Led as it is not mapped in TLC5954


uint32_t scanrow(uint32_t kpvalue);					// Scan Keypad single rows 
uint32_t kp_scan(void);											// Scan Keypad all rows and return the Key press map

uint32_t kp_verify(uint32_t input);					// Verify the single Key press and return the value if not zero 
uint8_t  kp_identify(uint32_t input);       // Identify the key position from Key Press mapped value
uint8_t  kp_input(void);										// Used as one function to get the Key press input instead of using above 3 individually

void user_interface(void);									// Keypress and LED User Interface defined here. In short GUI of remote
uint32_t sd_entry(uint8_t pos);							// Write the entry in Serdata directly as per Key Press Position


void setdata(uint8_t bitname);							// Can be used to Set any data as per bit name
void clrdata(uint8_t bitname);              // Can be used to Clear any data as per bit name

void reg_guest(void);												// Run the sequence for registering Guest and writes the data in Server data 
void reg_viewer(void);											// Run the sequence for user and guest viewer and writes the data in Server data 
void reg_guestdetails(uint8_t frwd);				// Register the guest details such as gender & age and writes the data in Server data 



#ifdef __cplusplus
}
#endif

#endif /* __TLC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
