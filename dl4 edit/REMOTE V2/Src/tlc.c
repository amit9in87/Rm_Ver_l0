/**
  ******************************************************************************
  * @file    tlc.c
  * @brief   tlc5954 SIPO register.
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
#include "led_position.h"
#include "key_position.h"
#include "ser_data.h"
#include "ser_mask.h"	

#include "tlc.h"


int kp_prev;
int kp_next;
int kp_valid;

int stage;
int kpvalue;
int serdata;
int sermask;

int guest1;
int guest2;
int guest3;
int guest4;
int guest5;

int64_t	ledreg; 
	 

/* USER CODE BEGIN 0 */
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

void buzzs(void)
{ TIM2->CCER |= TIM_CCER_CC1E|TIM_CCER_CC1P;                        // Select active high polarity on OC1 & enable the output on OC1 
  TIM2->CR1  |= TIM_CR1_CEN;
}

void buzzc(void)
{ TIM2->CCER &= ~(TIM_CCER_CC1E|TIM_CCER_CC1P);                        // Select active high polarity on OC1 & enable the output on OC1 
  TIM2->CR1  &= ~TIM_CR1_CEN;
}
void buzzer_init(void)
{
  TIM2->PSC |= 1;       // Prescalar 1+1 i.e. Timer clock freq is 500KHz, Xtal clock = 1 MHz 
  TIM2->ARR = 70-1;     // Auto reload is 70 i.e. Timer overrun freq is 7.142 KHz 
  TIM2->CCR1 = 35-1;    // Compare value is 35 and Output toggle on comparision Hence output freq is 3.571 KHz
  TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // Select toggle mode on OC1 (OC1M = 011)
  TIM2->CCER |= TIM_CCER_CC1E|TIM_CCER_CC1P;          // CCC1P select active low polarity on OC1 & CC1E enable the output on OC1 
}

void buzzer_beep(int ontime, int offtime)
{
  if(buzzer_lock==0)
  {     
	   Buzzs; 
	   ms3counts = 0;
		 buzzer_lock = 1;
	}
	if(ms3counts > ontime)
	{  Buzzc;
	}
	if(ms3counts > ontime+offtime)
	  buzzer_lock = 0;


}



void ledcmd(uint32_t BCR, uint32_t BCG, uint32_t BCB, uint64_t WRTCMD)
{
	uint64_t CMD = 0;
//	MAXBUR = MAXBUR;
	BCR = BCR<<3;                			// Brightness control for red lines in TLC5954
	BCG = BCG<<10;										// green lines
	BCB = BCB<<17;										// blue lines

	WRTCMD = WRTCMD<<40;							// Write sequece code for accessing register in TLC5954

  CMD = WRTCMD|BCB|BCG|BCR;					

  /*  For transmitting data in serial pattern MSB first */
	SINc;	SCLKc;	LATc;	BLANKs;				
	LATs;	LATc;											  // Latch the data transferred in Shift register of TLC5954 to respective registers
  SINs;	 SCLKs;	SCLKc;							// Used for generating the 49th bit in Sin pin of TLC5954
  for(int i=0; i<48; i++)           // Transmitting the rest 48 bit as per CMD value
	{
	  if(CMD&0x800000000000)
	  { SINs;  SCLKs;	SCLKc;
	  }
		else
		{ SINc;	 SCLKs;	SCLKc;
		}
    CMD = CMD<<1;							
	}
	LATs;	LATc;												// Latch the data transferred in Shift register of TLC5954 to respective registers 
																		// Necessary after data transferred completed
}

void leddata(uint64_t Data)
{
	/*  For transmitting data in serial pattern MSB first */
	SINc;	SCLKc; LATc;								// Do not toggle BLANK pin in TLC this generates flickering when used in sequence
	LATs;	LATc;												// Latch the data transferred in Shift register of TLC5954 to respective registers
  SINc;	 SCLKs;	SCLKc;							// Used for generating the 49th bit in Sin pin of TLC5954 
  for(int i=0; i<48; i++)						// Transmitting the rest 48 bit as per Data value
	{
	  if(Data&0x800000000000)
	  {  SINs;	SCLKs;	SCLKc;
	  }
		else
		{  SINc;	SCLKs;	SCLKc;
		}
    Data = Data<<1;
	}
	LATs;	LATc;												// Latch the data transferred in Shift register of TLC5954 to respective registers 
																		// Necessary after data transferred completed
}



void ledbrightness(uint8_t brightness)
{
  if(brightness<5) brightness = 5;										// Limits the brightness lower value
	if(brightness>95) brightness = 95;									// Limits the brightness highed value
  TIM22->CCR1 = 100-brightness-1;											// Store the set value in Timer compare register
}
void ledblink(int ontime, int offtime)
{ 
  if(led_lock==0)																			// To know the start event and initiallize the mscounts to 0	
  {     
	   TIM22->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;			// Set the Interrupt for generating PWM o/p 
	   TIM22->CR1 |= TIM_CR1_CEN;												// Enable and start the Timer 
	   ms1counts = 0;																		// Set the ms1counts to 0 for knowing the start time
		 led_lock = 1;																		// Set the lock to avoid repeatitive start
	}
	if(ms1counts > ontime)															// Compare with the Ontime mentioned for LED
	{																										// If exceeds the Ontime disable the timer
     TIM22->DIER &= ~(TIM_DIER_UIE | TIM_DIER_CC1IE);	// Disable the Interrupt before stop necessary 
   	 TIM22->CR1 &= ~TIM_CR1_CEN;											// Stop the Timer
		 BLANKs;																					// Turn off the LED if left ON 
	}
	if(ms1counts > ontime+offtime)											// Compare with Ontime + Offtime for the offtime delay
	 led_lock = 0;																			// Clear the Led Lock and free it for other operation
}


uint64_t setxtraled(uint64_t led_reg, uint8_t led_name)
{
	uint64_t temp;
	temp = 0x01;
  led_reg = (led_reg | (temp << led_name));						// Shift as per Bit name or will set the Bit
  return led_reg;  
}

uint64_t togglextraled(uint64_t led_reg, uint8_t led_name)
{
	uint64_t temp;
	temp = 0x01;
  led_reg = (led_reg ^ (temp << led_name));						// Shift as per Bit name or will set the Bit
  return led_reg;  
}

uint64_t clrxtraled(uint64_t led_reg, uint8_t led_name)
{
	uint64_t temp;
	temp = 0x01<<led_name;     // Shift as per Bit name
  temp = ~temp;              // Invert it
	led_reg = led_reg & temp;  // AND will clear the Bit
  return led_reg;  
}

uint64_t attn(uint64_t led_reg)
{
  led_reg = setxtraled(led_reg, LD_SAB1);   led_reg = setxtraled(led_reg, LD_SAB2);   led_reg = setxtraled(led_reg, LD_SAB3);  
	led_reg = setxtraled(led_reg, LD_SAB4);   led_reg = setxtraled(led_reg, LD_SAB5);   led_reg = setxtraled(led_reg, LD_SAB6);  
	led_reg = setxtraled(led_reg, LD_SAB7);   led_reg = setxtraled(led_reg, LD_SAB8);   led_reg = setxtraled(led_reg, LD_SAB9);  
	led_reg = setxtraled(led_reg, LD_SAB10);  led_reg = setxtraled(led_reg, LD_SAB11);  led_reg = setxtraled(led_reg, LD_SAB12);  
	led_reg = setxtraled(led_reg, LD_SAB13);  led_reg = setxtraled(led_reg, LD_SAB14);  led_reg = setxtraled(led_reg, LD_STATB);  
	return led_reg;
}

uint64_t kp_ledpos(uint8_t index)
{
	uint64_t led_reg=0;
	uint8_t pos=0;
	if(index!=0)																	
	{	
   switch(index)
	 {																							// Please refer TLC excel sheet for understanding Key press mapping to LED position
			case	13	: pos = 	LD_UB1	; break;			
			case	21	: pos = 	LD_UB2	; break;
			case	5	: pos = 	LD_UB3	; break;
			case	29	: pos = 	LD_UB4	; break;
			case	22	: pos = 	LD_UB5	; break;
			case	28	: pos = 	LD_UB6	; break;
			case	7	: pos = 	LD_UB7	; break;
			case	23	: pos = 	LD_UB8	; break;
			case	15	: pos = 	LD_UB9	; break;
			case	14	: pos = 	LD_UB10	; break;
			case	6	: pos = 	LD_UB11	; break;
			case	20	: pos = 	LD_UB12	; break;
			case	18	: pos = 	LD_GB1	; break;
			case	26	: pos = 	LD_GB2	; break;
			case	10	: pos = 	LD_GB3	; break;
			case	2	: pos = 	LD_GB4	; break;
			case	3	: pos = 	LD_GB5	; break;
			case	27	: pos = 	LD_VACB	; break;
			case	25	: pos = 	LD_AGB1	; break;
			case	17	: pos = 	LD_AGB2	; break;
			case	9	: pos = 	LD_AGB3	; break;
			case	1	: pos = 	LD_AGB4	; break;
			case	19	: pos = 	LD_AGB5	; break;
//			case	11	: pos = 	LD_MFB	; break;			// Male Female LED not mapped in TLC5954
			case	12	: pos = 	LD_GRB	; break;
			case	4	: pos = 	LD_NFB1	; break;

		  default:  break;
	 }

   if(index==4) led_reg = 0x0f;										// For mapping ON OFF keypress to four ON OFF Led
   else 	led_reg = 1;														// For mapping rest keypress to their respective Led 

 
	 for(int i=0; i<pos; i++)												// Shift the map as per position with respect to keypress
    led_reg = led_reg<<1; 												
  }
	 
	 
  return led_reg;																
}


// Keypad Functions
uint32_t scanrow(uint32_t kp_value)
{
	if(HAL_GPIO_ReadPin(KP_COL1_GPIO_Port, KP_COL1_Pin))
		kp_value = kp_value|(1<<0);
	if(HAL_GPIO_ReadPin(KP_COL2_GPIO_Port, KP_COL2_Pin))
		kp_value = kp_value|(1<<1);
	if(HAL_GPIO_ReadPin(KP_COL3_GPIO_Port, KP_COL3_Pin))
		kp_value = kp_value|(1<<2);
	if(HAL_GPIO_ReadPin(KP_COL4_GPIO_Port, KP_COL4_Pin))
		kp_value = kp_value|(1<<3);
	if(HAL_GPIO_ReadPin(KP_COL5_GPIO_Port, KP_COL5_Pin))
		kp_value = kp_value|(1<<4);
	if(HAL_GPIO_ReadPin(KP_COL6_GPIO_Port, KP_COL6_Pin))
		kp_value = kp_value|(1<<5);
	if(HAL_GPIO_ReadPin(KP_COL7_GPIO_Port, KP_COL7_Pin))
		kp_value = kp_value|(1<<6);
	return kp_value;
}	

uint32_t kp_scan(void)
{ 
	uint32_t kp_value = 0;
	
	KP_ROW4s;	kp_value = scanrow(kp_value);  KP_ROW4c;
	kp_value = kp_value<<8;

	KP_ROW3s;	kp_value = scanrow(kp_value);  KP_ROW3c;
	kp_value = kp_value<<8;

	KP_ROW2s;	kp_value = scanrow(kp_value);  KP_ROW2c;
	kp_value = kp_value<<8;

	KP_ROW1s;	kp_value = scanrow(kp_value);  KP_ROW1c;

	return kp_value;
}

uint32_t kp_verify(uint32_t input)
{
  uint32_t data = input;
	uint8_t single = 0;
  for(int i=0; i<32; i++)
	{
		if(input&0x80000000) single++;
    input = input<<1;
	} 
	if(single==1) return data;
	else return 0;
}

uint8_t kp_identify(uint32_t input)
{ uint64_t id=0;
	uint8_t pos=0;
  if(input>0)
	{	
	 do
	 {
	   pos++;
		 input = input>>1;
	 }while(input>0);

//   id = kp_ledpos(pos);
  }

//	leddata(id);
	return pos;
}

uint8_t kp_input(void)
{
	uint8_t nvalue;
  uint32_t kp_value1;

	kp_value1 = kp_scan();
	kp_value1 = kp_verify(kp_value1);
	nvalue = kp_identify(kp_value1);
		
  return nvalue;
}

uint32_t sd_entry(uint8_t pos)
{
	uint32_t temp=0;

	if(pos!=0)
	{
   switch(pos)
	 {																											// Please refer TLC excel sheet for understanding Key press mapping to Server data
			case	13	:  temp  = 	(1<<	SD_UB1	);   break;
			case	21	:  temp  = 	(1<<	SD_UB2	);   break;
			case	5	:  temp  = 	(1<<	SD_UB3	);   break;
			case	29	:  temp  = 	(1<<	SD_UB4	);   break;
			case	22	:  temp  = 	(1<<	SD_UB5	);   break;
			case	28	:  temp  = 	(1<<	SD_UB6	);   break;
			case	7	:  temp  = 	(1<<	SD_UB7	);   break;
			case	23	:  temp  = 	(1<<	SD_UB8	);   break;
			case	15	:  temp  = 	(1<<	SD_UB9	);   break;
			case	14	:  temp  = 	(1<<	SD_UB10	);   break;
			case	6	:  temp  = 	(1<<	SD_UB11	);   break;
			case	20	:  temp  = 	(1<<	SD_UB12	);   break;
			case	18	:  temp  = 	(1<<	SD_GB1	);   break;
			case	26	:  temp  = 	(1<<	SD_GB2	);   break;
			case	10	:  temp  = 	(1<<	SD_GB3	);   break;
			case	2	:  temp  = 	(1<<	SD_GB4	);   break;
			case	3	:  temp  = 	(1<<	SD_GB5	);   break;
			case	27	:  temp  = 	(1<<	SD_VACB	);   break;
			case	25	:  temp  = 	(1<<	SD_AGB1	);   break;
			case	17	:  temp  = 	(1<<	SD_AGB2	);   break;
			case	9	:  temp  = 	(1<<	SD_AGB3	);   break;
			case	1	:  temp  = 	(1<<	SD_AGB4	);   break;
			case	19	:  temp  = 	(1<<	SD_AGB5	);   break;
			case	11	:  temp  = 	(1<<	SD_MFB	);   break;

  		default: temp = 0;  break;
	 }	
  }
	return temp;
}


void user_interface(void)
{																															  
  kp_prev = kp_next;																						// Used to avoid keypress bounce and long keypress
	kp_next = kp_input();																					// Save the key press input after validation
		
	if(kp_next==0) kp_valid = kp_prev; 														// Saves the key press value after key press settles to zero
			
	if(kp_valid==KP_NFB) { if(stage==1) stage=0; else stage=1; }  // If ON OFF pressed, toggle stage 1 in next step
  if(kp_valid==KP_GRB) { if(stage==2) stage=0; else stage=2; }	// If Guest Reg pressed, toggle stage 2 in next step
  if(kp_valid==KP_VACB){ if(stage==3) stage=0; else stage=3; }	// If Vaccation pressed, toggle stage 3 in next step
		
  switch(stage)																									// State machine logic used to decide the remote interface flow
	{
	  case 1: reg_viewer();  																			// Call for user & guest viewer entry		
						leddata(ledreg);																		// Write the led data once received
						ledblink(100, 100);																	// Blink the user entry
		        if(kp_valid==KP_VACB) {stage = 0; kp_valid=0;}			// Exit on random key press
		        if(kp_valid==KP_GRB) {stage = 0; kp_valid=0;}				// Exit on random key press
		break;
		
		case 2: reg_guest(); stage = 0;															// Call for guest registration 
		break;
		
		case 3:	ledreg = 0;
			      ledreg = setxtraled(ledreg, LD_VACB);								// Set the LED_Vaccation 
		        ms2enable = 1; if(ms2counts > 2000)									// Enable the ms2counts by setting ms2enable
						{ stage = ms2counts = ms2enable = 0; }							// If ms2counts is greater by 3 seconds clear the values
            serdata ^= sd_entry(kp_valid); 										  // Set or toggle the Serdata for vaccation bit
					  leddata(ledreg);																	  // Set the LED Vaccation Bit in TLC5954 
					  ledblink(100, 100);																  // Blink the user entry
		        if(kp_valid==KP_NFB) {stage = 0; kp_valid=0;}			  // Exit on random key press
		        if(kp_valid==KP_GRB) {stage = 0; kp_valid=0;}				// Exit on random key press
		break;				
			
		default: ledreg=0; leddata(ledreg); //ledblink(1, 0);	      // Clear all LED data in TLC5954 if no values valid
		break;																											// Also use minimum delay for fast refresh
	}			

  HAL_GPIO_WritePin(GEN_F_GPIO_Port, GEN_F_Pin, GPIO_PIN_SET);	// Clear the Female Led necessary  
	HAL_GPIO_WritePin(GEN_M_GPIO_Port, GEN_M_Pin, GPIO_PIN_SET);	// Clear the Male LED necessary

}
	
	
	
void reg_viewer(void)
{
	ledreg = 0;
  
	switch(kp_valid)
	{							// Please refer TLC excel sheet for understanding Key press mapping to Server data
								// It checks whether the mask is ON or OFF for respective bits and then allow the change to server data respectively
	  case KP_UB1:  if((sermask & 1<<SM_UB1)!=0)  serdata ^= (1<<(SD_UB1)); break;
	  case KP_UB2:  if((sermask & 1<<SM_UB2)!=0)  serdata ^= (1<<(SD_UB2)); break;
	  case KP_UB3:  if((sermask & 1<<SM_UB3)!=0)  serdata ^= (1<<(SD_UB3)); break;
	  case KP_UB4:  if((sermask & 1<<SM_UB4)!=0)  serdata ^= (1<<(SD_UB4)); break;
	  case KP_UB5:  if((sermask & 1<<SM_UB5)!=0)  serdata ^= (1<<(SD_UB5)); break;
	  case KP_UB6:  if((sermask & 1<<SM_UB6)!=0)  serdata ^= (1<<(SD_UB6)); break;
	  case KP_UB7:  if((sermask & 1<<SM_UB7)!=0)  serdata ^= (1<<(SD_UB7)); break;
	  case KP_UB8:  if((sermask & 1<<SM_UB8)!=0)  serdata ^= (1<<(SD_UB8)); break;
	  case KP_UB9:  if((sermask & 1<<SM_UB9)!=0)  serdata ^= (1<<(SD_UB9)); break;
	  case KP_UB10: if((sermask & 1<<SM_UB10)!=0) serdata ^= (1<<(SD_UB10)); break;
	  case KP_UB11: if((sermask & 1<<SM_UB11)!=0) serdata ^= (1<<(SD_UB11)); break;
	  case KP_UB12: if((sermask & 1<<SM_UB12)!=0) serdata ^= (1<<(SD_UB12)); break;
	  case KP_GB1:  if((sermask & 1<<SM_GB1)!=0)  serdata ^= (1<<(SD_GB1));  break;
    case KP_GB2:  if((sermask & 1<<SM_GB2)!=0)  serdata ^= (1<<(SD_GB2));  break;
	  case KP_GB3:  if((sermask & 1<<SM_GB3)!=0)  serdata ^= (1<<(SD_GB3));  break;
	  case KP_GB4:  if((sermask & 1<<SM_GB4)!=0)  serdata ^= (1<<(SD_GB4));  break;
	  case KP_GB5:  if((sermask & 1<<SM_GB5)!=0)  serdata ^= (1<<(SD_GB5));  break;			
																							// Toggles the server data											
		default: break;
	}
	
	ledreg  = showuser(serdata);                                      // Show current user viewer in server data
	ledreg |= showguest(serdata);																			// Show current guest viewer in server data
	ledreg = setxtraled(ledreg, LD_NFB1);															// Blink the ON OFF LED 
  ledreg = setxtraled(ledreg, LD_NFB2);
	ledreg = setxtraled(ledreg, LD_NFB3);
	ledreg = setxtraled(ledreg, LD_NFB4);	
	ledreg = setxtraled(ledreg, LD_SRB1);															// Blink the Star Rating LED
	ledreg = setxtraled(ledreg, LD_SRB2);	
	ledreg = setxtraled(ledreg, LD_SRB3);	
	ledreg = setxtraled(ledreg, LD_SRB4);	
  	
}

void clrdata(uint8_t bitname)
{
  uint32_t temp;
	temp = (1<<(bitname-1));
	temp = ~temp;
	serdata &= temp;
}


void reg_guest(void)
{ 
	uint8_t frwd=0;													// frwd variable is used to identify the state running for guest registration																								
	ledreg = 0;
	do
  {
 		kp_prev = kp_next;										// Used to avoid keypress bounce and long keypress
		kp_next = kp_input();									// Save the key press input after validation
		HAL_Delay(1);
		if(kp_next==0) kp_valid = kp_prev; 		// Saves the key press value after key press settles to zero
		
		if((sermask & 1<<SM_GB1)==0)       { ledreg = setxtraled(ledreg, LD_GB1);  frwd = 1; }  // if Guest1 is not registered blink GB1 LED for entry
    else if((sermask & 1<<SM_GB2)==0)  { ledreg = setxtraled(ledreg, LD_GB2);  frwd = 2; }  // if Guest2 is not registered blink GB2 LED for entry
    else if((sermask & 1<<SM_GB3)==0)  { ledreg = setxtraled(ledreg, LD_GB3);  frwd = 3; }  // if Guest3 is not registered blink GB3 LED for entry
    else if((sermask & 1<<SM_GB4)==0)  { ledreg = setxtraled(ledreg, LD_GB4);  frwd = 4; }  // if Guest4 is not registered blink GB4 LED for entry
    else if((sermask & 1<<SM_GB5)==0)  { ledreg = setxtraled(ledreg, LD_GB5);  frwd = 5; }  // if Guest5 is not registered blink GB5 LED for entry
    else								{ ledreg = setxtraled(ledreg, LD_GB1);  						 // if all above guest registered blink all and turn off after 3 secs
													ledreg = setxtraled(ledreg, LD_GB2);
													ledreg = setxtraled(ledreg, LD_GB3);
													ledreg = setxtraled(ledreg, LD_GB4);
													ledreg = setxtraled(ledreg, LD_GB5);
			                    ms2enable = 1; if(ms2counts > 2000)								 // ms2counts enabled by setting ms2enable and check for 3000 msecs 
													{																									 // Once 3000msecs or 3 secs over clear all guest led and disable the ms2counts
														ledreg = clrxtraled(ledreg, LD_GB1);  
													  ledreg = clrxtraled(ledreg, LD_GB2);
													  ledreg = clrxtraled(ledreg, LD_GB3);
													  ledreg = clrxtraled(ledreg, LD_GB4);
													  ledreg = clrxtraled(ledreg, LD_GB5);
														ms2counts = ms2enable = 0; 
														frwd = 30;																			 // Used for knowing the exit state from guest registration
													}
												}
	
    ledreg = setxtraled(ledreg, LD_GRB);																		 // Blink GRB LED to know the guest registration state in ON
		leddata(ledreg);
			
		if(frwd==0)																															 // If not assigned here All Guest registered will not be displayed
	   ledblink(100, 100);																											 // Blink the set LED for 500 ms ON and 500 ms OFF 
	}while(frwd==0);																													 // Performed only once 
  reg_guestdetails(frwd);																										 // Sequence for entering the guest details
}

void reg_guestdetails(uint8_t frwd)
{ 			
	serdata &= 0xff83ffff;									// Mask to clear the age bit data from server data																		
  serdata |= (1<<(SD_AGB1));							// By default always select Age Group 1 for registration
	serdata |= (1<<(SD_MFB));								// By default always select Female for registration
	uint32_t temp;
	temp = ledreg;										      // Backup of LED register taken for knowing GRB & which Guest registration is in progress
 
  do
	{	
 		kp_prev = kp_next;										// Used to avoid keypress bounce and long keypress
		kp_next = kp_input();									// Save the key press input after validation
		HAL_Delay(1);
		if(kp_next==0) kp_valid = kp_prev; 		// Saves the key press value after key press settles to zero
		
	  switch(kp_valid)
	  {
	    case KP_AGB1: serdata &= 0xff83ffff;  serdata |= (1<<(SD_AGB1));  break;  // Only one Age group selection is allowed as per Key Press
	    case KP_AGB2: serdata &= 0xff83ffff;  serdata |= (1<<(SD_AGB2));  break;  // Only one Age group selection is allowed as per Key Press
	    case KP_AGB3: serdata &= 0xff83ffff;  serdata |= (1<<(SD_AGB3));  break;  // Only one Age group selection is allowed as per Key Press
	    case KP_AGB4: serdata &= 0xff83ffff;  serdata |= (1<<(SD_AGB4));  break;  // Only one Age group selection is allowed as per Key Press
	    case KP_AGB5: serdata &= 0xff83ffff;  serdata |= (1<<(SD_AGB5));  break;  // Only one Age group selection is allowed as per Key Press
      case KP_MFB : serdata ^= (1<<(SD_MFB));    break;													// Toggle the gender entry as per Key Press
			case KP_GRB:  if(frwd==1) { serdata |= (1<<SD_GB1); sermask |= 1<<SM_GB1; }  // Set the Guest 1 registered in server data & mask
								    if(frwd==2) { serdata |= (1<<SD_GB2); sermask |= 1<<SM_GB2; }  // Set the Guest 2 registered in server data & mask
										if(frwd==3) { serdata |= (1<<SD_GB3); sermask |= 1<<SM_GB3; }  // Set the Guest 3 registered in server data & mask
										if(frwd==4) { serdata |= (1<<SD_GB4); sermask |= 1<<SM_GB4; }  // Set the Guest 4 registered in server data & mask
										if(frwd==5) { serdata |= (1<<SD_GB5); sermask |= 1<<SM_GB5; }  // Set the Guest 5 registered in server data & mask			
										frwd = 30; break;																							 // Registration successfull and exit
			
			case KP_NFB:  if(frwd==1) serdata &= ~(1<<(SD_GB1));	// Clear the Guest 1 registered in server data only
										if(frwd==2) serdata &= ~(1<<(SD_GB2));	// Clear the Guest 1 registered in server data only
										if(frwd==3) serdata &= ~(1<<(SD_GB3));	// Clear the Guest 1 registered in server data only
										if(frwd==4) serdata &= ~(1<<(SD_GB4));	// Clear the Guest 1 registered in server data only
										if(frwd==5) serdata &= ~(1<<(SD_GB5));	// Clear the Guest 1 registered in server data only
										
										frwd = 30; break;												// Registration unsuccessfull and exit
										
			case KP_VACB:	if(frwd==1) serdata &= ~(1<<(SD_GB1));	// Clear the Guest 1 registered in server data only
										if(frwd==2) serdata &= ~(1<<(SD_GB2));	// Clear the Guest 1 registered in server data only
										if(frwd==3) serdata &= ~(1<<(SD_GB3));	// Clear the Guest 1 registered in server data only
										if(frwd==4) serdata &= ~(1<<(SD_GB4));	// Clear the Guest 1 registered in server data only
										if(frwd==5) serdata &= ~(1<<(SD_GB5));	// Clear the Guest 1 registered in server data only
										
										frwd = 30; break;												// Registration unsuccessfull and exit
								
		  default: break;
	  }

		
    showgender(serdata);																		// Toggles the server data 
    ledreg = showage(serdata);															// Show the Age group which is selected
		
		ledreg |= temp;																					// Combined the new LED status with previous backup
		leddata(ledreg);																				// Load the LED data in TLC5954

		ledblink(100, 100);																			// Assigned the Blink time ON & OFF
		
  }while(frwd<30);																					// Exit as per condition
}



uint32_t showuser(uint32_t entry)
{
	uint8_t pos = 0;	uint32_t temp = 0;
	
  do 
	{
	 pos++;																									 // Incremented first for starting switch index with 1
	 if(entry&0x00000001)																		 // Checked the entry as per bitwise position
	 {
	   switch(pos)
		 {  // Set the LED as per 1's position
				case	1	:  temp  |= 	(1<<	LD_UB1	);   break;
				case	2	:  temp  |= 	(1<<	LD_UB2	);   break;
				case	3	:  temp  |= 	(1<<	LD_UB3	);   break;
				case	4	:  temp  |= 	(1<<	LD_UB4	);   break;
				case	5	:  temp  |= 	(1<<	LD_UB5	);   break;
				case	6	:  temp  |= 	(1<<	LD_UB6	);   break;
				case	7	:  temp  |= 	(1<<	LD_UB7	);   break;
				case	8	:  temp  |= 	(1<<	LD_UB8	);   break;
				case	9	:  temp  |= 	(1<<	LD_UB9	);   break;
				case	10	:  temp  |= 	(1<<	LD_UB10	);   break;
				case	11	:  temp  |= 	(1<<	LD_UB11	);   break;
				case	12	:  temp  |= 	(1<<	LD_UB12	);   break;

       default : break;
		 }		 
	 } 
	 entry  = entry>>1;																		  // Shifted the entry right for checking lsb 
	}while(entry>0);																				// Performed untill zero is received i.e. no more data
	
	return temp;																						// return the LED status for the entry
}



uint32_t showguest(uint32_t entry)
{
	uint8_t pos = 0;	uint64_t temp = 0;										
  do 
	{
	 pos++;																									// Incremented first for starting switch index with 1
	 if(entry&0x00000001)																		// Checked the entry as per bitwise position
	 {
	   switch(pos)
		 {  // Set the LED as per 1's position
				case	13	:  temp  |= 	(1<<	LD_GB1	);   break;
				case	14	:  temp  |= 	(1<<	LD_GB2	);   break;
				case	15	:  temp  |= 	(1<<	LD_GB3	);   break;
				case	16	:  temp  |= 	(1<<	LD_GB4	);   break;
				case	17	:  temp  |= 	(1<<	LD_GB5	);   break;

       default : break;
		 }
			 
	 } 
	 entry  = entry>>1;																		 // Shifted the entry right for checking lsb 
	}while(entry>0);																			 // Performed untill zero is received i.e. no more data
  	
	return temp;																					 // return the LED status for the entry
}

uint32_t showage(uint32_t entry)
{
	uint8_t pos = 0;	uint64_t temp = 0;
  do 
	{
	 pos++;																								 // Incremented first for starting switch index with 1
	 if(entry&0x00000001)																	 // Checked the entry as per bitwise position
	 {
	   switch(pos)
		 {  // Set the LED as per 1's position
				case	19	:  temp  |= 	(1<<	LD_AGB1	);   break;
				case	20	:  temp  |= 	(1<<	LD_AGB2	);   break;
				case	21	:  temp  |= 	(1<<	LD_AGB3	);   break;
				case	22	:  temp  |= 	(1<<	LD_AGB4	);   break;
				case	23	:  temp  |= 	(1<<	LD_AGB5	);   break;

       default : break;
		 }
			 
	 } 
	 entry  = entry>>1;																	  // Shifted the entry right for checking lsb 
	}while(entry>0);																			// Performed untill zero is received i.e. no more data
  	
	return temp;																				  // return the LED status for the entry
}


void showgender(uint32_t entry)
{
	if(entry&0x800000)																								// Check the Male-female bit in serverdata
	{
	  HAL_GPIO_WritePin(GEN_F_GPIO_Port, GEN_F_Pin, GPIO_PIN_SET);    // Set the Female LED
	  HAL_GPIO_WritePin(GEN_M_GPIO_Port, GEN_M_Pin, GPIO_PIN_RESET);	// Clear the Male LED
	}
  else
	{
	  HAL_GPIO_WritePin(GEN_F_GPIO_Port, GEN_F_Pin, GPIO_PIN_RESET);	// Clear the Female LED
	  HAL_GPIO_WritePin(GEN_M_GPIO_Port, GEN_M_Pin, GPIO_PIN_SET);		// Set the Male LED
	}
}




/* USER CODE END 1 */
/************************ (C) COPYRIGHT Vayve Technology *****END OF FILE****/
