/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ala_Config.h
  * @brief   This file contains all the function prototypes for
  *          the ala_sendData.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 08/07/2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define KP 0.1
#define MIN_SPEED 3
#define MAX_SPEED 50

//#define Min_Speed1 3
//#define Max_Speed1 250
#define Min_Speed1 3
#define Max_Speed1 50

#ifndef __ALA_CONGIG_H__
#define __ALA_CONGIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
/* -------------------------KHAI BAO DINH NGHIA PWM---------------------------*/
extern uint8_t motorData[80];																			// mang du lieu truyen di


#define	byteStart1														motorData[0]=0xff		// byte dau cho chuoi truyen di	
#define	ID1 		 															motorData[1]=1			//  byte dia chi
#define mor_0hRO_back 	 					 						motorData[2]=1			//  byte dao chieu dong co
#define mor_0hRO_next 												motorData[2]=0			
#define	mor_0hRO			 		 										motorData[3]			// 	byte toc do

#define	byteStart2														motorData[4]=0xff	
#define	ID2 		 															motorData[5]=2
#define mor_0h_next 											 		motorData[6]=0
#define mor_0h_back 													motorData[6]=1
#define	mor_0h																motorData[7]

#define	byteStart3														motorData[8]=0xff	
#define	ID3 		 															motorData[9]=3
#define mor_4hRo_back 												motorData[10]=1
#define mor_4hRo_next 												motorData[10]=0
#define	mor_4hRo 															motorData[11]

#define	byteStart4														motorData[12]=0xff	
#define	ID4 		 															motorData[13]=4
#define mor_4h_next 													motorData[14]=0
#define mor_4h_back 													motorData[14]=1
#define	mor_4h 																motorData[15]

#define	byteStart5														motorData[16]=0xff	
#define	ID5 		 															motorData[17]=5
#define mor_8hRo_next 												motorData[18]=0
#define mor_8hRo_back 												motorData[18]=1
#define	mor_8hRo 															motorData[19]

#define	byteStart6														motorData[20]=0xff	
#define	ID6 		 															motorData[21]=6
#define mor_8h_next 													motorData[22]=0
#define mor_8h_back 													motorData[22]=1
#define	mor_8h 																motorData[23]

#define	byteStart7														motorData[24]=0xff	
#define	ID7	 															    motorData[25]=7
#define mor_9hRo_next 												motorData[26]=0
#define mor_9hRo_back 												motorData[26]=1
#define	mor_9hRo 														  motorData[27]

#define	byteStart8														motorData[28]=0xff	
#define	ID8	 															    motorData[29]=8
#define mor_9h_next 													motorData[30]=1
#define mor_9h_back 													motorData[30]=0
#define	mor_9h 														    motorData[31]

#define	byteStart9														motorData[32]=0xff	
#define	ID9	 															    motorData[33]=9
#define xoayNang_next 												motorData[34]=0
#define xoayNang_back 												motorData[34]=1
#define	xoayNang 													  	motorData[35]

#define	byteStart10												  	motorData[36]=0xff	
#define	ID10	 															  motorData[37]=10
#define goc_Ban_next 												  motorData[38]=0
#define goc_Ban_back 											  	motorData[38]=1
#define	goc_Ban 														  motorData[39]


#define	byteStart11														motorData[40]=0xff	
#define	ID11 	 															  motorData[41]=11
#define mam_Xoay_next    											motorData[42]=0
#define mam_Xoay_back 								    		motorData[42]=1
#define	mam_Xoay 												  		motorData[43]

#define	byteStart12														motorData[44]=0xff	
#define	ID12	 															  motorData[45]=12
#define DC_Ban3h_next 										  	motorData[46]=1
#define DC_Ban3h_back 										  	motorData[46]=0
#define	DC_Ban3h 														  motorData[47]

#define	byteStart13														motorData[48]=0xff	
#define	ID13	 															  motorData[49]=13
#define DC_Ban6h_next 				  							motorData[50]=0
#define DC_Ban6h_back 		   									motorData[50]=1
#define	DC_Ban6h 														  motorData[51]

#define	byteStart14														motorData[52]=0xff	
#define	ID14	 															  motorData[53]=14
#define DC_Ban0h_next    											motorData[54]=1
#define DC_Ban0h_back 						    				motorData[54]=0
#define	DC_Ban0h 									  					motorData[55]

#define	byteStart15														motorData[56]=0xff	
#define	ID15	 															  motorData[57]=15
#define DC_Nap_next    											  motorData[58]=0
#define DC_Nap_back 						    			   	motorData[58]=1
#define	DC_Nap 									  				   	motorData[59]

#define	byteStart16														motorData[60]=0xff	
#define	ID16	 															  motorData[61]=16
#define Den_On    											      motorData[62]=0
#define Den_Off 						    			      	motorData[62]=1
#define	Den    									  				   	motorData[63]

#define	byteStart20														motorData[76]=0xff	
#define	ID20	 															  motorData[77]=20		// Van set ID 20
//#define DriverVan_On    											motorData[78]=1
//#define DriverVan_Off 						    			  motorData[78]=0
#define	DriverVan    									  			motorData[79]

//Chi doi ten Van khong sua phia sau

#define Den_Xanh_on 					DriverVan|=0b00000001
#define Den_Xanh_off 					DriverVan&=0b11111110

#define Den_Vang_on 					DriverVan|=0b00000010
#define Den_Vang_off 					DriverVan&=0b11111101

#define Den_Do_on 						DriverVan|=0b00000100
#define Den_Do_off 						DriverVan&=0b11111011

#define Van_4_on 							DriverVan|=0b00001000
#define Van_4_off 						DriverVan&=0b11110111

#define Van_5_on 							DriverVan|=0b00010000
#define Van_5_off 						DriverVan&=0b11101111

#define Van_6_on 							DriverVan|=0b00100000
#define Van_6_off 						DriverVan&=0b11011111

#define Van_7_on 							DriverVan|=0b01000000
#define Van_7_off 						DriverVan&=0b10111111

#define Van_8_on 							DriverVan|=0b10000000
#define Van_8_off 						DriverVan&=0b01111111

#define RED_BLUE	          	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) // HIGH IS BLUE - LOW IS RED

#define denVangOn							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET)
#define denVangOff	  	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET)

#define d1On	 	  	        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET)
#define d1Off	  	        		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET)

#define d3On	 	  	        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET)
#define d3Off	  		        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET)

//#define LazerOn	 	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)
//#define LazerOff	  		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)

//#define denDoOn	 	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
//#define denDoOff	  	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)

//#define denXanhOn	 	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET)
//#define denXanhOff	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET)
//#define d8On	 	  	    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
//#define d8Off	  	    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
//#define denVangOn 	  	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)
//#define denVangOff	  	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)

#define Buzzer	 		    TIM12 -> CCR1		//B14
//Khai bao INPUT
// IO-3
#define CB_0h	 		    		    HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)
#define CB_4h	 		      			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)
#define CB_8h	 		    	      HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)
#define CTHT_Duoi           	HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)
#define CTHT_Tren         	 	HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)
#define CB_Quang	 		      	    HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)
#define CB_nhan	 	      			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define CTHT	 	        			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)


// IO-2
#define CB_9h	 		      					HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define CB_nhanBong_Duoi	 		    HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)
#define CB_4	 		          			HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12)
#define CB_nhanBong_Tren	 		    HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)
#define Home_napBanh	 		        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)
#define CB_Nh	                  	HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)
#define CB_2	 		  	            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)
#define CB_1            	 				HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)
/*---------------------Ket thuc khai bao chuoi du lieu dieu khien Driver--------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern uint16_t crc_1, crc_2;

typedef struct struct_ControlMotor {
  uint8_t a[8];    
  // uint8_t b;
  // uint8_t c;
  // uint8_t d;
  // uint8_t e;
  // uint8_t f;
  // uint8_t g;
  // uint8_t h;
} struct_message;

int32_t readEncoderTim_1(void);
int32_t readEncoderTim_2(void);
int32_t readEncoderTim_3(void);
int32_t readEncoderTim_4(void);
int32_t readEncoderTim_5(void);
int32_t readEncoderTim_8(void);
int32_t en_mamXoay(void);
int32_t readEncoderCan2_Ex(void);

void putchar4(unsigned char ch);
void run_read_gyro_uart4(void);
void sendData_Gamepad_Grygo(void);
void sendDataToDriver(void);
//void Error_Handler(void);
//void UART_DMA_Transmit(uint8_t *pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __ALA_CONGIG_H__ */

