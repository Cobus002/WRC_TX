/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
//Joystick stuff
#define JS_X				0
#define JS_Y				1

#define TX		//comment out this line for RX

#ifndef TX
#define RX
#endif

#define clear(X)			memset(X, 0, sizeof(X))

#define SPI_MAX_TIMEOUT 	0x100

/************Commands********************/
#define R_MASK				0b00000000
#define W_MASK				0b00100000
#define R_RX_PAYLOAD		0b01100001
#define W_TX_PAYLOAD		0b10100000
#define FLUSH_TX			0b11100001
#define NOP 				0xff

/***********Registers********************/
#define CONFIG_REG				0x00
/*
 * 7: 	Reserved
 * 6:	MASK_RX_DR
 * 5:	MASK_TX_DS
 * 4:	MASK_MAX_RT
 * 3:	EN_CRC
 * 2:	CRCO
 * 1:	PWR_UP				[1: PWR UP, 0: PWR DOWN]
 * 0:	PRIM_RX				RX/TX Control [1: PRX, 0: PTX]
 */
#define	EN_CRC					(uint8_t)(3)
#define	PWR_UP					(uint8_t)(1)
#define PRIM_RX					(uint8_t)(0)

#define STATUS_REG				(uint8_t)0x07
/*
 * 7: 	Reserved
 * 6:	RX_DR
 * 5:	TX_DS					Set when data successfully sent
 * 4:	MAX_RT					Maximum retransmit tries, write 1 to clear
 * 3:	RX_P_NO[3]
 * 2:	RX_P_NO[2]
 * 1:	RX_P_NO[1]				[1: PWR UP, 0: PWR DOWN]
 * 0:	TX_FULL					TX_FIFO full
 */
#define RX_DR					(uint8_t)(6)
#define TX_DS					(uint8_t)(5)
#define MAX_RT					(uint8_t)(4)
#define TX_FULL					(uint8_t)(0)


#define EN_AA_REG 				0x01
#define EN_RXADDR_REG			0x02
#define RX_PW_P0				(uint8_t)0x11



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t irq_pin=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

HAL_StatusTypeDef send_receive_spi(uint8_t *pTX, uint8_t *pRX, uint8_t numBytes);

HAL_StatusTypeDef setup_tx(void);

HAL_StatusTypeDef setup_rx(void);

HAL_StatusTypeDef tx_send(uint8_t *pTX, uint8_t numBytes);

HAL_StatusTypeDef data_available(void);

HAL_StatusTypeDef get_data(uint8_t *dataBuff);

uint16_t joystick_read(ADC_HandleTypeDef *adc, uint8_t chan);

void joystick_init(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */



  uint8_t buff[3] = {0xFF, 0xFF, 0xff};
  uint8_t rxBuff[2] = {0x00, 0x00};
  uint8_t str[20];
  uint8_t on = 0xaa;
  uint8_t off = 0x66;

  //Clear the receive buffer, will be updated in the interrupt handler

  memset(str, '\0', sizeof(str));
  HAL_StatusTypeDef status;
  HAL_Delay(3000);	//Wait for the user to open comm port

  #ifdef TX
  //Usually sets itself up except for pwr_up bit
  //Use powr_up pin wen sending data
  joystick_init();
  HAL_Delay(10);
  int count = 0;
  uint16_t xValue=0;
  uint16_t yValue=0;
  #endif

  #ifdef RX
  status = setup_rx();
  switch(status){
  case HAL_OK:
	  strcpy(str, "RX Setup: Success\n\r");
	  break;
  case HAL_BUSY:
	  strcpy(str, "RX Setup: Busy\n\r");
	  break;
  case HAL_ERROR:
	  strcpy(str, "RX Setup: Error\n\r");
	  break;
  default:
	  strcpy(str, "SAD Face =(\n\r");
	  break;
  }

  CDC_Transmit_FS(str, strlen(str));


  #endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

#ifdef TX
	  if(count){
		  count=0;
		  buff[0]=off;
	  }else{
		  count++;
		  buff[0]=on;

	  }
	  status = tx_send(buff, 1);
	  switch(status){
	  case HAL_OK:
		  strcpy(str, "TX_SEND: Success \n\r");
		  break;
	  case HAL_BUSY:
		  strcpy(str, "TX_SEND: Busy\n\r");
		  break;
	  case HAL_ERROR:
		  strcpy(str, "TX_SEND: Error\n\r");
		  break;
	  default:
		  strcpy(str, "Sad face =( \n\r");
		  break;
	  }
	  //Send Debug message
	  CDC_Transmit_FS(str, strlen(str));
	  HAL_Delay(20);

	  //Check if max retransmits bit is set, means failed to send
	  	  if(((0x01)&(rxBuff[0]>>MAX_RT))>0){
	  		  //Maximum retransmits occured, failed to send
	  		  //Flush the TX_ fifo and then clear the bit
	  		  clear(buff);
	  		  clear (rxBuff);
	  		  buff[0] = (uint8_t)FLUSH_TX;
	  		  send_receive_spi(buff, rxBuff, 1);

	  		  //Now clear the bit
	  		  clear(buff);
	  		  clear (rxBuff);
	  		  buff[0] = (uint8_t)(W_MASK|STATUS_REG);
	  		  buff[1] = (uint8_t)(1<<MAX_RT);
	  		  send_receive_spi(buff, rxBuff, 2);

		  	  clear(rxBuff);
		  	  clear(buff);
		  	  buff[0] = (uint8_t)(R_MASK|CONFIG_REG);
		  	  buff[1]= (uint8_t)NOP;

		  	  if(send_receive_spi(buff, rxBuff, 2)==HAL_OK){
		  		  //print out the buff contents
		  		  memset(str, '\0', sizeof(str));
		  		  sprintf(str, ">>STAT: %02x CONF: %02x\r\n", rxBuff[0], rxBuff[1]);
		  		  CDC_Transmit_FS(str, strlen(str));
		  		  HAL_Delay(20);
		  	  }

	  	  }

	  	  HAL_Delay(1);

	  	  xValue = joystick_read(&hadc1, JS_X);
	  	  yValue = joystick_read(&hadc1, JS_Y);
	  	  memset(str, '\0', sizeof(str));
	  	  sprintf(str, "<->X: 0x%04x <->Y: 0x%04x\n\r", xValue, yValue);
	  	  CDC_Transmit_FS(str, strlen(str));
	  	  HAL_Delay(20);


#endif


	  //Check the status register and the config register
	  clear(rxBuff);
	  clear(buff);
	  buff[0] = (uint8_t)(R_MASK|CONFIG_REG);
	  buff[1]= (uint8_t)NOP;

	  if(send_receive_spi(buff, rxBuff, 2)==HAL_OK){
		  //print out the buff contents
		  memset(str, '\0', sizeof(str));
		  sprintf(str, "STAT: %02x CONF: %02x\r\n", rxBuff[0], rxBuff[1]);
		  CDC_Transmit_FS(str, strlen(str));
		  HAL_Delay(20);
	  }

	  /************Check if the intterupt has been triggered******/
	  if(irq_pin==1){
		  irq_pin =0;
		  CDC_Transmit_FS("Pressed\n\r", strlen("Pressed\n\r")+2);
		  HAL_Delay(20);
#ifdef RX
		  //Check if there is data available:
		  if(data_available()==HAL_OK){
			  //data is available
			  clear(rxBuff);
			  if(get_data(rxBuff)==HAL_OK){
				  //Got the data
				  if(rxBuff[1]==on){
					  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

				  }else if(rxBuff[1]==off){
					  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


				  }

				  memset(str, '\0', sizeof(str));
				  sprintf(str, "Got: %02x\n\r", rxBuff[1]);
				  CDC_Transmit_FS(str, strlen(str));


			  }


		  }

#endif
	  }else{
		  CDC_Transmit_FS("Not Pressed\n\r", strlen("Not Pressed\n\r")+2);
	  }

	  HAL_Delay(200);







  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRF_CE_Pin|NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */


/**
  * @brief  This function handles sending data over spi
  * @param
  * 	pTX			Transmit buffer
  * 	pRX			Receive buffer
  * 	numBytes	number of bytes to send
  * @retval
  * 	HAL_StatusTypeDef HAL_OK or HAIL_FAIL
  */
HAL_StatusTypeDef send_receive_spi(uint8_t *pTX, uint8_t *pRX, uint8_t numBytes){

	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status;
	status = HAL_SPI_TransmitReceive(&hspi1, pTX, pRX, 2 , SPI_MAX_TIMEOUT );
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
	return status;

}

HAL_StatusTypeDef setup_tx(void){
	return HAL_OK;


}


/**
  * @brief  This function sets up the nrf24l01 in rx mode
  * @param None
  * @retval HAL_StatusTypeDef
  *
  * 	HAL_OK 		all is good
  * 	HAL_BUSY	spi error
  * 	HAL_ERROR	config error
  */
HAL_StatusTypeDef setup_rx(void){

	uint8_t buff[]={0x00, 0x00};
	uint8_t rxBuff[] = {0x00, 0x00};
	uint8_t check =0x00;
	uint8_t errStr[20];
	memset(errStr, '\0', sizeof(errStr));

	//Power up the module
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	buff[0] = (uint8_t)(W_MASK|CONFIG_REG);
	buff[1] = (uint8_t)((1<<EN_CRC)|(1<<PWR_UP)|(1<<PRIM_RX));
	if(send_receive_spi(buff, rxBuff, 2)!=HAL_OK){
		return HAL_BUSY;
	}
	HAL_Delay(1);
	clear(buff);
	clear(rxBuff);

	buff[0] = (uint8_t)(R_MASK|CONFIG_REG);
	buff[1] = (uint8_t)NOP;
	if(send_receive_spi(buff, rxBuff, 2)!=HAL_OK){
		return HAL_BUSY;
	}
	//check if the data was set successfully
	check = (uint8_t)(uint8_t)((1<<EN_CRC)|(1<<PWR_UP)|(1<<PRIM_RX));
	if(check!=rxBuff[1]){
		sprintf(errStr, "515 STAT: %02x CONF: %02x\n\r",rxBuff[0], rxBuff[1]);
		CDC_Transmit_FS(errStr, strlen(errStr));
		return HAL_ERROR;
	}
	HAL_Delay(1);

	clear(buff);
	clear(rxBuff);
	buff[0] = (uint8_t)(W_MASK|RX_PW_P0);
	buff[1]	= 0x01;

	if(send_receive_spi(buff, rxBuff, 2)!=HAL_OK){
		return HAL_BUSY;
	}

	//Power up the module
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);

	//All is well and unicorns live
	return HAL_OK;
}


/**
  * @brief  This function tries to send data
  * @param
  *
  * @retval HAL_StatusTypeDef
  *
  * 	HAL_OK 		all is good
  * 	HAL_BUSY	spi error
  * 	HAL_ERROR	config error
  */
HAL_StatusTypeDef tx_send(uint8_t *pTX, uint8_t numBytes){
	uint8_t txBuff[]={0x00, 0x00};
	uint8_t rxBuff[]={0x00, 0x00};
	uint8_t *allData = (uint8_t *)malloc(sizeof(pTX)+2*sizeof(uint8_t));

	//Set the power up bit high
	txBuff[0] = (uint8_t)(W_MASK|CONFIG_REG);
	txBuff[1] = (uint8_t)((1<<EN_CRC)|(1<<PWR_UP));
	if(send_receive_spi(txBuff, rxBuff, 2)!=HAL_OK){
		return HAL_BUSY;
	}

	//Now send the data
	allData[0] = (uint8_t)(W_TX_PAYLOAD);
	memcpy(allData+sizeof(uint8_t), pTX, sizeof(pTX));

	if(send_receive_spi(allData, rxBuff, numBytes+1)!=HAL_OK){
		free(allData);
		return HAL_BUSY;
	}
	free(allData);

	//Data loaded into the device now pulse the CE pin for more than 10us
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

	return HAL_OK;
}


/**
  * @brief  This function checks the RX_DR bit in the status register to
  * see if there is data available
  * @param
  *
  * @retval HAL_StatusTypeDef
  *
  * 	HAL_OK 		all is good there is data
  * 	HAL_BUSY	spi error
  * 	HAL_ERROR	no data
  */
HAL_StatusTypeDef data_available(void){
	//Check the RX_DR bit to see if there is any data
	uint8_t reg = 0x00;
	uint8_t buff[] = {0x00, 0x00};
	buff[0] = NOP;

	if(send_receive_spi(buff, &reg, 1)!=HAL_OK){
		return HAL_BUSY;
	}

	if(((0x01)&(reg>>RX_DR))==0){
		//There is no data
		return HAL_ERROR;
	}
	//If we get here then there is data in the rx fifo

	return HAL_OK;
}


/**
  * @brief  This function gets the data in the RX_FIFO
  * @param
  * uint8_t *dataBuff		load the data into this buffer
  * @retval HAL_StatusTypeDef
  *
  * 	HAL_OK 		all is good there is data
  * 	HAL_BUSY	spi error
  * 	HAL_ERROR	no data
  */
HAL_StatusTypeDef get_data(uint8_t *dataBuff){
	//Read the data into the data buff which will auto clear the data
	uint8_t buff[]={0x00, 0x00};
	uint8_t rxBuff[]={0x00, 0x00};
	buff[0] = (uint8_t)R_RX_PAYLOAD;
	buff[1] = (uint8_t)NOP;

	clear(dataBuff);

	if(send_receive_spi(buff, dataBuff, 2)!=HAL_OK){
		return HAL_BUSY;
	}
	//Clear the RX_DR flag by writting a 1 to it

	buff[0] = (uint8_t)(W_MASK|STATUS_REG);
	buff[1]	= (uint8_t)(1<<RX_DR);
	if(send_receive_spi(buff, rxBuff, 2)!=HAL_OK){
		return HAL_BUSY;
	}
	return HAL_OK;
}


void joystick_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	//__HAL_RCC_GPIOB_CLK_ENABLE();
	/* Configure A0 as analog input */
	GPIO_InitStructure.Pin = JS_X_Pin;			//Set A0 pin
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister
	HAL_GPIO_Init(JS_X_GPIO_Port, &GPIO_InitStructure);

	/* Configure A0 as analog input */
	GPIO_InitStructure.Pin = JS_Y_Pin;			//Set A0 pin
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		//Set to Analog input
	GPIO_InitStructure.Pull = GPIO_NOPULL ;			//No Pull up resister
	HAL_GPIO_Init(JS_Y_GPIO_Port, &GPIO_InitStructure);

}


/**
  * @brief  This function gets the data in the RX_FIFO
  * @param
  *	ADC_HandleTypeDef   adc handler to be used
  *	uint8_t				the channel number that you want
  * @retval uint16_t converted adc value
  */
uint16_t joystick_read(ADC_HandleTypeDef *adc, uint8_t chan){
	uint16_t adcValues[2]={0x00, 0x00};
	uint8_t numConversions = 2;

	for(int i=0; i<numConversions; i++){
		HAL_ADC_Start(adc);
		while(HAL_ADC_PollForConversion(adc, 10)!=HAL_OK){
			;//Do nothing
		}
		//Conversion done
		adcValues[i]=(uint16_t)(HAL_ADC_GetValue(adc));
	}

	return adcValues[chan];

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
