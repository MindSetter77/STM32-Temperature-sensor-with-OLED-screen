/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdarg.h>
#include "onewire.h"
#include "ds18b20.h"
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//zmienne do obsługi protokołu
#define USART_TXBUF_LEN 300
#define USART_RXBUF_LEN 300
uint8_t USART_TxBUF[USART_TXBUF_LEN];
uint8_t USART_RxBUF[USART_RXBUF_LEN];

__IO int USART_TX_EMPTY=0; //wskaźnik dla bufora nadawczego
__IO int USART_TX_BUSY=0;
__IO int USART_RX_EMPTY=0; //wskaźnik dla bufora odbiorczego
__IO int USART_RX_BUSY=0;

//zmienne do ramki
const char nazwa_stm[3]="stm";
char nadawca[3];
char odbiorca[3];
char dlugoscKomendy2[3];
char sumaKontrolna2[2];
int sumaKontrolna;
int dlugoscKomendy;
int ramkaDlugosc=0;
int poczatekRamki=0;

int dlugosc;

float temperature;
char string[64];

uint8_t aShowTime[50] = {0};
uint8_t aShowDate[50] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t USART_kbhit(){
	if(USART_RX_EMPTY==USART_RX_BUSY){
		return 0;
	}else{
		return 1;
	}
}//USART_kbhit

int16_t USART_getchar(){
int16_t tmp;
	if(USART_RX_EMPTY!=USART_RX_BUSY){
		tmp=USART_RxBUF[USART_RX_BUSY];
		USART_RX_BUSY++;
		if(USART_RX_BUSY >= USART_RXBUF_LEN)USART_RX_BUSY=0;
		return tmp;
	}else return 0;
}//USART_getchar

uint8_t USART_getline(char *buf){
static uint8_t bf[300]; //bufor pomocniczy
int i;
int ret=0;
	while(USART_kbhit())
	{
		bf[ramkaDlugosc]=USART_getchar();
		USART_fsend("%c", bf[ramkaDlugosc]);
		if(bf[ramkaDlugosc]==58)
		{
			ramkaDlugosc=0;
			poczatekRamki=1;
		}
		if(bf[ramkaDlugosc]==59 && poczatekRamki==1)
		{
			if(ramkaDlugosc>=12)
			{
				for(int i = 0; i <= ramkaDlugosc; i++)
				{
					buf[i] = bf[i];
				}
				ret = ramkaDlugosc;
				ramkaDlugosc=0;
				poczatekRamki=0;
				dlugosc=ret;
				return ret;
			}
			ramkaDlugosc=0;
			poczatekRamki=0;
		}
		else
		{
			ramkaDlugosc++;
			if(ramkaDlugosc>=269)
			{
				ramkaDlugosc=0;
				poczatekRamki=0;
			}
		}
	}
	return 0;
}//USART_getline

void USART_fsend(char* format,...){
char tmp_rs[128];
int i;
__IO int idx;
va_list arglist;
	va_start(arglist,format);
	vsprintf(tmp_rs,format,arglist);
	va_end(arglist);
	idx=USART_TX_EMPTY;
	for(i=0;i<strlen(tmp_rs);i++)
	{
		USART_TxBUF[idx]=tmp_rs[i];
		idx++;
		if(idx>=USART_TXBUF_LEN)idx=0;
	}
	__disable_irq(); //wyłączenie przerwań
	if((USART_TX_EMPTY==USART_TX_BUSY)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET))
	{
		USART_TX_EMPTY=idx;
		uint8_t tmp=USART_TxBUF[USART_TX_BUSY];
		USART_TX_BUSY++;
		if(USART_TX_BUSY>=USART_TXBUF_LEN)USART_TX_BUSY=0;
		HAL_UART_Transmit_IT(&huart2,&tmp,1);
	}else{
		USART_TX_EMPTY=idx;
	}
	__enable_irq(); //włączenie przerwań
}
//nadawanie
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart2){
		if(USART_TX_EMPTY!=USART_TX_BUSY){
			uint8_t tmp=USART_TxBUF[USART_TX_BUSY];
			USART_TX_BUSY++;
			if(USART_TX_BUSY>=USART_TXBUF_LEN)USART_TX_BUSY=0;
			HAL_UART_Transmit_IT(&huart2,&tmp,1);
		}
	}
}
//odbiór
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart2)
	{
		USART_RX_EMPTY++;
		if(USART_RX_EMPTY>=USART_RXBUF_LEN)
			{
			USART_RX_EMPTY=0;
			}
		HAL_UART_Receive_IT(&huart2,&USART_RxBUF[USART_RX_EMPTY],1);
	}
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}
void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t read (void)
{
	uint8_t value=0;
	gpio_set_input ();

	for (int i=0;i<8;i++)
	{
		gpio_set_output ();   // set as output

		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us

		gpio_set_input ();  // set as input
		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}

void UruchomOled()
{





}

void ZatrzymajOled()
{
	SSD1306_Clear();
	SSD1306_UpdateScreen();
}


void SprawdzKomende(char *cmd)
{
	int suma=0;
	for(int i = 0; i < dlugoscKomendy; i++)
	{
		suma=suma+cmd[i];
	}

	suma=suma % 100;
	//USART_fsend("\n%d", suma);

	if(sumaKontrolna==suma)
	{
		if(cmd[0] == 'S' && cmd[1] == 'T' && cmd[2] == 'A' && cmd[3] == 'R' && cmd[4] == 'T' && dlugoscKomendy == 5)
		{
			UruchomOled();
		}
		else if(cmd[0] == 'S' && cmd[1] == 'T' && cmd[2] == 'O' &&cmd[3] == 'P' && dlugoscKomendy == 4)
		{
			ZatrzymajOled();
		}
		else
		{
			USART_fsend("\nBADCMD\r\n");
		}
	}
	else
	{
		USART_fsend("\BADSUM\r\n");
	}
}

void ReadFrame(char *buf)
{
	memcpy(nadawca, &buf[1], 3);
	memcpy(odbiorca, &buf[4], 3);
	memcpy(dlugoscKomendy2, &buf[7], 3);
	for(int i = 0; i<3; i++)
	{
		if(!isdigit(dlugoscKomendy2[i]))
		{
			USART_fsend("\nBADFRAME \r \n");
			return;
		}
	}
	dlugoscKomendy=atoi(dlugoscKomendy2);
	memcpy(sumaKontrolna2, &buf[dlugosc-2], 2);

	for(int i=0; i<2; i++)
	{
		if(!isdigit(sumaKontrolna2[i]))
		{
			USART_fsend("\nBADFRAME\r\n");
			return;
		}
	}
	sumaKontrolna = atoi(sumaKontrolna2);

	SprawdzRamke(buf);


}

void SprawdzRamke(char *buf)
{
	if(strncmp(nazwa_stm, odbiorca, 3) == 0)
	{
		if(dlugoscKomendy <= 256 && dlugoscKomendy > 0)
		{

			int poprawnaRamka = dlugoscKomendy+12;

			if(dlugosc < poprawnaRamka)
			{
				USART_fsend("\nBADCMD\r\n");
				return;
			}
			char command[dlugoscKomendy + 1];
			memcpy(command, &buf[10], dlugoscKomendy);
			SprawdzKomende(command);

		}
		else if(dlugoscKomendy == 0)
		{
			USART_fsend("\nBADCCOMM\r\n");
			return;
		}
		else
		{
			USART_fsend("\nLENCOMM\r\n");
			return;
		}
	}
	else
	{
		USART_fsend("\nBADFRAME\r\n");
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2,&USART_RxBUF[0],1);

      USART_fsend("=============\r\n",72);
      USART_fsend("START1 0x%04X\r\n",72);
      USART_fsend("=============\r\n",72);
      //uint32_t xxx=0;
  int len=0;
  char bx[300];
  while (1)
  {
	  HAL_TIM_Base_Start(&htim1);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  delay_us(10);

	  Presence = DS18B20_Start ();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t
	  HAL_Delay (800);

	  Presence = DS18B20_Start ();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad

	  Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	  if((len=USART_getline(bx))>0){
    	    		  ReadFrame(bx);
    	    	  }
    	      }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
