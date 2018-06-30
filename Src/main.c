/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "ringbuffer_dma.h"
#include <string.h>

// RingBuffer variables
//extern DMA_HandleTypeDef hdma_uart2_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* Ringbuffer for Rx messages */
RingBuffer_DMA rx_buf;
/* Array for DMA to save Rx bytes */
#define BUF_SIZE 256
uint8_t rx[BUF_SIZE];
uint32_t rx_count = 0;
/* Array for received commands */
char cmd[512];
uint8_t icmd = 0;

/* ESP8266 variables */
//#define THINGSPEAK_ADDRESS	"api.thingspeak.com"
#define THINGSPEAK_ADDRESS	"192.168.1.86"
#define THINGSPEAK_API_KEY	"5ZRXVI7HIJQPGKID"				// Enter your Write API here
//#define WIFI_SSID			"Kitchen"
//#define WIFI_PASS			"Papanina36"
#define WIFI_SSID			"Tapac"
#define WIFI_PASS			"27051329"

char wifi_cmd[512];

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void UART_Read(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	int i=50;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

	/* Init RingBuffer_DMA object */
	RingBuffer_DMA_Init(&rx_buf, &hdma_usart2_rx, rx, BUF_SIZE);
	/* Start UART4 DMA Reception */
	HAL_UART_Receive_DMA(&huart2, rx, BUF_SIZE);

	//sprintf(wifi_cmd,"\nHello ThingSpeak! \n\r");
	sprintf(wifi_cmd,"\nHello Impulse \n\r");
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 100);
  	HAL_Delay(100);

  	/* CONNECT TO WIFI ROUTER */
	/* Simple ping */
	sprintf(wifi_cmd, "AT+RST\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(500);
	UART_Read();

	/* Turn on message echo */
	sprintf(wifi_cmd, "ATE1\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(100);
	UART_Read();

	/* Set to client mode */
	sprintf(wifi_cmd, "AT+CWMODE=1\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(100);
	UART_Read();
	HAL_Delay(2000);

	/* Connect to network */
	sprintf(wifi_cmd, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(10000);
	UART_Read();
	/* CONNECTED (hope so) */
	/* Check for IP */
	sprintf(wifi_cmd, "AT+CIFSR\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(5000);
	UART_Read();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	i++;
	char http_req[200];
	//sprintf(http_req, "GET /update?api_key=%s&field1=%d\r\n\r\n",THINGSPEAK_API_KEY,i); // rand() % 100);
	sprintf(http_req, "http://192.168.1.86/menu.htm\r\n");

	/* Connect to server */
	sprintf(wifi_cmd, "AT+CIPSTART=\"TCP\",\"%s\",80\r\n", THINGSPEAK_ADDRESS);
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(2000);
	UART_Read();
	/* Send data length (length of request) */
	sprintf(wifi_cmd, "AT+CIPSEND=%d\r\n", strlen(http_req));
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(2000); // wait for ">"
	UART_Read();
	/* Send data */
	HAL_UART_Transmit(&huart1, (uint8_t *)http_req, strlen(http_req), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)http_req, strlen(http_req), 1000);
	HAL_Delay(100);
	UART_Read();



	sprintf(http_req, "http://192.168.1.86/cgi-bin/web_is_local?_=1530358626355\r\n");

	/* Connect to server */
	sprintf(wifi_cmd, "AT+CIPSTART=\"TCP\",\"%s\",80\r\n", THINGSPEAK_ADDRESS);
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(2000);
	UART_Read();

	/* Send data length (length of request) */
	sprintf(wifi_cmd, "AT+CIPSEND=%d\r\n", strlen(http_req));
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(2000); // wait for ">"
	UART_Read();
	/* Send data */
	HAL_UART_Transmit(&huart1, (uint8_t *)http_req, strlen(http_req), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)http_req, strlen(http_req), 1000);
	HAL_Delay(100);
	UART_Read();



	sprintf(http_req, "http://192.168.1.86/cgi-bin/ow_set_reg/19.4/1/1/?_=1530358346812\r\n");

	/* Connect to server */
	sprintf(wifi_cmd, "AT+CIPSTART=\"TCP\",\"%s\",80\r\n", THINGSPEAK_ADDRESS);
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(2000);
	UART_Read();

	/* Send data length (length of request) */
	sprintf(wifi_cmd, "AT+CIPSEND=%d\r\n", strlen(http_req));
	HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 1000);
	HAL_Delay(2000); // wait for ">"
	UART_Read();
	/* Send data */
	HAL_UART_Transmit(&huart1, (uint8_t *)http_req, strlen(http_req), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t *)http_req, strlen(http_req), 1000);
	HAL_Delay(100);
	UART_Read();


	/* Wait for next transmission (at least 15 sec for 1 loop) */
	HAL_Delay(10000);


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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//=================================================================
void UART_Read(void)
 {
	/* Check number of bytes in RingBuffer */
	rx_count = RingBuffer_DMA_Count(&rx_buf);
	/* Process each byte individually */
	while (rx_count--) {
		/* Read out one byte from RingBuffer */
		uint8_t b = RingBuffer_DMA_GetByte(&rx_buf);
		if (b == '\n') { /* If \n process command */
			/* Terminate string with \0 */
			cmd[icmd] = 0;
			icmd = 0;
			/* Display received messages */
			//LCD_Printf("%s\n", cmd);
			sprintf(wifi_cmd,"%s\r\n", cmd);
			HAL_UART_Transmit(&huart1, (uint8_t *)wifi_cmd, strlen(wifi_cmd), 100);
		  	HAL_Delay(100);
		} else if (b == '\r') { /* If \r skip */
			continue;
		} else { /* If regular character, put it into cmd[] */
			cmd[icmd++ % 512] = b;
		}
	}
 }// void UART_Read(void)
//======================================================================
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
