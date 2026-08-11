/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "bme280_app.h"
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
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {.name = "defaultTask",.stack_size = 128 * 4,.priority = (osPriority_t) osPriorityNormal,};
/* USER CODE BEGIN PV */
volatile int8_t bmeInitStatus = BME280_E_COMM_FAIL;

typedef struct
{
    uint32_t sampleNumber;
    uint32_t timestampMs;
    float temperatureC;
    float pressureHpa;
    float humidityPercent;
} SensorLogData_t;

QueueHandle_t sensorLogQueue = NULL;


#define COMMAND_BUFFER_SIZE 64

typedef struct
{
    uint32_t sampleRateMs;
    uint8_t loggingEnabled;
} SystemConfig_t;

SystemConfig_t systemConfig =
{
    .sampleRateMs = 1000,
    .loggingEnabled = 0
}; //shared resource protected by mutex

uint8_t uartRxByte;

char uartRxBuffer[COMMAND_BUFFER_SIZE];
char commandBuffer[COMMAND_BUFFER_SIZE];

volatile uint8_t commandReady = 0;
volatile uint32_t uartRxIndex = 0;
volatile uint8_t uartRxOverflow = 0;
volatile uint8_t uartTypingActive = 0;

TaskHandle_t commandTaskHandle=NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t sdTaskHandle = NULL;

SemaphoreHandle_t uartMutex = NULL;
SemaphoreHandle_t configMutex = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void CommandTask(void *argument);
void SensorTask(void *argument);
void SDTask(void *argument);
void UART_Send(const char *message);
void Config_Get(SystemConfig_t *config);
void Config_SetLogging(uint8_t enabled);
void Config_SetRate(uint32_t rate);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  char bmeMessage[60];

  bmeInitStatus = BME280_App_Init();

  snprintf(bmeMessage,sizeof(bmeMessage),"BME280 init result: %d\r\n",bmeInitStatus);
  HAL_UART_Transmit(&huart2,(uint8_t *)bmeMessage,strlen(bmeMessage),1000);

  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  /* Create queue, mutexes and tasks */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  uartMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();

  if ((uartMutex == NULL) || (configMutex == NULL))
  {
      Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  sensorLogQueue = xQueueCreate(16, sizeof(SensorLogData_t));

  if (sensorLogQueue == NULL)
  {
      Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate(CommandTask,"CommandTask",512,NULL,2,&commandTaskHandle);
  xTaskCreate(SensorTask, "SensorTask", 512, NULL, 1, &sensorTaskHandle);
  xTaskCreate(SDTask, "SDTask", 768, NULL, 1, &sdTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void UART_Send(const char *message)
{
    if ((message == NULL) || (uartMutex == NULL))
    {
        return;
    }

    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        HAL_UART_Transmit(&huart2,(uint8_t *)message,strlen(message),1000);
        xSemaphoreGive(uartMutex);
    }
}


void Config_Get(SystemConfig_t *config)
{
    if ((config == NULL) || (configMutex == NULL))
    {
        return;
    }

    if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE)
    {
        *config = systemConfig;
        xSemaphoreGive(configMutex);
    }
}


void Config_SetLogging(uint8_t enabled)
{
    if (configMutex == NULL)
    {
        return;
    }

    if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE)
    {
        systemConfig.loggingEnabled = enabled;
        xSemaphoreGive(configMutex);
    }
}


void Config_SetRate(uint32_t rate)
{
    if (configMutex == NULL)
    {
        return;
    }

    if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE)
    {
        systemConfig.sampleRateMs = rate;
        xSemaphoreGive(configMutex);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartTypingActive=1;
	BaseType_t higherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART2)
    {
        if((uartRxByte=='\r') || (uartRxByte=='\n')) {
        	if( (uartRxIndex>0) && (uartRxOverflow==0))  {
        		uartRxBuffer[uartRxIndex]='\0';
        		strcpy(commandBuffer,uartRxBuffer);

        		if(commandTaskHandle!=NULL)	{
        			vTaskNotifyGiveFromISR(commandTaskHandle, &higherPriorityTaskWoken);
        		}
        	}
        	uartRxIndex=0;
			uartRxOverflow = 0;
        } else {
        	if(uartRxIndex < (COMMAND_BUFFER_SIZE-1)) {
        		uartRxBuffer[uartRxIndex]=(char)uartRxByte;
        		uartRxIndex++;
        	} else {
        		 /* Buffer is full; ignore characters until Enter */
        		 uartRxOverflow = 1;
        	}
        }

        HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void CommandTask(void *argument)
{
	char response[120];
    (void)argument;
    snprintf(response, sizeof(response), "\r\nAvailable commands:\r\nSTART\r\nSTOP\r\nRATE <milliseconds>\r\nMinimum rate: 100 ms\r\n\r\n");
    UART_Send(response);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (strcmp(commandBuffer, "START") == 0)
        {
        	Config_SetLogging(1U);
            snprintf(response, sizeof(response),"Logging started\r\n");
        }
        else if (strcmp(commandBuffer, "STOP") == 0)
        {
        	Config_SetLogging(0U);
            snprintf(response, sizeof(response),"Logging stopped\r\n");
        }
        else if (strncmp(commandBuffer, "RATE ", 5) == 0)
        {
        	uint8_t validNumber = 1; //DEFAULT:VALID
        	for (uint32_t i = 5; commandBuffer[i] != '\0'; i++)
        	    {
        	        if ((commandBuffer[i] < '0') ||(commandBuffer[i] > '9')) {
        	            validNumber = 0; //INVALID
        	            break;
        	        }
        	    }
			if ((validNumber == 1) && (commandBuffer[5] != '\0')) {
				   uint32_t newRate =(uint32_t)atoi(&commandBuffer[5]);

			   if (newRate >= 100) {
				   Config_SetRate(newRate);
				   snprintf(response, sizeof(response),"Sample rate set to %lu ms\r\n",(unsigned long)newRate);
			   }
			   else
			   {
				   snprintf(response, sizeof(response),"Invalid rate: minimum is 100 ms\r\n");
			   }
		    }
			else
			{
			    snprintf(response, sizeof(response),"Invalid RATE command\r\nExample: RATE 1000\r\n");
			}
        }
        else
        {
            snprintf(response, sizeof(response), "Invalid command\r\nUse: START, STOP, or RATE <milliseconds>\r\n");
        }
        UART_Send(response);
        uartTypingActive = 0; //NOT ACTIVE
    }
}

void SensorTask(void *argument)
{
    (void)argument;

    struct bme280_data sensorData;
    SensorLogData_t logData;
    SystemConfig_t localConfig;

    char output[160];
    int8_t result;

    uint32_t sampleCounter = 0U;

    while (1)
    {
    	Config_Get(&localConfig);

        if ((localConfig.loggingEnabled == 1U) &&(uartTypingActive == 0U) && (bmeInitStatus == BME280_OK))
        {
            result = BME280_App_Read(&sensorData);

            if (result == BME280_OK)
            {
                sampleCounter++;

                logData.sampleNumber = sampleCounter;
                logData.timestampMs = HAL_GetTick();
                logData.temperatureC =(float)sensorData.temperature;
                logData.pressureHpa =(float)(sensorData.pressure / 100.0);
                logData.humidityPercent =(float)sensorData.humidity;

                snprintf(output,sizeof(output),"Sample %lu: %.2f C, %.2f hPa, %.2f %%\r\n",(unsigned long)logData.sampleNumber,
                         logData.temperatureC,logData.pressureHpa,logData.humidityPercent);

                UART_Send(output);

                if (sensorLogQueue != NULL)
                {
                    xQueueSend(sensorLogQueue,&logData,pdMS_TO_TICKS(100));
                }

                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            }
            else
            {
                snprintf(output,sizeof(output),"BME280 read failed: %d\r\n",result);
                UART_Send(output);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(localConfig.sampleRateMs));
    }
}

void SDTask(void *argument)
{
    (void)argument;

    FRESULT result;
    FIL file;
    UINT bytesWritten;

    SensorLogData_t logData;

    char csvLine[160];
    char message[100];

    const char csvHeader[] ="sample,uptime_ms,temperature_c,pressure_hpa,humidity_percent\r\n";

    vTaskDelay(pdMS_TO_TICKS(500));

    result = f_mount(&USERFatFS, USERPath, 1);

    if (result != FR_OK)
    {
        snprintf(message,sizeof(message),"SD mount failed: %d\r\n",result);
        UART_Send(message);

        vTaskDelete(NULL);
    }

    result = f_open(&file,"0:/sensor.csv",FA_OPEN_ALWAYS | FA_WRITE);

    if (result != FR_OK)
    {
        snprintf(message,sizeof(message),"CSV open failed: %d\r\n",result);
        UART_Send(message);

        vTaskDelete(NULL);
    }

    /* Write header only when the file is empty */
    if (f_size(&file) == 0U)
    {
        result = f_write(&file,csvHeader,strlen(csvHeader),&bytesWritten);

        if ((result != FR_OK) ||
            (bytesWritten != strlen(csvHeader)))
        {
            const char errorMessage[] ="CSV header write failed\r\n";

            UART_Send(errorMessage);
        }
    }

    /* Move to end so new samples are appended */
    result = f_lseek(&file, f_size(&file));

    if (result != FR_OK)
    {
        snprintf(message,sizeof(message),"CSV seek failed: %d\r\n",result);

        UART_Send(message);

        f_close(&file);
        vTaskDelete(NULL);
    }

    f_sync(&file);

    const char readyMessage[] = "SD CSV logger ready\r\n";
    UART_Send(readyMessage);

    while (1)
    {
        if (xQueueReceive(sensorLogQueue,&logData,portMAX_DELAY) == pdPASS)
        {
            int lineLength = snprintf(csvLine,sizeof(csvLine),"%lu,%lu,%.2f,%.2f,%.2f\r\n",
                    (unsigned long)logData.sampleNumber,(unsigned long)logData.timestampMs,
                                   logData.temperatureC,logData.pressureHpa,logData.humidityPercent);

            bytesWritten = 0U;

            result = f_write(&file,csvLine,(UINT)lineLength,&bytesWritten);

            if ((result == FR_OK) && (bytesWritten == (UINT)lineLength))
            {
                /* Save every sample safely to the card */
                f_sync(&file);
            }
            else
            {
                snprintf(message, sizeof(message),"SD log write failed: %d\r\n",result);
                UART_Send(message);
            }
        }
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
