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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lidar.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART1_BUFFER_SIZE 256  // USART1接收缓冲区大小
#define USART3_BUFFER_SIZE 512  // USART3接收缓冲区大小
#define UART_TRANSMIT_TIMEOUT 500  // UART发送超时时间(ms)，避免长时间阻塞
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t usart1_rx_data;
uint16_t rx_index = 0;
uint8_t rx_buffer[256];
uint8_t uart1_rx_buffer[256];
uint8_t uart3_rx_buffer[512];
// 声明DMA句柄
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  // if (lidar_uart3_init(230400) != HAL_OK) {
  //   Error_Handler();
  // }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // 开启接收中断
  uint8_t rx_byte;
  //HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE | UART_FLAG_TC | UART_FLAG_ORE);
  memset(uart3_rx_buffer, 0, USART3_BUFFER_SIZE);
  /* 这一行函数会同时开启 DMA 搬运和空闲中断检测 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, USART3_BUFFER_SIZE);
  // 禁用DMA半传输中断（仅需要空闲中断，减少不必要的中断触发）
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);


  // 初始化雷达
  LIDAR_Init();

  // 启动USART3接收中断
  //uint8_t rx_byte;
  //HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  printf("===LiDAR System Ready===\r\n");
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, USART1_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint8_t printed_0deg = 0;
static uint8_t printed_90deg = 0;
static uint8_t printed_180deg = 0;
static uint8_t printed_270deg = 0;

if (lidar_data_ready()) {
  // 重置打印标记
  printed_0deg = 0;
  printed_90deg = 0;
  printed_180deg = 0;
  printed_270deg = 0;

  printf("=========================================\r\n");
  printf("LD14P Valid Data (ASCII Format)\r\n");
  printf("=========================================\r\n");

  for(int i = 0; i < 720; i++)
  {
    // 双重筛选：距离>0（有效） + 置信度≥20（可靠）
    if(Dataprocess[i].distance > 0 && Dataprocess[i].confidence >= 20)
    {
      // 0°角度：只打印一次
      if(float_abs(Dataprocess[i].angle-0)<2.0 && printed_0deg == 0)
      {
        printf("0 deg: Distance=%d mm, Confidence=%d\r\n",
               Dataprocess[i].distance,
               Dataprocess[i].confidence);
        printed_0deg = 1;
      }
      // 90°角度：只打印一次
      else if(float_abs(Dataprocess[i].angle-90)<2.0 && printed_90deg == 0)
      {
        printf("90 deg: Distance=%d mm, Confidence=%d\r\n",
               Dataprocess[i].distance,
               Dataprocess[i].confidence);
        printed_90deg = 1;
      }
      // 180°角度：只打印一次
      else if(float_abs(Dataprocess[i].angle-180)<2.0 && printed_180deg == 0)
      {
        printf("180 deg: Distance=%d mm, Confidence=%d\r\n",
               Dataprocess[i].distance,
               Dataprocess[i].confidence);
        printed_180deg = 1;
      }
      // 270°角度：只打印一次
      else if(float_abs(Dataprocess[i].angle-270)<2.0 && printed_270deg == 0)
      {
        printf("270 deg: Distance=%d mm, Confidence=%d\r\n",
               Dataprocess[i].distance,
               Dataprocess[i].confidence);
        printed_270deg = 1;
      }
    }
  }

  // 打印无有效数据的角度
  printf("-----------------------------------------\r\n");
  if(printed_0deg == 0) printf("0 deg: No valid data (Distance=0 or low confidence)\r\n");
  if(printed_90deg == 0) printf("90 deg: No valid data (Distance=0 or low confidence)\r\n");
  if(printed_180deg == 0) printf("180 deg: No valid data (Distance=0 or low confidence)\r\n");
  if(printed_270deg == 0) printf("270 deg: No valid data (Distance=0 or low confidence)\r\n");
  printf("=========================================\r\n\r\n");

  lidar_reset_data_flag();
}


    HAL_Delay(100); // 加延时，避免空转
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // LED心跳
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  //USART1 回显
  if (huart->Instance == USART1)
  {
    if (Size > 0)
    {
      HAL_UART_Transmit(&huart1, uart1_rx_buffer, Size, UART_TRANSMIT_TIMEOUT);
      //重启USART1 DMA+Idle接收（循环接收）
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, USART1_BUFFER_SIZE);
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }

  // ===== USART3 雷达接收 =====
  if (huart->Instance == USART3 && Size > 0)
  {
    //HAL_UART_Transmit(&huart1, uart3_rx_buffer, Size, 100);
    lidar_parse_data(uart3_rx_buffer, Size);
    // 1. 打印提示
    //printf("\r\n=== LiDAR Data (len: %d) ===\r\n", Size);
    // // 2. 将雷达数据转发到USART1（电脑端查看）
    // HAL_UART_Transmit(&huart1, uart3_rx_buffer, Size, UART_TRANSMIT_TIMEOUT);
    // 3. 重启USART3,接收下一批数据 DMA+Idle接收（循环接收雷达数据）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, USART3_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  static uint32_t last_error_time = 0;

  if (HAL_GetTick() - last_error_time > 1000) {
    // USART1错误处理
    if (huart->Instance == USART1)
    {
      //printf("USART1 Error! Restart receive...\r\n");
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, USART1_BUFFER_SIZE);
      __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }

    // USART3错误处理（雷达串口）
    if (huart->Instance == USART3)
    {
      //printf("USART3 (LiDAR) Error! Restart receive...\r\n");
      HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, USART3_BUFFER_SIZE);
      __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
  }
}

// printf重定向（如果还没有）
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
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
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
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
