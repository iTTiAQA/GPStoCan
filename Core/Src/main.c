/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
  * shaoqi 2024.8.21 锟斤拷锟斤拷说锟斤拷
  * 锟斤拷锟侥匡拷模锟絀nterface 锟斤拷频锟侥匡拷锟斤拷锟轿拷锟斤拷锟斤拷惴斤拷锟斤拷炭锟斤拷倏刹锟斤拷
  * 实锟斤拷原锟斤拷锟斤拷锟斤拷锟斤拷锟教诧拷片锟斤拷锟截碉拷锟脚猴拷通锟斤拷锟斤拷锟斤拷锟斤拷锟酵碉拷锟斤拷示锟斤拷锟斤拷锟接讹拷锟斤拷锟斤拷锟斤拷示锟斤拷锟斤拷示锟斤拷锟斤拷
  * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "atk_mw1278d.h"
#include "atk_mw1278d_uart.h"
#include "demo.h"
#include"delay.h"
#include "Locate.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
//HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALSE 0
#define TRUE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int speed = 999;      //锟斤拷锟斤拷锟劫讹拷
int SOC = 100;        //锟斤拷氐锟斤拷锟斤拷
int Mode_Index = 0;   //锟斤拷驶模式锟斤拷锟斤拷
int P_FL=0;           //锟斤拷前锟斤拷锟斤拷
int P_FR=0;           //锟斤拷前锟斤拷锟斤拷
int P_RL=0;           //锟斤拷锟斤拷锟�
int P_RR=0;           //锟揭猴拷锟斤拷
int Sum_Voltage=0;           //锟杰碉拷压
int Top_Temperature=0;           //锟斤拷锟斤拷锟斤拷锟斤拷露锟�
int Sum_I=0;
extern uint8_t UART2_RX_Buffer[50];
//extern uint8_t UART1_RX_Buffer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int flag=0;
uint8_t Rxbuf[10];
uint8_t RxFlag = 0;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  CAN_Filter_Config_TXOnly(); // 只发不收过滤器配置
  HAL_CAN_Start(&hcan);       // 启动 can
  //HAL_UART_Receive_IT(&huart2, (uint8_t *)Rxbuf, 10);    // 使锟杰达拷锟节斤拷锟斤拷锟叫讹拷
  //wireless sent settings
  sys_stm32_clock_init(RCC_PLL_MUL9); /* 设置时钟, 72Mhz */
  delay_init(72);                     /* 延时初始化 */
  atk_mw1278d_init(115200);
  // usart_init(115200);                 /* 串口初始化为115200 */
  // demo_run();                         /* 运行示例程序 */
  HAL_Delay(1000);

  /* GNSS 模块配置握手 */
  {
      // const char* cmds[] = {"GNGGA 0.1\r\n", "GNRMC 0.1\r\n"};
      const char* cmds[] = {
        "GNRMC 0.1\r\n", 
        "CONFIG HEADING VARIABLELENGTH\r\n", 
        "MODE ROVER AUTOMOTIVE DEFAULT\r\n"
      };
      uint8_t gnss_ok = 1;

      //for(int cmd_idx = 0; cmd_idx < 2; cmd_idx++)
      for(int cmd_idx = 0; cmd_idx < 3; cmd_idx++)
      {
          uint8_t ack = 0;
          for(int retry = 0; retry < GNSS_CMD_RETRY; retry++)
          {
              uart2_got_ok = 0;
              HAL_UART_Transmit(&huart2, (uint8_t*)cmds[cmd_idx], strlen(cmds[cmd_idx]), HAL_MAX_DELAY);
              /* delay_init 已关闭SysTick中断，HAL_GetTick()失效，改用delay_ms()分次轮询 */
              for(int wait = 0; wait < GNSS_CMD_TIMEOUT / 100; wait++)
              {
                  delay_ms(100);
                  if(uart2_got_ok){ack = 1; break;}
              }
              if(ack) break;
          }
          if(!ack){gnss_ok = 0; break;}
      }
      if(!gnss_ok){atk_mw1278d_uart_printf("GNSS_Error\r\n");}
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // shaoqi
  // 锟斤拷锟斤拷锟竭硷拷:锟斤拷锟斤拷锟斤拷锟斤拷锟酵拷锟斤拷锟斤拷锟斤拷锟斤拷锟�"0"锟斤拷
  // 锟斤拷锟斤拷锟斤拷锟絢ey1锟斤拷锟斤拷"1"
  // 锟斤拷锟斤拷锟斤拷锟絢ey2锟斤拷锟斤拷"2"

  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

      /* GNSS 新数据打印(在主循环中执行，不阻塞中断) */
      if(uart2_data_ready)
      {
          uart2_data_ready = 0;
          GNSS_Data_t GNSS_Data;
          get_gnss_data(&GNSS_Data);
          send_gnss_data_uart1(&GNSS_Data);

          /* 通过 CAN 发送 GNSS 速度信息 (ID:0x301, km/h*10 -> uint16) */
          /* 通过 CAN 发送 GNSS 速度信息 (ID:0x301, 1:1无缩放 -> uint16) */
          uint8_t can_data[2] = {0}; // DBC中定义该报文长度为2字节，用8或2都可以，但2更严谨

          // 保留两位小数：乘以100后再取整，左移两位小数
          uint16_t speed_val = (uint16_t)(GNSS_Data.speed_kmh * 100.0f + 0.5f); 

          // Intel 格式（小端模式）拆分：低字节在前，高字节在后
          can_data[0] = speed_val & 0xFF;         // 低 8 位
          can_data[1] = (speed_val >> 8) & 0xFF;  // 高 8 位
          User_CAN_Send_sq(0x41, can_data);

          /* 通过 CAN 发送 GNSS 经纬度信息
           * ID:0x42 -> 纬度 (int64, 度*10^8, 小端, 8字节)
           * ID:0x43 -> 经度 (int64, 度*10^8, 小端, 8字节)
           */
          {
              uint8_t can_lat[8] = {0}, can_lon[8] = {0};
              int64_t lat_val = (int64_t)(GNSS_Data.latitude * 100000000.0);
              int64_t lon_val = (int64_t)(GNSS_Data.longitude * 100000000.0);

              can_lat[0] = (uint8_t)( lat_val        & 0xFF);
              can_lat[1] = (uint8_t)((lat_val >> 8)  & 0xFF);
              can_lat[2] = (uint8_t)((lat_val >> 16) & 0xFF);
              can_lat[3] = (uint8_t)((lat_val >> 24) & 0xFF);
              can_lat[4] = (uint8_t)((lat_val >> 32) & 0xFF);
              can_lat[5] = (uint8_t)((lat_val >> 40) & 0xFF);
              can_lat[6] = (uint8_t)((lat_val >> 48) & 0xFF);
              can_lat[7] = (uint8_t)((lat_val >> 56) & 0xFF);

              can_lon[0] = (uint8_t)( lon_val        & 0xFF);
              can_lon[1] = (uint8_t)((lon_val >> 8)  & 0xFF);
              can_lon[2] = (uint8_t)((lon_val >> 16) & 0xFF);
              can_lon[3] = (uint8_t)((lon_val >> 24) & 0xFF);
              can_lon[4] = (uint8_t)((lon_val >> 32) & 0xFF);
              can_lon[5] = (uint8_t)((lon_val >> 40) & 0xFF);
              can_lon[6] = (uint8_t)((lon_val >> 48) & 0xFF);
              can_lon[7] = (uint8_t)((lon_val >> 56) & 0xFF);

              User_CAN_Send_sq(0x42, can_lat);
              User_CAN_Send_sq(0x43, can_lon);
          }
      }

 //锟饺达拷,锟斤拷止锟斤拷锟斤拷锟斤拷锟斤拷jly)
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(173);
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
//int fputc(int ch, FILE*f)
//{
//	uint8_t temp[1] = {ch};
//	//锟斤拷询锟斤拷锟斤拷1锟街斤拷锟斤拷锟斤拷
//	HAL_UART_Transmit(&huart1,temp,1,2);
//	return ch;
//}
//int fgetc(FILE *f)
//{
//	uint8_t ch;
//	// 锟斤拷锟斤拷锟斤拷询锟斤拷式锟斤拷锟斤拷 1锟街斤拷锟斤拷锟捷ｏ拷锟斤拷时时锟斤拷锟斤拷锟斤拷为锟斤拷锟睫等达拷
//	HAL_UART_Receive( &huart1,(uint8_t*)&ch,1, HAL_MAX_DELAY );
//
//	return ch;
//}

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
