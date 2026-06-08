/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include"stdio.h"
#include"string.h"
#include"Locate.h"
#include "atk_mw1278d.h"
#include "atk_mw1278d_uart.h"

extern uint8_t UART1_RX_Buffer;
uint16_t g_usart_rx_sta = 0;
/* 锟斤拷锟秸伙拷锟斤拷, 锟斤拷锟経SART_REC_LEN锟斤拷锟街斤拷. */
uint8_t g_usart_rx_buf[USART_REC_LEN];
uint8_t g_rx_buffer[RXBUFFERSIZE];  /* HAL锟斤拷使锟矫的达拷锟节斤拷锟秸伙拷锟斤拷 */

/* UART2 (GNSS) 接收缓冲 */
#define UART2_RX_BUF_SIZE  300
uint8_t uart2_rx_byte;
uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];
uint16_t uart2_rx_index = 0;
volatile uint8_t uart2_got_ok = 0;  /* GNSS 配置应答标志 */
volatile uint8_t uart2_data_ready = 0;  /* GNSS 新数据就绪标志(在主循环中打印) */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
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
    /* 启动UART2接收中断(GNSS数据接收) */
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = Wireless_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Wireless_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Wireless_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Wireless_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  // 接收寄存器非空中断
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, Wireless_TX_Pin|Wireless_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void send(char sign, int number)//锟斤拷锟斤拷锟斤拷锟斤拷
{
	char buff[30];
	sprintf(buff,"%s\n",(const char*)number);
	HAL_UART_Transmit(&huart2, (uint8_t*)sign, 1, HAL_MAX_DELAY);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart2, (uint8_t*) buff, strlen(buff), HAL_MAX_DELAY);
	HAL_Delay(5);
//	//wireless communication
	switch(sign){
	case'A':{HAL_UART_Transmit(&huart1,(uint8_t*)"speed:",6, HAL_MAX_DELAY);break;}
	case'B':{HAL_UART_Transmit(&huart1,(uint8_t*)"SOC:",4, HAL_MAX_DELAY);break;}
	case'C':{HAL_UART_Transmit(&huart1,(uint8_t*)"P_FL:",5, HAL_MAX_DELAY);break;}
	case'D':{HAL_UART_Transmit(&huart1,(uint8_t*)"P_FR:",5, HAL_MAX_DELAY);break;}
	case'E':{HAL_UART_Transmit(&huart1,(uint8_t*)"P_RL:",5, HAL_MAX_DELAY);break;}
	case'F':{HAL_UART_Transmit(&huart1,(uint8_t*)"P_RR:",5, HAL_MAX_DELAY);break;}
	case'G':{HAL_UART_Transmit(&huart1,(uint8_t*)"Sum_Voltage:",12, HAL_MAX_DELAY);break;}
	case'H':{HAL_UART_Transmit(&huart1,(uint8_t*)"Top_Temperature:",16, HAL_MAX_DELAY);break;}
	case'I':{HAL_UART_Transmit(&huart1,(uint8_t*)"Sum_I:",6, HAL_MAX_DELAY);break;}
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) buff, strlen(buff), HAL_MAX_DELAY);
	HAL_Delay(20);
}
/**
 * @brief       串口数据接收回调函数
                数据处理在这里进行
 * @param       huart:串口句柄
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//the call-back function is changed for new device ,not for the blue-tooth
{
    /* UART1 (无线模块) 接收处理 */
    if(huart->Instance == USART1)
    {
        if((g_usart_rx_sta & 0x8000) == 0)      /* 锟斤拷锟斤拷未锟斤拷锟� */
        {
            if(g_usart_rx_sta & 0x4000)         /* 锟斤拷锟秸碉拷锟斤拷0x0d */
            {
                if(g_rx_buffer[0] != 0x0a)
                {
                    g_usart_rx_sta = 0;         /* 锟斤拷锟秸达拷锟斤拷,锟斤拷锟铰匡拷始 */
                }
                else
                {
                    g_usart_rx_sta |= 0x8000;   /* 锟斤拷锟斤拷锟斤拷锟斤拷锟� */
                }
            }
            else                                /* 锟斤拷没锟秸碉拷0X0D */
            {
                if(g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0] ;
                    g_usart_rx_sta++;
                    if(g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;     /* 锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷,锟斤拷锟铰匡拷始锟斤拷锟斤拷 */
                    }
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)g_rx_buffer, RXBUFFERSIZE);
    }
    /* UART2 (GNSS) 接收处理 - 接收NMEA语句并解析 */
    else if(huart->Instance == USART2)
    {
        uart2_rx_buf[uart2_rx_index++] = uart2_rx_byte;

        /* 收到换行符(一帧NMEA结束) 或 缓冲区满 */
        if(uart2_rx_byte == '\n' || uart2_rx_index >= UART2_RX_BUF_SIZE)
        {
            /* 将换行符替换为字符串结束符 */
            if(uart2_rx_index > 0 && uart2_rx_buf[uart2_rx_index - 1] == '\n')
            {
                uart2_rx_buf[uart2_rx_index - 1] = '\0';
            }
            else
            {
                uart2_rx_buf[uart2_rx_index] = '\0';
            }

            /* 检测GNSS配置应答 "OK" */
            if(strstr((const char*)uart2_rx_buf, "OK") != NULL)
            {
                uart2_got_ok = 1;
            }

            /* 解析NMEA语句 */
            if(uart2_rx_index > 1)
            {
                parse_gnss_sentence((const char*)uart2_rx_buf, uart2_rx_index);

                    /* 通过UART1打印原始GNGGA/GNRMC语句（添加\r\n换行，避免\r被覆盖） */
                if(strstr((const char*)uart2_rx_buf, "$GNGGA") || strstr((const char*)uart2_rx_buf, "$GNRMC"))
                {
                    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
                    HAL_UART_Transmit(&huart1, (uint8_t*)uart2_rx_buf, strlen((const char*)uart2_rx_buf), 100);
                    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
                }

                uart2_data_ready = 1;  /* 通知主循环处理 */
            }

            uart2_rx_index = 0;
        }

        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    }
}

/**
 * @brief       串口X中断服务函数
                注意,读取USARTx->SR能避免莫名其妙的错误
 * @param       无
 * @retval      无
 */
void USART_UX_IRQHandler(void)
{
#if SYS_SUPPORT_OS                              /* 使锟斤拷OS */
    OSIntEnter();
#endif

    HAL_UART_IRQHandler(&huart1);       /* 锟斤拷锟斤拷HAL锟斤拷锟叫断达拷锟斤拷锟矫猴拷锟斤拷 */
    HAL_UART_IRQHandler(&huart2);       /* UART2 (GNSS) 中断处理 */

#if SYS_SUPPORT_OS                              /* 使锟斤拷OS */
    OSIntExit();
#endif

}

/* 锟斤拷锟斤拷锟斤拷锟铰达拷锟斤拷, 支锟斤拷printf锟斤拷锟斤拷, 锟斤拷锟斤拷锟斤拷要选锟斤拷use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使锟斤拷AC6锟斤拷锟斤拷锟斤拷时 */
__asm(".global __use_no_semihosting\n\t");  /* 锟斤拷锟斤拷锟斤拷使锟矫帮拷锟斤拷锟斤拷模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6锟斤拷锟斤拷要锟斤拷锟斤拷main锟斤拷锟斤拷为锟睫诧拷锟斤拷锟斤拷式锟斤拷锟斤拷锟津部凤拷锟斤拷锟教匡拷锟杰筹拷锟街帮拷锟斤拷锟斤拷模式 */

#else
/* 使锟斤拷AC5锟斤拷锟斤拷锟斤拷时, 要锟斤拷锟斤拷锟斤定锟斤拷__FILE 锟斤拷 锟斤拷使锟矫帮拷锟斤拷锟斤拷模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 锟斤拷使锟矫帮拷锟斤拷锟斤拷模式锟斤拷锟斤拷锟斤拷锟斤拷要锟截讹拷锟斤拷_ttywrch\_sys_exit\_sys_command_string锟斤拷锟斤拷,锟斤拷同时锟斤拷锟斤拷AC6锟斤拷AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 锟斤拷锟斤拷_sys_exit()锟皆憋拷锟斤拷使锟矫帮拷锟斤拷锟斤拷模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 锟斤拷 stdio.h锟斤拷锟芥定锟斤拷. */
FILE __stdout;

/* MDK锟斤拷锟斤拷要锟截讹拷锟斤拷fputc锟斤拷锟斤拷, printf锟斤拷锟斤拷锟斤拷锟秸伙拷通锟斤拷锟斤拷锟斤拷fputc锟斤拷锟斤拷址锟斤拷锟斤拷锟斤拷锟斤拷锟� */
int fputc(int ch, FILE *f)
{
    while ((USART1 ->SR & 0X40) == 0);     /* 锟饺达拷锟斤拷一锟斤拷锟街凤拷锟斤拷锟斤拷锟斤拷锟� *///changed for new device

    USART1 ->DR = (uint8_t)ch;             /* 锟斤拷要锟斤拷锟酵碉拷锟街凤拷 ch 写锟诫到DR锟侥达拷锟斤拷 */
    return ch;
}
#endif
/* USER CODE END 1 */
