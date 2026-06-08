/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
extern int speed;   //�����ٶ�
extern int SOC;    //��ص����
extern int P_FL;
extern int P_FR;
extern int P_RL;
extern int P_RR;
extern int Sum_Voltage;
extern int Top_Temperature;
extern int Sum_I;

CAN_TxHeaderTypeDef TxHeader;
uint8_t CAN_TxData[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
uint32_t CAN_TxMail;
uint32_t CAN_ID = 0x102;

CAN_RxHeaderTypeDef RxHeader;
uint8_t CAN_RxData[8] = { 0 };

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// ������յı���ID
//shaoqi_change
uint16_t CAN_RX_MSG_ID[4] = {0x401, 0x501, 0x502, 0x50};

/*
 * @func: ����CAN���˲�
 *
 */
void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef CAN_FilterInitStructure = {
			.FilterActivation = ENABLE,                    //ʹ�ܹ�����
			.FilterBank = 0x00,                            //���������ţ���Χ��0-13
			.FilterFIFOAssignment = CAN_FILTER_FIFO0,      //���Ĵ洢FIFO��ţ�FIFO0
			.FilterIdHigh = CAN_RX_MSG_ID[0]<<5,           //��һ��ID
			.FilterIdLow = CAN_RX_MSG_ID[1]<<5,            //�ڶ���ID
			.FilterMaskIdHigh = CAN_RX_MSG_ID[2]<<5,       //������ID
			.FilterMaskIdLow = CAN_RX_MSG_ID[3]<<5,                     //���ĸ�ID
			.FilterMode = CAN_FILTERMODE_IDLIST,           //ID�б�ģʽ
			.FilterScale = CAN_FILTERSCALE_16BIT,          //16λ��һ�����������������4����ͨ��id
			.SlaveStartFilterBank = 0                      //���ӹ������ֽ��ߣ���can������
	};
	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure);
}

/*
 * @func: CAN 过滤器配置 — 只发不收
 *        掩码全部置0，屏蔽所有ID，FIFO不接收任何报文
 */
void CAN_Filter_Config_TXOnly(void)
{
	CAN_FilterTypeDef CAN_FilterInitStructure = {
		.FilterActivation = ENABLE,
		.FilterBank = 0,
		.FilterFIFOAssignment = CAN_FILTER_FIFO0,
		.FilterIdHigh = 0x0000,
		.FilterIdLow  = 0x0000,
		.FilterMaskIdHigh = 0x0000,  /* 掩码全0 → 不关心任何bit → 不匹配任何ID */
		.FilterMaskIdLow  = 0x0000,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_16BIT,
		.SlaveStartFilterBank = 0
	};
	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure);
}

/*
 * @func: CAN���ķ���[��׼��ʽ������֡]
 */
void User_CAN_Send()
{
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = CAN_ID;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_TxData, &CAN_TxMail);
}

/*
 * @func: CAN���ķ���[��׼��ʽ������֡]
 */
//shaoqi_add
//����ָ��ID������
void User_CAN_Send_sq(uint32_t CAN_ID_NEW,uint8_t* CAN_TxData_NEW)
{
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = CAN_ID_NEW;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_TxData_NEW, &CAN_TxMail);
}

/*
 * @func: CAN���Ľ����ж�[FIFO0]
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, CAN_RxData)!= HAL_OK)
	{
	    // ������
	    Error_Handler();
	}
	printf("ID:0x%X\r\n",RxHeader.StdId);

	//���ݲ�ͬ��ID����ò�ͬ����Ϣ�������뿴���ӵ�CANЭ��
	if(RxHeader.StdId==0x401)
	{
		SOC=(int)CAN_RxData[6];
		Sum_Voltage=CAN_RxData[0]+(int)(CAN_RxData[1])*256;
		Top_Temperature = (int)CAN_RxData[7];
		Sum_I = CAN_RxData[4]+(int)(CAN_RxData[5])*256;
	}
	if(RxHeader.StdId==0x501)
	{
		speed=(int)CAN_RxData[0];
	}
	if(RxHeader.StdId==0x502)
	{
		/*��Ҫ����ת�ټ��㹦��*/
		P_FL=(int)(256*CAN_RxData[3]+CAN_RxData[2])/9549;
		P_FR=(int)(256*CAN_RxData[7]+CAN_RxData[6])/9549;
		P_RL=(int)(256*CAN_RxData[1]+CAN_RxData[0])/9549;
		P_RR=(int)(256*CAN_RxData[5]+CAN_RxData[4])/9549;
	}
	if(RxHeader.StdId==0x50)
	{
//		User_CAN_Send_sq(0x03,CAN_RxData);
		if(CAN_RxData[1]==0x50)
		{
			User_CAN_Send_sq(0x60,CAN_RxData);
		}else if(CAN_RxData[1]==0x51)
		{
			User_CAN_Send_sq(0x61,CAN_RxData);
		}else if(CAN_RxData[1]==0x52){
			User_CAN_Send_sq(0x62,CAN_RxData);
		}else if(CAN_RxData[1]==0x53){
			if(CAN_RxData[2]==0x01)
			{
				User_CAN_Send_sq(0x63,CAN_RxData);
			}else if(CAN_RxData[2]==0x02)
			{
				User_CAN_Send_sq(0x64,CAN_RxData);
			}else if(CAN_RxData[2]==0x03)
			{
				User_CAN_Send_sq(0x65,CAN_RxData);
			}

		}else if(CAN_RxData[1]==0x54){
			User_CAN_Send_sq(0x66,CAN_RxData);
		}
	}
}

/*
 * @func: CAN���Ľ����ж�[FIFO1],���0x50
 */

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	if(HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, CAN_RxData)!= HAL_OK)
//	{
//	    // ������
//	    Error_Handler();
//	}
//	printf("ID:0x%X\r\n",RxHeader.StdId);
//	if(RxHeader.StdId==0x50)
//	{
//		if(CAN_RxData[1]==0x50)
//		{
//			User_CAN_Send_sq(0x60,CAN_RxData);
//		}else if(CAN_RxData[1]==0x51)
//		{
//			User_CAN_Send_sq(0x61,CAN_RxData);
//		}else if(CAN_RxData[1]==0x52){
//			User_CAN_Send_sq(0x62,CAN_RxData);
//		}else if(CAN_RxData[1]==0x53){
//			if(CAN_RxData[2]==0x01)
//			{
//				User_CAN_Send_sq(0x63,CAN_RxData);
//			}else if(CAN_RxData[2]==0x02)
//			{
//				User_CAN_Send_sq(0x64,CAN_RxData);
//			}else if(CAN_RxData[2]==0x03)
//			{
//				User_CAN_Send_sq(0x65,CAN_RxData);
//			}
//
//		}else if(CAN_RxData[1]==0x54){
//			User_CAN_Send_sq(0x66,CAN_RxData);
//		}
//	}
//
//}
/* USER CODE END 1 */
