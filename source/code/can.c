/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "can.h"

#include "gpio.h"

#include "ADIS16488A.h"

/* USER CODE BEGIN 0 */
#include <string.h>
static void MX_CAN1_FG(void);
static void MX_CAN2_FG(void);
extern void Error_Handler(void);
extern uint16_t x_gyro, y_gyro, z_gyro;
//extern uint8_t uavcan_node_id;

/* USER CODE END 0 */

//extern CAN_TypeDef hcan1;
//extern CAN_TypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  CAN_InitTypeDef CAN_InitStruct;
  CAN_InitStruct.CAN_Prescaler =5;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_TXFP = DISABLE;
  CAN_Init(CAN1,&CAN_InitStruct);
  MX_CAN1_FG();
}
/* CAN2 init function */   
//CANbps=APB1总线频率42,000,000/6/(8+5+1))=500k bps  
//总体配置方向: Tseg1>=Tseg2  Tseg2>=tq; Tseg2>=2TSJW  
void MX_CAN2_Init(void)
{
  CAN_InitTypeDef CAN_InitStruct;
  CAN_InitStruct.CAN_Prescaler = 3;//6;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_Init(CAN2,&CAN_InitStruct);
  MX_CAN2_FG();

}


/* USER CODE BEGIN 1 */

void tidIncrement(uint8_t* transfer_id)
{
    *transfer_id += 1;
    if (*transfer_id >= 32)
    {
        *transfer_id = 0;
    }
}


/**
 * Sends a broadcast transfer.
 * If the node is in passive mode, only single frame transfers will be allowed.
 * 数据长度不超过8个字节
 */
int canardBroadcast(uint8_t node_id,
                    uint64_t data_type_signature,
                    uint16_t data_type_id,
										uint8_t transfer_type,
                    uint8_t* inout_transfer_id,
                    const void* payload,
                    uint16_t payload_len)
{
    uint32_t can_id;
    uint16_t crc = 0xFFFFU;
		uint8_t frame_index, last_frame;
	
    if (payload == NULL)
    {
        return -1;
    }
		frame_index = 0x00;
		last_frame = 0x80;
		
    can_id = ((uint32_t)data_type_id << 19) | ((uint32_t) transfer_type << 17) | ((uint32_t)node_id << 10) | *inout_transfer_id | 0x80;
	  CanTxMsg   TxMessage;
		TxMessage.ExtId = can_id;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = payload_len;
    memcpy(TxMessage.Data, payload, payload_len);		
		
		CAN_Transmit(CAN2, &TxMessage);
		
    tidIncrement(inout_transfer_id);
    return 1;
}

/*CANfilterinit*/ 
static void MX_CAN1_FG(void)
{
	CAN_FilterInitTypeDef CAN_FCT;
	
	CAN_FCT.CAN_FilterNumber=0;
	CAN_FCT.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FCT.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FCT.CAN_FilterIdHigh=0x0000;
	CAN_FCT.CAN_FilterIdLow=0x0000;//(uint32_t)uavcan_node_id;

	CAN_FCT.CAN_FilterMaskIdHigh=0x0000;
	CAN_FCT.CAN_FilterMaskIdLow=0xffff;
	CAN_FCT.CAN_FilterFIFOAssignment=CAN_FIFO0;
	CAN_FCT.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FCT);

}

/*CANfilterinit*/ 
static void MX_CAN2_FG(void)
{
	CAN_FilterInitTypeDef CAN_FCT;
	
	CAN_FCT.CAN_FilterNumber=14;
	CAN_FCT.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FCT.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FCT.CAN_FilterIdHigh=0x0000;
	CAN_FCT.CAN_FilterIdLow= 0x0000;//(uint32_t)uavcan_node_id;

	CAN_FCT.CAN_FilterMaskIdHigh=0x0000;
	CAN_FCT.CAN_FilterMaskIdLow=0xffff;
	CAN_FCT.CAN_FilterFIFOAssignment=CAN_FIFO1;
	CAN_FCT.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FCT);

}

void TestCAN(uint32_t device_id,uint16_t X_data,uint16_t Y_data,uint16_t Z_data,uint16_t Iddata)
{
	  uint32_t priority = 0x00;
	  uint32_t messagetypeID;
	  uint32_t servicenotmessage = 0x00;
    uint32_t sourcenodeID = 0x01;
	  uint32_t temp;
		CanTxMsg  TxMessage;
	  messagetypeID = device_id;
	  //TxMessage.StdId = std_id;
	  TxMessage.ExtId = ((priority << 24)|(messagetypeID <<8) | (servicenotmessage <<7 )| sourcenodeID );//uavcan 的传输标志
		
		//TxMessage.ExtId = 0x0003E901;
		TxMessage.IDE = CAN_ID_EXT;//CAN_ID_STD;//
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x00000008;
	
    TxMessage.Data[0] = ( X_data >> 0) & 0xFF;		
    TxMessage.Data[1] = ( X_data >> 8) & 0xFF;
    TxMessage.Data[2] = ( Y_data >> 0) & 0xFF;
    TxMessage.Data[3] = ( Y_data >> 8) & 0xFF;
    TxMessage.Data[4] = ( Z_data >> 0) & 0xFF;
    TxMessage.Data[5] = ( Z_data >> 8) & 0xFF;
    TxMessage.Data[6] = 0x00;
    TxMessage.Data[7] = ( Iddata >> 0) & 0xFF;
	
		CAN_Transmit(CAN2,&TxMessage);
	 
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
