/**
  ******************************************************************************
  * @file    CAN/CAN_LoopBack/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "spi.h"
#include "can.h"
#include "..\source\ADIS16488A\ADIS16488A.h"

uint16_t iddata=0xc0;
/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup CAN_LoopBack
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//extern __IO uint32_t ret;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
		
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
	
	
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/
/**
  * @brief This function handles CAN2 TX1 interrupt.
  */ 
void CAN2_TX1_IRQHandler(void)
{

}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */ 
void CAN2_RX1_IRQHandler(void)
{

}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */ 
void CAN2_RX0_IRQHandler(void)
{
  if(CAN_GetITStatus(CAN2,CAN_IT_FMP0))
	{
		  
	}
	CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */ 
void CAN1_RX1_IRQHandler(void)
{

}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */ 
void CAN1_RX0_IRQHandler(void)
{

}
/**
  * @brief This funtion handles TIM3 interrupt.
  */ 
void TIM2_IRQHandler(void)
{
  if( TIM_GetITStatus(TIM2,TIM_IT_Update) == 1)
	{
		uint32_t accl_id = 0x000007D1;//0x00000081;
		uint32_t gyro_id = 0x000007D0;//0x00000080;
		uint32_t magn_id = 0x000007D2;//0x00000082;
		uint32_t deltang_id = 0x00000083;
		uint32_t deltvel_id = 0x00000084;
		//uint16_t iddata;
		adis_read_gyro();//readgyro();//
		TestCAN(gyro_id,x_gyro,y_gyro,z_gyro,iddata);
		
		adis_read_accl();//readaccl();
		TestCAN(accl_id,x_accl,y_accl,z_accl,iddata);
		
		adis_read_magn();//readmagn();
		TestCAN(magn_id,x_magn,y_magn,z_magn,iddata);
		//readdeltang();
		//TestCAN(deltang_id,x_deltang,y_deltang,z_deltang);
		//readdeltvel();
		//TestCAN(deltvel_id,x_deltvel,y_deltvel,z_deltvel);
		iddata++;
		if(iddata > 0xdf)
			iddata = 0xc0;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
/**
  * @brief This funtion handles  interrupt.
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
