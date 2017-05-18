/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "can.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "stm32f4xx_it.h"
#include "..\source\UAVCAN\uavcan.h"
#include "..\source\ADIS16488A\ADIS16488A.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

 SPI_TypeDef  spi1;
 
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void Delay_n(void);
void Delay_ms(void);
void Delay_us(uint32_t time);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int GAValid, BARValid, MAGNValid;
int DELTANGValid,DELTVELValid; 

/* USER CODE END 0 */

int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Configure the system clock */
  SystemClock_Config();

 /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_CAN1_Init();
  MX_CAN2_Init();
  //MX_CRC_Init();
  //MX_IWDG_Init();
  //MX_RNG_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  //MX_TIM6_Init();
  //MX_TIM7_Init();
  //MX_WWDG_Init();
  Delay_n();
  /* USER CODE BEGIN 2 */
  //adis16480_initial_setup();
	uint32_t i;
	//TestCAN();
  /* USER CODE END 2 */
  uint32_t gyro_id = 0x00000080; 
	uint32_t magn_id = 0x00000082;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
  /* USER CODE END WHILE */
   
		//readgyro();
		//TestCAN(gyro_id,x_gyro,y_gyro,z_gyro);
		//Delay_n();
		//readmagn();
		//TestCAN(magn_id,x_magn,y_magn,z_magn);
		//Delay_n();
		
		//readaccl();
		//TestCAN(x_accl,y_accl,z_accl);
		//GPIO_ResetBits(GPIOC,GPIO_Pin_8);
    //Delay_n();
	}

}



void  Delay_n(void)
{
  int i,j;
	for(i=0;i<20000;i++)
	{
		for(j=0;j<10000;j++)
		{
		}
	}
}
void Delay_ms(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))
  {
    while (1);
  }
}

void Delay_us(uint32_t time)
{   
	  uint32_t TimingDelay;
    if(SysTick_Config(SystemCoreClock/1000000))
    {
      while(1);
    } 
   TimingDelay = time; 
   while(TimingDelay != 0);
}
/** System Clock Configuration 
*/ 
void SystemClock_Config(void) 
{ 
    
 ErrorStatus HSEStartUpStatus; 
 uint32_t        PLL_M = 24;       
 uint32_t        PLL_N = 336; 
 uint32_t        PLL_P = 2; 
 uint32_t        PLL_Q = 7; 
 RCC_DeInit(); 
 RCC_HSEConfig(RCC_HSE_ON);                                     
 HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
 if(HSEStartUpStatus == SUCCESS) 
 { 
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);                             
    RCC_PCLK2Config(RCC_HCLK_Div2); 
    RCC_PCLK1Config(RCC_HCLK_Div4); 
    
    FLASH_SetLatency(FLASH_Latency_5);           
    FLASH_PrefetchBufferCmd(ENABLE); 
    
    RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);     
    RCC_PLLCmd(ENABLE); 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)   
   {   
    
   }     
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
                                            
 } 

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}
/* 
* ADIS16488A中断回调函数，读GYRO/ACCL/MAGN/DELTANG/DELVAL数据 
* 读状态寄存器，根据状态寄存器指示读取MAGN/BAROM
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	unsigned int sta;
	
	readgyro();
	readaccl();
  readdeltang();
  readdeltvel();
	
	GAValid = 0xff;
	sta = adis16480_StatusReg();
	
	if(sta & 0x0200)
	{
	  readbarom(); 
		BARValid = 0xff;
	}
	
	if(sta & 0x0100)
	{
	  readmagn(); 
	  MAGNValid = 0xff;
	}
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
