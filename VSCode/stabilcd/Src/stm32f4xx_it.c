/**
  ******************************************************************************
  * @file    MEMS_Example/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @changed eg
  * @version 
  * @date    
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

/** @addtogroup STM32F429I_DISCOVERY_Examples
  * @{
  */
    
/** @addtogroup MEMS_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

volatile uint32_t systick_cnt = 0;
uint32_t time_prev_usec = 0;

extern uint8_t lcd_period_flag;
extern uint8_t calib_flag;
extern float phi_integrated;
extern float omega_z;
extern uint32_t delta_time_usec;

extern uint8_t main_sts; 
extern uint8_t fifo_sts; 

float omega_z_bias = 0.0;
float calib_sum = 0.0;
uint32_t calib_cnt = 0;

#ifdef STABILCD_GYRO_TMPLY_VECTORS
float tmp_calib[I3G4250D_CALIB_SAMPLES];
int16_t tmp_calib_int[I3G4250D_CALIB_SAMPLES];
uint32_t tmp_cnt = 0;
#endif


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
    TimingDelay_Decrement();
    systick_cnt++;
    if (systick_cnt % LCD_ILI9341_PERIOD_USEC)	
    {
        // tmply
        lcd_period_flag = 1;
	}
			
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
    I3G4250D_INT2InterruptCmd(DISABLE);
    if(EXTI_GetITStatus(I3G4250D_SPI_INT2_EXTI_LINE) != RESET)
    {
#ifdef STABILCD_LED_BLINKS        
        // tgl LED3
        STM_EVAL_LEDToggle(LED3);
#endif        
        EXTI_ClearITPendingBit(I3G4250D_SPI_INT2_EXTI_LINE);   
    }
    
    // used after calib completion
    delta_time_usec = systick_cnt - time_prev_usec;
    time_prev_usec = systick_cnt;
   
    // read sts
    main_sts = I3G4250D_GetDataStatus();
    fifo_sts = I3G4250D_GetFIFOStatus();
    
    // get actual fifo entries qty
    uint8_t fss = fifo_sts & I3G4250D_FIFO_SRC_FSS_MASK;
    if ((fss == 31)&&(fifo_sts & I3G4250D_FIFO_SRC_OVRN)) fss++;
   
    // readout FIFO head
    uint8_t tmpbuffer[6*32];
    for (uint8_t i=0; i<fss; i++)
    {
        // dummy read
        uint8_t tmp;
        I3G4250D_Read(&tmp, I3G4250D_OUT_X_L_ADDR, 1);
        // read valuables
        // Y
        I3G4250D_Read(tmpbuffer + 6*i + 2, I3G4250D_OUT_Y_L_ADDR, 2);
        // Z
        I3G4250D_Read(tmpbuffer + 6*i + 4, I3G4250D_OUT_Z_L_ADDR, 2);
        // Y
        I3G4250D_Read(tmpbuffer + 6*i, I3G4250D_OUT_X_L_ADDR, 2);
    }
    
  
    // reset FIFO: to BYPASS mode
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_BYPASS, 0);

    // int rate mean by triples
    int32_t sum = 0;
    for (uint8_t i=0; i<I3G4250D_FIFO_WM_LEVEL; i++)
    {
        int16_t tmp = (int16_t)(((uint16_t)tmpbuffer[5+6*i] << 8) | (uint16_t)tmpbuffer[4+6*i]);
        sum += (int32_t)tmp;
#ifdef STABILCD_GYRO_TMPLY_VECTORS        
        // tmply
        if (tmp_cnt < I3G4250D_CALIB_SAMPLES) tmp_calib_int[tmp_cnt++] = tmp;
#endif
    }
    int32_t omega_raw = (int16_t)(sum/I3G4250D_FIFO_WM_LEVEL); 

    // float rate
    omega_z = (float)omega_raw/L3G_Sensitivity_245dps - omega_z_bias;
    
    if (calib_cnt == I3G4250D_CALIB_SAMPLES) 
    {
        // calc bias
        if (calib_flag) omega_z_bias = calib_sum/(float)I3G4250D_CALIB_SAMPLES;
        // clr it
        calib_flag = 0; 
        // integrate
        phi_integrated += (omega_z/1000.0F)*(float)(delta_time_usec/1000);  
    }
    if (calib_flag)
    {
        if (calib_cnt < I3G4250D_CALIB_SAMPLES)
        {
            calib_sum += omega_z;
#ifdef STABILCD_GYRO_TMPLY_VECTORS
            // tmply store
            tmp_calib[calib_cnt] = omega_z;
#endif
            calib_cnt++;
        }
    }   
    
    // reset FIFO: to FIFO mode again
    I3G4250D_INT2InterruptCmd(ENABLE);
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_FIFO, I3G4250D_FIFO_WM_LEVEL-1);
}



/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
