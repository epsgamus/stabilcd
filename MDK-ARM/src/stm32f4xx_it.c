/**
  ******************************************************************************
  * @file    MEMS_Example/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
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
#define MACHEPS_FLOAT   1e-06
#define MAX(x,y)    (x < y) ? y : x
#define MIN(x,y)    (x < y) ? x : y


/* Private variables ---------------------------------------------------------*/

uint8_t pBuffer;
uint32_t systick_cnt = 0;
extern uint8_t lcd_period_flag;
extern uint8_t exti_int2_flag;
extern uint8_t calib_flag;
extern float phi_integrated;
extern float omega_z;

extern uint8_t main_sts; 
extern uint8_t fifo_sts; 

float omega_z_bias = 0.0;
float calib_sum = 0.0;
uint32_t calib_cnt = 0;

volatile float tmp_calib[I3G4250D_CALIB_SAMPLES];
int16_t tmp_calib_int[I3G4250D_CALIB_SAMPLES];
uint32_t tmp_cnt = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static float sgn(float x)
{
    if (x > MACHEPS_FLOAT) return 1.0F;
    if (x < -MACHEPS_FLOAT) return -1.0F;
}



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
    if (systick_cnt == LCD_ILI9341_PERIOD_USEC)	
    {
        lcd_period_flag = 1;
		systick_cnt = 0;
	}
			
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                                               */
/******************************************************************************/

#define GYRO_ODR105_PERIOD_SEC		0.0095238095F


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
        // tgl LED3
        STM_EVAL_LEDToggle(LED3);
        EXTI_ClearITPendingBit(I3G4250D_SPI_INT2_EXTI_LINE);   
        exti_int2_flag = 1;
    }
   
    // read sts
    main_sts = I3G4250D_GetDataStatus();
    fifo_sts = I3G4250D_GetFIFOStatus();
    
    // readout FIFO head
    uint8_t tmpbuffer[6*I3G4250D_FIFO_WM_LEVEL];
    for (uint8_t i=0; i<I3G4250D_FIFO_WM_LEVEL; i++)
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
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_BYPASS, I3G4250D_FIFO_WM_LEVEL);

    int16_t omega_raw = 0;
    
#ifdef  I3G4250D_CALIB_PREFILTER_MEDIAN    
    // median
    uint8_t max_idx = 255;
    int16_t max = -32767;
    int16_t tmp[I3G4250D_FIFO_WM_LEVEL];
    for (uint8_t i=0; i<I3G4250D_FIFO_WM_LEVEL; i++)
    {
        tmp[i] = (int16_t)(((uint16_t)tmpbuffer[5+6*i] << 8) | (uint16_t)tmpbuffer[4+6*i]);
        if (tmp[i] >= max)
        {
            max = tmp[i];
            max_idx = i;
        }
    }
    omega_raw = -32767;
    for (uint8_t i=0; (i<I3G4250D_FIFO_WM_LEVEL)&&(i!=max_idx); i++)
    {
        if (tmp[i] >= omega_raw)
        {
            omega_raw = tmp[i];
        }
    }
#else     
    // mean
    int32_t sum = 0;
    for (uint8_t i=0; i<I3G4250D_FIFO_WM_LEVEL; i++)
    {
        int16_t tmp = (int16_t)(((uint16_t)tmpbuffer[5+6*i] << 8) | (uint16_t)tmpbuffer[4+6*i]);
        sum += (int32_t)tmp;
        // tmply
        if (tmp_cnt < I3G4250D_CALIB_SAMPLES) tmp_calib_int[tmp_cnt++] = tmp;
         
    }
    omega_raw = (int16_t)(sum/I3G4250D_FIFO_WM_LEVEL); 
#endif    
    
    omega_z = (float)omega_raw/L3G_Sensitivity_245dps - omega_z_bias;
    
    if (calib_cnt == I3G4250D_CALIB_SAMPLES) 
    {
        // calc bias
        if (calib_flag) omega_z_bias = calib_sum/(float)I3G4250D_CALIB_SAMPLES;
        // clr it
        calib_flag = 0; 
        // integrate
        phi_integrated += omega_z*GYRO_ODR105_PERIOD_SEC*I3G4250D_FIFO_WM_LEVEL;    
        
    }
    if (calib_flag)
    {
        if (calib_cnt < I3G4250D_CALIB_SAMPLES)
        {
            //if (fabs(omega_z) <= 2.0) calib_sum += omega_z; else calib_sum += 2.0*sgn(omega_z);
            calib_sum += omega_z;
            // tmply store
            tmp_calib[calib_cnt] = omega_z;
            calib_cnt++;
           
        }
    }   
    
    
    // reset FIFO: to FIFO mode again
    I3G4250D_INT2InterruptCmd(ENABLE);
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_FIFO, I3G4250D_FIFO_WM_LEVEL);

    exti_int2_flag = 0;
}


/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
