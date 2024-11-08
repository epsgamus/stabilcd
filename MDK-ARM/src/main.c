/**
  ******************************************************************************
  * @file    MEMS_Example/main.c 
  * @author  MCD Application Team
  * @changed eg
  * @version 
  * @date    
  * @brief   This example shows a simple test of how to use the MEMS sensor(I3G4250D) 
             mounted on the STM32F429I-DISCO board.
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
#include "main.h"

/** @addtogroup STM32F429I_DISCOVERY_Examples
  * @{
  */

/** @addtogroup MEMS_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

    
volatile uint8_t lcd_period_flag = 0;
volatile uint8_t exti2_flag = 0;
uint8_t calib_flag = 0;

/* Private variables ---------------------------------------------------------*/
float phi_integrated = 0.0;
float omega_z = 0.0;
extern float omega_z_bias;

extern uint32_t systick_cnt;

float sens_coeff = L3G_SENS_INITIAL;


volatile uint8_t main_sts; 
volatile uint8_t fifo_sts; 
uint32_t delta_time_usec;

static __IO uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

// transformed active zone
uint8_t frame_new[ACTIVE_WIDTH*ACTIVE_HEIGHT]; 

// actual active zone
uint8_t frame_cur[ACTIVE_WIDTH*ACTIVE_HEIGHT]; 

// original image
uint8_t frame_bmp[BMP_WIDTH*BMP_HEIGHT]; 


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
  * @brief  Set I3G4250D Initialization.
  * @param  
  *         
  * @retval WHOAMI value
  */
static uint8_t I3G4250D_Init(void)
{  
    // > 10 ms since poweron
    Delay(10000);

		// SPI
    I3G4250D_LowLevel_Init();

    // reset device regs
    // CTRL1
    uint8_t ctrl1 = 0;
    I3G4250D_Read(&ctrl1, I3G4250D_CTRL_REG1_ADDR, 1);
    if (ctrl1 & I3G4250D_CTRL1_PD)
    {
        // if sensor already power on
        // to POWERDOWN intentionally
        uint8_t tmp = 0;
        I3G4250D_Write(&tmp, I3G4250D_CTRL_REG1_ADDR, 1);  
        while (ctrl1 & I3G4250D_CTRL1_PD)
        {
            // poll until PD is 0
            I3G4250D_Read(&ctrl1, I3G4250D_CTRL_REG1_ADDR, 1);
        }
    }
    // fifo and ints disable
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_BYPASS, 0);
    uint8_t ctrl3 = 0;
		I3G4250D_Write(&ctrl3, I3G4250D_CTRL_REG3_ADDR, 1);  		
    I3G4250D_FIFOEnaCmd(DISABLE);
     
    // CTRL2: set up HP filter
    uint8_t cltr2 = 0;
    I3G4250D_Read(&cltr2, I3G4250D_CTRL_REG2_ADDR, 1);
    cltr2 &= 0xC0;
    cltr2 |= (uint8_t) (I3G4250D_HPM_NORMAL_MODE_RES | I3G4250D_HPFCF_ODR105_8HZ);                             
    I3G4250D_Write(&cltr2, I3G4250D_CTRL_REG2_ADDR, 1);

    // CLTR4: selftest initiated
    uint8_t ctrl4 = 0;
    ctrl4 |= (uint8_t) (I3G4250D_BLE_LSB | I3G4250D_FULLSCALE_RANGE | STABILCD_ST_TYPE);
    I3G4250D_Write(&ctrl4, I3G4250D_CTRL_REG4_ADDR, 1);
    
    // CTRL5: HP (and LP filters)
    uint8_t ctrl5 = 0;
    ctrl5 |= (uint8_t) (I3G4250D_CTRL5_HPF_ENA /*| I3G4250D_CTRL3_OUT_SEL1*/);
    I3G4250D_Write(&ctrl5, I3G4250D_CTRL_REG5_ADDR, 1);

    // CTRL1: mode ACTIVE
    ctrl1 = 0;
    ctrl1 |= (uint8_t) (I3G4250D_CTRL1_PD | I3G4250D_OUTPUT_DATARATE_105HZ | \
        I3G4250D_AXES_ENABLE | I3G4250D_ODR105_BANDWIDTH_25HZ);
    I3G4250D_Write(&ctrl1, I3G4250D_CTRL_REG1_ADDR, 1);
		
    // > 250 ms since poweron
		Delay(300000);

		// id
		uint8_t id = 0;
    I3G4250D_Read(&id, I3G4250D_WHO_AM_I_ADDR, 1);
		
#ifdef STABILCD_USE_SELFTEST		
		// read selftest rates
		sens_coeff = 0.0;
		int8_t sens_qty = 0;
		for (uint8_t i=0; i<16; i++)
		{
			while (!(I3G4250D_GetDataStatus() & I3G4250D_STATUS_ZYX_DA));
			uint8_t tmp_lo, tmp_hi;
			I3G4250D_Read(&tmp_lo, I3G4250D_OUT_X_L_ADDR, 1);
			I3G4250D_Read(&tmp_hi, I3G4250D_OUT_X_H_ADDR, 1);
			I3G4250D_Read(&tmp_lo, I3G4250D_OUT_Y_L_ADDR, 1);
			I3G4250D_Read(&tmp_hi, I3G4250D_OUT_Y_H_ADDR, 1);
			I3G4250D_Read(&tmp_lo, I3G4250D_OUT_Z_L_ADDR, 1);
			I3G4250D_Read(&tmp_hi, I3G4250D_OUT_Z_H_ADDR, 1);
			int16_t z_st_raw = (int16_t)(((uint16_t)tmp_hi << 8) | (uint16_t)tmp_lo);
			sens_coeff = (float)z_st_raw/L3G_SELFTEST_VALUE;
		}

    // CLTR4: selftest stopped
    ctrl4 = 0;
    ctrl4 |= (uint8_t) (I3G4250D_BLE_LSB | I3G4250D_FULLSCALE_RANGE);
    I3G4250D_Write(&ctrl4, I3G4250D_CTRL_REG4_ADDR, 1);
		
		
		Delay(100000);
#endif		
		
	
		return id;
}


/**
* @brief  Load active zone with a bitmap
* @param  dst active zone image address, src bmp image address
* @retval None
*/
static void InitActiveZone(uint8_t *dst_image, uint8_t *bmp_image, uint8_t back_color)
{
	uint32_t i,j;
    for (i=0; i<ACTIVE_HEIGHT; i++)
    {
        for (j=0; j<ACTIVE_WIDTH; j++) 
        {
            if ((j >= ACTIVE_WIDTH/2 - BMP_WIDTH/2)&&(j < ACTIVE_WIDTH/2 + BMP_WIDTH/2)&&
                (i >= ACTIVE_HEIGHT/2 - BMP_HEIGHT/2)&&(i < ACTIVE_HEIGHT/2 + BMP_HEIGHT/2))
                *(dst_image + i*ACTIVE_WIDTH + j) = \
                *(bmp_image + (i - ACTIVE_HEIGHT/2 + BMP_HEIGHT/2)*BMP_WIDTH + j - ACTIVE_WIDTH/2 + BMP_WIDTH/2);
            else *(dst_image + i*ACTIVE_WIDTH + j) = back_color;
        }
    }
}


/**
* @brief  Rotate active zone
* @param  dst image address, src image address, rotation angle, back color
* @retval None
*/
static void RotateActiveZone(uint8_t *dst_image, uint8_t *src_image, float phi, uint8_t back_color)
{
    
	uint32_t i,j;

    // clr
    for (i=0; i<ACTIVE_HEIGHT; i++)
    {
        for (j=0; j<ACTIVE_WIDTH; j++) 
        {
            dst_image[i*ACTIVE_WIDTH + j] = back_color;
        }
    }
    
    // fpu burden
    float cos_phi = cos(phi);
    float sin_phi = sin(phi);
    
    for (i=0; i<ACTIVE_HEIGHT; i++)
    {
        for (j=0; j<ACTIVE_WIDTH; j++) 
        {
            uint8_t px = src_image[i*ACTIVE_WIDTH + j];
            
            // skip points of no interest
            if (px == back_color) continue;
                
            vec_2_d vec_src = {(float)i, (float)j};
            
            // find out where another point of interest is after rotation
            vec_2_d vec_dst = VectorSimpleRotation(vec_src, cos_phi, sin_phi);
            
            // extract their fraction parts (modf was time-devouring)
            uint32_t ii = (uint32_t)vec_dst.i1; 
            uint32_t jj = (uint32_t)vec_dst.i2; 
            float i_frac = vec_dst.i1 - (float)ii;
            float j_frac = vec_dst.i2 - (float)jj;
            
            // cond trueish
            if ((ii > 0)&&(jj > 0)&&(ii < ACTIVE_HEIGHT-1)&&(jj < ACTIVE_WIDTH-1)) 
            {
                dst_image[ii*ACTIVE_WIDTH + jj] = px;
                
                // apply edge smoother
                // where to bloom horzly
                if (i_frac < BLOOMING_LO_THRESH) dst_image[(ii - 1)*ACTIVE_WIDTH + jj] = px;
                if (i_frac > BLOOMING_HI_THRESH) dst_image[(ii + 1)*ACTIVE_WIDTH + jj] = px;
                // where to bloom vertly
                if (j_frac < BLOOMING_LO_THRESH) dst_image[ii*ACTIVE_WIDTH + jj - 1] = px;
                if (j_frac > BLOOMING_HI_THRESH) dst_image[ii*ACTIVE_WIDTH + jj + 1] = px;
            }
        }
    }
    
}


/**
* @brief  Read BMP
* @param  source BMP-file address, dst image address, image sizes 
* @retval bytes read
*/
static uint32_t ReadBMP(uint32_t BmpAddress, uint8_t *bmp_image, uint32_t *width, uint32_t *height)
{
  uint32_t index = 0, size = 0, bit_pixel = 0;
  uint32_t currentline = 0, linenumber = 0;
 
  /* Read bitmap size */
  size = *(__IO uint16_t *) (BmpAddress + 2);
  size |= (*(__IO uint16_t *) (BmpAddress + 4)) << 16;
  
  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (BmpAddress + 10);
  index |= (*(__IO uint16_t *) (BmpAddress + 12)) << 16;

  uint32_t raw_width = 0;
  uint32_t raw_height = 0;
    
  /* Read bitmap width */
  raw_width = *(__IO uint16_t *) (BmpAddress + 18);
  raw_width |= (*(__IO uint16_t *) (BmpAddress + 20)) << 16;

  /* Read bitmap height */
  raw_height = *(__IO uint16_t *) (BmpAddress + 22);
  raw_height |= (*(__IO uint16_t *) (BmpAddress + 24)) << 16;

  /* Read bit/pixel */
  bit_pixel = *(__IO uint16_t *) (BmpAddress + 28);  
 
   /* compute the real size of the picture (without the header)) */  
  size = (size - index); 

  /* bypass the bitmap header */
  BmpAddress += index;

  /* start copie image from the bottom */
  uint32_t  Address = raw_width*(raw_height-1)*(bit_pixel/8);
  
  for(index = 0; index < size; index++)
  {
    *(bmp_image + Address) = *(__IO uint8_t *)BmpAddress;
    
    /*jump on next byte */   
    BmpAddress++;
    Address++;
    currentline++;
    
    if((currentline/(bit_pixel/8)) == raw_width)
    {
      if(linenumber < raw_height)
      {
        linenumber++;
        Address -=(2*raw_width*(bit_pixel/8));
        currentline = 0;
      }
    }
  }
  
  *width = raw_width;
  *height = raw_height;
  return index;
}

/**
  * @brief  Refreshes a bitmap picture 
  * @param  
  * @retval None
  */
static void DrawActiveZone(uint8_t *img, uint16_t horz_pos, uint16_t vert_pos, uint8_t back_color)
{
    uint32_t Address;
 	uint32_t i, j, src_pixel;
	uint32_t RGB565_pixel;
	uint32_t R_comp, B_comp, G_comp;

    // FOREGROUND
    LTDC_LayerSize(LTDC_Layer2, LCD_SIZE_PIXEL_WIDTH, LCD_SIZE_PIXEL_HEIGHT);
    LTDC_ReloadConfig(LTDC_VBReload); 
 
	// assume 24 BPP
	LTDC_LayerPixelFormat(LTDC_Layer1, LTDC_Pixelformat_RGB565);
	LTDC_ReloadConfig(LTDC_VBReload);
  
    for (i=0; i<LCD_SIZE_PIXEL_HEIGHT; i++)
	{
		for (j=0; j<LCD_SIZE_PIXEL_WIDTH; j++)
		{
			Address = LCD_FRAME_BUFFER + BUFFER_OFFSET + i*LCD_SIZE_PIXEL_WIDTH*2 + j*2;
            
            if ((j >= horz_pos - ACTIVE_WIDTH/2)&&(j < horz_pos + ACTIVE_WIDTH/2)&&
                (i >= vert_pos - ACTIVE_HEIGHT/2)&&(i < vert_pos + ACTIVE_HEIGHT/2))
            {
                src_pixel = *(img + (i - vert_pos + ACTIVE_HEIGHT/2)*ACTIVE_WIDTH + j - horz_pos + ACTIVE_WIDTH/2) >> 3;
            }
            else
            {    
                src_pixel = back_color >> 3;
			}
         
			R_comp = src_pixel & 0x1F;
			G_comp = (src_pixel << 1) & 0x3F;
			B_comp = src_pixel & 0x1F;
			RGB565_pixel = (R_comp << 11) | (G_comp << 5) | B_comp;
			
			*(__IO uint8_t*) (Address) = (uint8_t)(RGB565_pixel & 0xFF);
			*(__IO uint8_t*) (Address+1) = (uint8_t)((RGB565_pixel >> 8) & 0xFF);
		}
	}
}


/**
* @brief  Basic management of the timeout situation.
* @param  None.
* @retval None.
*/
uint32_t I3G4250D_TIMEOUT_UserCallback(void)
{
  return 0;
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

    /* Initialize LEDs and user button on STM32F429I-DISCO board ****************/
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);  
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 

    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOff(LED4);

  
    /* SysTick end of count event each 1 us */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000000);
  
    /* Initialize the LCD */
    LCD_Init();
    /* Initialize the LCD Layers*/
    LCD_LayerInit();
  
    /* Enable the LTDC */
    LTDC_Cmd(ENABLE);
    
    /* Set LCD Background Layer  */
    LCD_SetLayer(LCD_FOREGROUND_LAYER);
  
    /* Clear the Background Layer */ 
    LCD_Clear(LCD_COLOR_WHITE);
		LCD_SetColors(LCD_COLOR_BLUE, LCD_COLOR_WHITE);
	
    uint8_t str[15];
   
    // BMP read
    uint32_t bmp_width, bmp_height;
    uint32_t bytes = ReadBMP(IMG_BMP_ADDR, (uint8_t*)frame_bmp, &bmp_width, &bmp_height);
    if (bytes == bmp_width*bmp_height)
    {
        sprintf((char*)str, "BMP %dx%d Ok", bmp_width, bmp_height);
    }
    else
    {
        sprintf((char*)str, "BMP error=%d", bytes);
    }
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)str);
    InitActiveZone((uint8_t*)frame_cur, (uint8_t*)frame_bmp, BACKGR_COLOR);
   
    /* Gyroscope configuration */
    uint8_t id = I3G4250D_Init();
    if (id == I_AM_I3G4250D)
    {
        LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)"Sensor:I3G4250D");
    }
    else
    {
        sprintf((char*)str, "Sensor ID=0x%X", id);
        LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)str);
    }

  
    LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)"Press USER...");

    
    /* Wait user button to be pressed */
    while(STM_EVAL_PBGetState(BUTTON_USER) != RESET)
    {}
    while(STM_EVAL_PBGetState(BUTTON_USER) != SET)
    {}

        
		LCD_Clear(LCD_COLOR_WHITE);

        
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)"Calibration...");
    calib_flag = 1;
    Delay(1000000);            
        
    // ints and FIFO ena
    I3G4250D_INT2_EXTI_Config(); 
    I3G4250D_INT2InterruptCmd(ENABLE);
    I3G4250D_FIFOEnaCmd(ENABLE);
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_FIFO, I3G4250D_FIFO_WM_LEVEL-1);

    // clr (if any) pending line
    if(EXTI_GetITStatus(I3G4250D_SPI_INT2_EXTI_LINE) != RESET)
    {
        EXTI_ClearITPendingBit(I3G4250D_SPI_INT2_EXTI_LINE);      
    }


		phi_integrated = 0.0;
    
#ifdef STABILCD_LCD_VERBOSE            
    uint32_t systick_prev = 0;
    uint32_t delta_frame_usec = 0;
#endif
    
    while (1)
    {
        // sync with gyro
        if (exti2_flag)
        {

#ifdef STABILCD_LCD_VERBOSE            
            delta_frame_usec = systick_cnt-systick_prev;
            systick_prev = systick_cnt;
#endif
            
#ifdef STABILCD_LED_BLINKS        
            // tgl LED4
            STM_EVAL_LEDToggle(LED4);
#endif            
            
						
            // rotate
            RotateActiveZone((uint8_t*)frame_new, (uint8_t*)frame_cur, -phi_integrated*MATH_PI_DIV_180, BACKGR_COLOR);

            // redraw
            DrawActiveZone((uint8_t*)frame_new, LCD_SIZE_PIXEL_WIDTH/2, LCD_SIZE_PIXEL_HEIGHT/2, BACKGR_COLOR);
					  
						

            if (calib_flag)
            {
                LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)"Calibration...");
            }
#ifdef STABILCD_LCD_VERBOSE            
           
            
            sprintf((char*)str, "omega=%5.1f", omega_z);
            LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)str);
						/*
            sprintf((char*)str, "phi=%6.1f", phi_integrated);
            LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)str);
            sprintf((char*)str, "deltaT=%5d", delta_time_usec/1000);
            LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)str);
            sprintf((char*)str, "deltaF=%5d", delta_frame_usec/1000);
            LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)str);
            */
            sprintf((char*)str, "deltaT=%5d", delta_time_usec/1000);
            LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)str);
            //sprintf((char*)str, "deltaF=%5d", delta_frame_usec/1000);
            //LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)str);
						

            
            sprintf((char*)str, "sens=%5.1f", sens_coeff);
            LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)str);
						
            sprintf((char*)str, "phi=%6.1f", phi_integrated);
            LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)str);
						/*
            sprintf((char*)str, "sts=0x%X", main_sts);
            LCD_DisplayStringLine(LCD_LINE_5, (uint8_t*)str);
            sprintf((char*)str, "fifo=0x%X", fifo_sts);
            LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)str);
            sprintf((char*)str, "deltaT=%5d", delta_time_usec/1000);
            LCD_DisplayStringLine(LCD_LINE_7, (uint8_t*)str);
            */
#endif
    
    
						exti2_flag = 0;
						
				} // exti2_flag
    }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
