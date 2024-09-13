/**
  ******************************************************************************
  * @file    MEMS_Example/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
  * @brief   This example shows a simple test of how to use the MEMS sensor(L3GD20) 
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
#define ABS(x)                     (x < 0) ? (-x) : x
#define L3G_Sensitivity_250dps     (float)114.285f        /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_500dps     (float)57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_2000dps    (float)14.285f         /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
	
uint8_t lcd_period_flag = 0;
uint32_t frame_cnt = 0;

/* Private variables ---------------------------------------------------------*/
float Buffer[6];
float Gyro[3];
float X_BiasError, Y_BiasError, Z_BiasError = 0.0;
uint8_t Xval, Yval = 0x00;
static __IO uint32_t TimingDelay;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void Demo_MEMS(void);
static void Demo_GyroConfig(void);
static void Demo_GyroReadAngRate (float* pfData);
static void Gyro_SimpleCalibration(float* GyroData);

uint8_t frame_cur[ACTIVE_WIDTH*ACTIVE_HEIGHT]; 

// original image
uint8_t frame_bmp[BMP_WIDTH*BMP_HEIGHT]; 


/**
* @brief  
* @param  None
* @retval None
*/
static void stabil_calc(void)
{}

/**
* @brief  
* @param  None
* @retval None
*/
static void LCD_InitActiveZone(void)
{
	
    
	int32_t i,j = 0;
  
    
	// fill the pict
	for (i=0; i<ACTIVE_HEIGHT; i++)
    {
        for (j=0; j<ACTIVE_WIDTH; j++) 
        {
            if ((j >= ACTIVE_WIDTH/2 - BMP_WIDTH/2)&&(j < ACTIVE_WIDTH/2 + BMP_WIDTH/2)&&
                (i >= ACTIVE_HEIGHT/2 - BMP_HEIGHT/2)&&(i < ACTIVE_HEIGHT/2 + BMP_HEIGHT/2))
                frame_cur[i*ACTIVE_WIDTH + j] = \
                frame_bmp[(i - ACTIVE_HEIGHT/2 + BMP_HEIGHT/2)*BMP_WIDTH + j - ACTIVE_WIDTH/2 + BMP_WIDTH/2];
            else frame_cur[i*ACTIVE_WIDTH + j] = 128;
        }
    }
    
	    
	

}


/**
* @brief  Read BMP
* @param  source BMP-file address, dst image address, image sizes 
* @retval bytes read
*/
uint32_t LCD_ReadBMP(uint32_t BmpAddress, uint32_t *width, uint32_t *height)
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
    frame_bmp[Address] = *(__IO uint8_t *)BmpAddress;
    
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
void LCD_DrawActiveZone(uint16_t horz_pos, uint16_t vert_pos, uint8_t back_color)
{
    uint32_t Address;
 	uint32_t i, j, src_pixel;
	uint32_t RGB565_pixel;
	uint32_t R_comp, B_comp, G_comp;
 

    // (CurrentLayer == LCD_BACKGROUND_LAYER)
    /* reconfigure layer size in accordance with the picture */
    LTDC_LayerSize(LTDC_Layer1, LCD_SIZE_PIXEL_WIDTH, LCD_SIZE_PIXEL_HEIGHT);
    LTDC_ReloadConfig(LTDC_VBReload);
    // assume 24 BPP
    LTDC_LayerPixelFormat(LTDC_Layer1, LTDC_Pixelformat_RGB565);
	LTDC_ReloadConfig(LTDC_VBReload);
 
    for (i=0; i<LCD_SIZE_PIXEL_HEIGHT; i++)
	{
		for (j=0; j<LCD_SIZE_PIXEL_WIDTH; j++)
		{
			Address = LCD_FRAME_BUFFER + i*LCD_SIZE_PIXEL_WIDTH*2 + j*2;
            
            if ((j >= horz_pos - ACTIVE_WIDTH/2)&&(j < horz_pos + ACTIVE_WIDTH/2)&&
                (i >= vert_pos - ACTIVE_HEIGHT/2)&&(i < vert_pos + ACTIVE_HEIGHT/2))
            {
                // src_pixel = frame_cur[(i - vert_pos + ACTIVE_HEIGHT/2)*ACTIVE_WIDTH + j - horz_pos + ACTIVE_WIDTH/2] >> 3;
            }
            else
            {    
                // src_pixel = back_color >> 3;
			}
            
            ////
            src_pixel = back_color >> 3;
            
			R_comp = src_pixel & 0x1F;
			G_comp = (src_pixel << 1) & 0x3F;
			B_comp = src_pixel & 0x1F;
			RGB565_pixel = (R_comp << 11) | (G_comp << 5) | B_comp;
			
			*(__IO uint8_t*) (Address) = (uint8_t)(RGB565_pixel & 0xFF);
			*(__IO uint8_t*) (Address+1) = (uint8_t)((RGB565_pixel >> 8) & 0xFF);
		}
	}

/*
	
  Red_Value = (0xF800 & CurrentTextColor) >> 11;
  Blue_Value = 0x001F & CurrentTextColor;
  Green_Value = (0x07E0 & CurrentTextColor) >> 5;
*/	
	
	
}


RCC_ClocksTypeDef RCC_Clocks;
/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    //int32_t i,j = 0;
  
	/// fill the pict
	//for (i=0; i<LCD_SIZE_PIXEL_WIDTH*LCD_SIZE_PIXEL_HEIGHT; i++) frame_cur[i] = i % 240;
	
    /* Initialize LEDs and user button on STM32F429I-DISCO board ****************/
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);  
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
  
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
    LCD_Clear(LCD_COLOR_BLACK);
	
	/////
	LCD_SetColors(LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)"LCD ready ***");
    
    /* Gyroscope configuration */
    Demo_GyroConfig();
    
    /* Gyroscope calibration */
    Gyro_SimpleCalibration(Gyro);
    
    /* Enable INT1 interrupt */  
    L3GD20_INT1InterruptCmd(ENABLE);
    
    /* Configure interrupts on all axes */
    L3GD20_INT1InterruptConfig(L3GD20_AXES_INTERRUPT_ENABLE);
    
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);      
    }
    else
    {
        STM_EVAL_LEDOn(LED4);      
    }
    
    /* Wait user button to be pressed */
    while(STM_EVAL_PBGetState(BUTTON_USER) != RESET)
    {}
    while(STM_EVAL_PBGetState(BUTTON_USER) != SET)
    {}
    
    /* Disable INT1 interrupt */  
    L3GD20_INT1InterruptCmd(DISABLE);  
  
	////
	LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)"INT1 dis ***");
	
	LCD_Clear(LCD_COLOR_BLACK);
	
    // BMP read
    uint32_t bmp_width, bmp_height;
    uint32_t bytes = LCD_ReadBMP(IMG_BMP_ADDR, &bmp_width, &bmp_height);

    uint8_t str[20];
    sprintf((char*)str, "bytes=%d", bytes);
  	LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)str);
    sprintf((char*)str, "width=%d", bmp_width);
  	LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)str);
    sprintf((char*)str, "height=%d", bmp_height);
  	LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)str);

    //LCD_InitActiveZone();

    while (1)
    {
        
        if (lcd_period_flag)
        {
            //Demo_MEMS();	
            stabil_calc();
            
            LCD_DrawActiveZone(LCD_SIZE_PIXEL_WIDTH/2, LCD_SIZE_PIXEL_HEIGHT/2, 128);
            
			frame_cnt++;
			if (frame_cnt == LCD_SIZE_PIXEL_WIDTH) frame_cnt = 0;
			lcd_period_flag = 0;
		}
    }
}

/**
* @brief  Mems gyroscope Demo application.
* @param  None
* @retval None
*/
static void Demo_MEMS(void)
{   
	
	uint8_t str[20];
  
  /* Read Gyro Angular data */
  Demo_GyroReadAngRate(Buffer);

  Buffer[0] = (int8_t)Buffer[0] - (int8_t)Gyro[0];
  Buffer[1] = (int8_t)Buffer[1] - (int8_t)Gyro[1];
  
  /* Update autoreload and capture compare registers value*/
  Xval = ABS((int8_t)(Buffer[0]));
  Yval = ABS((int8_t)(Buffer[1])); 
	
/*	
	////
	sprintf((char*)str, "X=%10.2f", Buffer[0]);
	LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)str);
	sprintf((char*)str, "Y=%10.2f", Buffer[1]);
	LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)str);
	sprintf((char*)str, "Z=%10.2f", Buffer[2]);
	LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)str);
*/	
	///
	
	LCD_Refresh_BMP();

#if 0  
  if ( Xval>Yval)
  {
    if ((int16_t)Buffer[0] > 40)
    {
			/* Clear the LCD */
      LCD_Clear(LCD_COLOR_WHITE);
      LCD_SetTextColor(LCD_COLOR_MAGENTA);
      LCD_DrawFullRect(100, 40, 40, 120);
      LCD_FillTriangle(50, 190, 120, 160, 160, 310);
      Delay(50);
    }
    if ((int16_t)Buffer[0] < -40)
    {
      /* Clear the LCD */
      LCD_Clear(LCD_COLOR_WHITE);
      LCD_SetTextColor(LCD_COLOR_RED);
      LCD_DrawFullRect(100, 160, 40, 120);
      LCD_FillTriangle(50, 190, 120, 160, 160, 10);
      Delay(50);
    }
  }
  else
  {
    if ((int16_t)Buffer[1] < -40)
    {
      /* Clear the LCD */
      LCD_Clear(LCD_COLOR_WHITE);
      LCD_SetTextColor(LCD_COLOR_GREEN);
      LCD_DrawFullRect(120, 140, 100, 40);
      LCD_FillTriangle(120, 120, 5, 60, 260, 160);      
      Delay(50);
    }
    if ((int16_t)Buffer[1] > 40)
    {      
      /* Clear the LCD */ 
      LCD_Clear(LCD_COLOR_WHITE);
      LCD_SetTextColor(LCD_COLOR_BLUE);
      LCD_DrawFullRect(20, 140, 100, 40);
      LCD_FillTriangle(120, 120, 235, 60, 260, 160);
      Delay(50);
    } 
  } 
#endif	
	
}

/**
* @brief  Configure the Mems to gyroscope application.
* @param  None
* @retval None
*/
static void Demo_GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
  L3GD20_Init(&L3GD20_InitStructure);
  
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
* @brief  Calculate the angular Data rate Gyroscope.
* @param  pfData : Data out pointer
* @retval None
*/
static void Demo_GyroReadAngRate (float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
  pfData[i]=(float)RawData[i]/sensitivity;
  }
}

/**
* @brief  Calculate offset of the angular Data rate Gyroscope.
* @param  GyroData : Data out pointer
* @retval None
*/
static void Gyro_SimpleCalibration(float* GyroData)
{
  uint32_t BiasErrorSplNbr = 500;
  int i = 0;
  
  for (i = 0; i < BiasErrorSplNbr; i++)
  {
    Demo_GyroReadAngRate(GyroData);
    X_BiasError += GyroData[0];
    Y_BiasError += GyroData[1];
    Z_BiasError += GyroData[2];
  }
  /* Set bias errors */
  X_BiasError /= BiasErrorSplNbr;
  Y_BiasError /= BiasErrorSplNbr;
  Z_BiasError /= BiasErrorSplNbr;
  
  /* Get offset value on X, Y and Z */
  GyroData[0] = X_BiasError;
  GyroData[1] = Y_BiasError;
  GyroData[2] = Z_BiasError;
}


/**
* @brief  Basic management of the timeout situation.
* @param  None.
* @retval None.
*/
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
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
