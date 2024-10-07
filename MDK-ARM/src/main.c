/**
  ******************************************************************************
  * @file    MEMS_Example/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
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
#define ABS(x)                     (x < 0) ? (-x) : x

#if 0
#define L3G_Sensitivity_250dps     (float)114.285f        /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_500dps     (float)57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps]  */
#define L3G_Sensitivity_2000dps    (float)14.285f         /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#endif	
    
uint8_t lcd_period_flag = 0;
uint8_t exti_int2_flag = 0;
uint8_t calib_flag = 0;
uint32_t frame_cnt = 0;

/* Private variables ---------------------------------------------------------*/
float Buffer[6];
float Gyro[3];
float X_BiasError, Y_BiasError, Z_BiasError = 0.0;
uint8_t Xval, Yval = 0x00;

// tmply
#define CACHE_DEPTH     100
float cache[CACHE_DEPTH];

float phi_integrated = 0.0;
float omega_z = 0.0;
extern float omega_z_bias;

volatile uint8_t main_sts; 
volatile uint8_t fifo_sts; 

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
static void Demo_GyroReadAngRate (float* pfData);
static void Gyro_SimpleCalibration(float* GyroData);



/**
* @brief  
* @param  None
* @retval None
*/
static float StabilPhiCalc(void)
{
    // tmply
    float phi = (360.0/LCD_ILI9341_FPS_INT)*MATH_PI/180.0;
    
    return phi;
}

/**
* @brief  
* @param  dst active zone image address, src bmp image address
* @retval None
*/
static void InitActiveZone(uint8_t *dst_image, uint8_t *bmp_image)
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
            else *(dst_image + i*ACTIVE_WIDTH + j) = 255;
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
    
	int32_t i,j;
    
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
            
            // new displacements
            vec_2_d vec_dst = VectorSimpleRotation(vec_src, cos_phi, sin_phi);
            
            // their fraction parts
            double i_int, j_int;
            float i_frac = modf(vec_dst.i1, &i_int);
            float j_frac = modf(vec_dst.i2, &j_int);
            
            uint32_t ii = (uint32_t)i_int; 
            uint32_t jj = (uint32_t)j_int; 
            
            
            if ((ii > 0)&&(jj > 0)&&(ii < ACTIVE_HEIGHT-1)&&(jj < ACTIVE_WIDTH-1)) 
            {
                dst_image[ii*ACTIVE_WIDTH + jj] = px;
                
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

#if 0
/**
* @brief  Reassign (copy) active zone
* @param  dst image address, src image address
* @retval None
*/
static void ReassignActiveZone(uint8_t *dst_image, uint8_t *src_image)
{
    
	uint32_t i,j;
    for (i=0; i<ACTIVE_HEIGHT; i++)
    {
        for (j=0; j<ACTIVE_WIDTH; j++) 
        {
            dst_image[i*ACTIVE_WIDTH + j] = src_image[i*ACTIVE_WIDTH + j];
        }
    }
}
#endif

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


#if 0  

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

}

	


/**
* @brief  Configure the Mems to gyroscope application.
* @param  None
* @retval 0 if I3G4250D sensor found
*/
static uint8_t Demo_GyroConfig(void)
{
    I3G4250D_InitTypeDef I3G4250D_InitStructure;
    I3G4250D_FilterConfigTypeDef I3G4250D_FilterStructure;

    // CTRL2: set up HP filter
    I3G4250D_FilterStructure.HighPassFilter_Mode_Selection =I3G4250D_HPM_NORMAL_MODE_RES;
    I3G4250D_FilterStructure.HighPassFilter_CutOff_Frequency = I3G4250D_HPFCF_ODR105_8HZ;
    I3G4250D_FilterConfig(&I3G4250D_FilterStructure);

    // CTRL3: ints DRDY
    I3G4250D_INT2InterruptCmd(ENABLE);

       // 
    I3G4250D_InitStructure.Power_Mode = I3G4250D_MODE_ACTIVE;
    I3G4250D_InitStructure.Output_DataRate = I3G4250D_OUTPUT_DATARATE_105HZ;
    I3G4250D_InitStructure.Axes_Enable = I3G4250D_AXES_ENABLE;
    I3G4250D_InitStructure.Band_Width = I3G4250D_ODR105_BANDWIDTH_25HZ;
    I3G4250D_InitStructure.BlockData_Update = I3G4250D_BlockDataUpdate_Continous;
    I3G4250D_InitStructure.Endianness = I3G4250D_BLE_LSB;
    I3G4250D_InitStructure.Full_Scale = I3G4250D_FULLSCALE_245; 
    I3G4250D_Init(&I3G4250D_InitStructure);
    
    // CTRL5: 
    I3G4250D_FilterCmd(I3G4250D_HIGHPASSFILTER_ENABLE);

    // turn on FIFO
    I3G4250D_FIFOEnaCmd(ENABLE);
    
    /* Read WHOAMI register */
    uint8_t tmpreg;
    I3G4250D_Read(&tmpreg, I3G4250D_WHO_AM_I_ADDR, 1);
    if (tmpreg == I_AM_I3G4250D) return 0; else return tmpreg;

}

#endif
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
  
  I3G4250D_Read(&tmpreg,I3G4250D_CTRL_REG4_ADDR,1);
  
  I3G4250D_Read(tmpbuffer,I3G4250D_OUT_X_L_ADDR,6);
  
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
    sensitivity=L3G_Sensitivity_245dps;
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
uint32_t I3G4250D_TIMEOUT_UserCallback(void)
{
  return 0;
}

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
	LCD_SetColors(LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
	
    uint8_t str[15];
    const uint8_t blank[15] = "               ";
    
   
   
    // > 10 ms since poweron
    Delay(10000);
   
    /* Gyroscope configuration */
    I3G4250D_Init();

    // > 250 ms since poweron
    Delay(300000);    
    
    uint8_t id;
    I3G4250D_Read(&id, I3G4250D_WHO_AM_I_ADDR, 1);
    if (id == I_AM_I3G4250D)
    {
        LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)"Sensor:I3G4250D");
    }
    else
    {
        sprintf((char*)str, "Sensor ID=0x%X", id);
        LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)str);
    }

    // tmply
    uint8_t sts = I3G4250D_GetDataStatus();
    uint8_t fifo = I3G4250D_GetFIFOStatus();
    sprintf((char*)str, "sts=0x%X", sts);
    LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)str);
    sprintf((char*)str, "fifo=0x%X", fifo);
    LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)str);

   
  
    /* Gyroscope calibration */
    //Gyro_SimpleCalibration(Gyro);
    
    /* Disable all interrupts for a while */  
    //I3G4250D_INT1InterruptCmd(DISABLE);
    //I3G4250D_INT2InterruptCmd(ENABLE);
    
    // configure INT2/DRDY
    //I3G4250D_INT2InterruptConfig();
    
    
    
    
    
    
    /* Wait user button to be pressed */
    while(STM_EVAL_PBGetState(BUTTON_USER) != RESET)
    {}
    while(STM_EVAL_PBGetState(BUTTON_USER) != SET)
    {}

        
	LCD_Clear(LCD_COLOR_BLACK);

        
    LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)"Calibration...");
    calib_flag = 1;
    Delay(1000000);            
        
    // ints and FIFO ena
    I3G4250D_INT2_EXTI_Config(); 
    I3G4250D_INT2InterruptCmd(ENABLE);
    I3G4250D_FIFOEnaCmd(ENABLE);
    I3G4250D_SetFIFOMode_WMLevel(I3G4250D_FIFO_MODE_FIFO, I3G4250D_FIFO_WM_LEVEL);

    // clr (if any) pending line
    if(EXTI_GetITStatus(I3G4250D_SPI_INT2_EXTI_LINE) != RESET)
    {
        EXTI_ClearITPendingBit(I3G4250D_SPI_INT2_EXTI_LINE);      
    }

        
       
        
	/*
    // BMP read
    uint32_t bmp_width, bmp_height;
    uint32_t bytes = ReadBMP(IMG_BMP_ADDR, (uint8_t*)frame_bmp, &bmp_width, &bmp_height);

    sprintf((char*)str, "BMP read Ok");
  	LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)str);
    sprintf((char*)str, "width=%d", bmp_width);
  	LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)str);
    sprintf((char*)str, "height=%d", bmp_height);
  	LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)str);
        
    Delay(500000);

    InitActiveZone((uint8_t*)frame_cur, (uint8_t*)frame_bmp);
    */
    
    float phi = 0.0;

    // rewind phi
	phi_integrated = 0.0;


    while (1)
    {
        
                
        if (lcd_period_flag)
        {
            
          
            //Demo_MEMS();	

            // rotate
            //RotateActiveZone((uint8_t*)frame_new, (uint8_t*)frame_cur, phi, BACKGR_COLOR);

            // redraw
            //DrawActiveZone((uint8_t*)frame_new, LCD_SIZE_PIXEL_WIDTH/2, LCD_SIZE_PIXEL_HEIGHT/2, BACKGR_COLOR);

            // tmply
            //int8_t temp = I3G4250D_GetTemp();
            //uint8_t sts = I3G4250D_GetDataStatus();
            //uint8_t fifo = I3G4250D_GetFIFOStatus();
            if (!calib_flag)
            {
                LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)blank);
            }
            LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)blank);
            LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)blank);
            LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)blank);
            LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)blank);
            LCD_DisplayStringLine(LCD_LINE_5, (uint8_t*)blank);
            LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)blank);
            
            cache[frame_cnt] = omega_z;
            
            sprintf((char*)str, "F=%04d", frame_cnt);
            LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)str);
            sprintf((char*)str, "omega=%.1f", omega_z);
            LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)str);
            sprintf((char*)str, "bias=%.1f", omega_z_bias);
            LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)str);
            sprintf((char*)str, "phi=%.1f", phi_integrated);
            LCD_DisplayStringLine(LCD_LINE_4, (uint8_t*)str);
            
            sprintf((char*)str, "sts=0x%X", main_sts);
            LCD_DisplayStringLine(LCD_LINE_5, (uint8_t*)str);
            sprintf((char*)str, "fifo=0x%X", fifo_sts);
            LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)str);
            

    
            // integrate calc phi
            phi += StabilPhiCalc();
            
        
            // reassign
            //ReassignActiveZone((uint8_t*)frame_cur, (uint8_t*)frame_new);
            
			frame_cnt++;
			if (frame_cnt == LCD_ILI9341_FPS_INT + 1) 
            {
                frame_cnt = 0;
            }   
			lcd_period_flag = 0;
		}
        
        
        // 
        /*
        if (!exti_int2_flag && (calib_cnt == I3G4250D_CALIB_SAMPLES))
        {
            // calc bias
            omega_z_bias = calib_sum/(float)I3G4250D_CALIB_SAMPLES;
            calib_flag = 0; 
        }
        */
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
