/**
  ******************************************************************************
  * @file    MEMS_Example/main.h 
  * @author  MCD Application Team
  * @changed eg
  * @version 
  * @date    
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_i3g4250d.h"
#include "stdio.h"
#include "math.h"


/* Exported constants --------------------------------------------------------*/

// uncmt to use scratch vecs
//#define STABILCD_GYRO_TMPLY_VECTORS

// uncmt to blink with leds
//#define STABILCD_LED_BLINKS

// uncmt to echo msgs to lcd
#define STABILCD_LCD_VERBOSE

// uncmt to get initial sensty val using selfest
//#define STABILCD_USE_SELFTEST

// fullscale selected (245/500/2000)
#define STABILCD_FS_DEG			245


#ifdef STABILCD_USE_SELFTEST
	#define STABILCD_ST_TYPE I3G4250D_CTRL4_ST_NEG
#else
	#define STABILCD_ST_TYPE I3G4250D_CTRL4_ST_DIS
#endif

#if STABILCD_FS_DEG == 245
	#define L3G_SELFTEST_VALUE 					L3G_245dps_ST_VALUE
	#define L3G_SENS_INITIAL	 					L3G_Sensitivity_245dps
	#define I3G4250D_FULLSCALE_RANGE 		I3G4250D_FULLSCALE_245
#elif STABILCD_FS_DEG == 500
	#define L3G_SELFTEST_VALUE 					L3G_500dps_ST_VALUE
	#define L3G_SENS_INITIAL	 					L3G_Sensitivity_500dps
	#define I3G4250D_FULLSCALE_RANGE 		I3G4250D_FULLSCALE_500
#elif STABILCD_FS_DEG == 2000
	#define L3G_SELFTEST_VALUE 					L3G_2000dps_ST_VALUE
	#define L3G_SENS_INITIAL	 					L3G_Sensitivity_2000dps
	#define I3G4250D_FULLSCALE_RANGE 		I3G4250D_FULLSCALE_2000
#else
	#error STABILCD_FS_DEG must be 245, 500 or 2000
#endif

// FIFO depth used, samples of ODR
#define I3G4250D_FIFO_WM_LEVEL    	3

// calib samples
#define I3G4250D_CALIB_SAMPLES      300



/*
// 278*327@6Mhz 
// frame portions for specific fps rate
// total 278
#define LCD_ILI9341_GYRO_HSYNC		9
#define LCD_ILI9341_GYRO_HBP			20
#define LCD_ILI9341_GYRO_HADR			240
#define LCD_ILI9341_GYRO_HFP			9
// total 327
#define LCD_ILI9341_GYRO_VSYNC		1
#define LCD_ILI9341_GYRO_VBP			2
#define LCD_ILI9341_GYRO_VADR			320
#define LCD_ILI9341_GYRO_VFP			4

#define LCD_ILI9341_FPS_MAX					66.0022440762985941f
#define LCD_ILI9341_FPS_INT						33
#define LCD_ILI9341_PERIOD_USEC_MIN		15151ul
*/

#define LCD_ILI9341_PERIOD_USEC		30000ul
#define IMG_BMP_ADDR    0x08100000UL

// source BMP sizes
#define  BMP_WIDTH    100
#define  BMP_HEIGHT   100

// rotatable sizes, > BMP*sqrt(2)
#define  ACTIVE_WIDTH    150
#define  ACTIVE_HEIGHT   150

// back
#define BACKGR_COLOR    255

// math
#define MACHEPS_FLOAT   1e-06
#define MATH_PI	3.141592653589793F
#define NORM_COEFF      2.0F/150.0F
#define MATH_PI_DIV_180 MATH_PI/180.0F
#define RECIP_NORM_COEFF      75.0F
#define BLOOMING_EXCEN_MARGIN  0.1F
#define BLOOMING_LO_THRESH  0.5F - BLOOMING_EXCEN_MARGIN
#define BLOOMING_HI_THRESH  0.5F + BLOOMING_EXCEN_MARGIN

/* Exported types ------------------------------------------------------------*/
typedef struct 
{
	float i1;
	float i2;
} vec_2_d;


/* Exported macro ------------------------------------------------------------*/
#define ABS(x)                     (x < 0) ? (-x) : x
#define MAX(x,y)    (x < y) ? y : x
#define MIN(x,y)    (x < y) ? x : y


/* Exported functions ------------------------------------------------------- */
static inline vec_2_d VectorSimpleRotation(vec_2_d src, float cos_phi, float sin_phi)
{
    float x = (src.i1 - ACTIVE_HEIGHT/2);
    float y = (src.i2 - ACTIVE_WIDTH/2);
    vec_2_d dst;
    dst.i1 = (cos_phi*x - sin_phi*y) + ACTIVE_HEIGHT/2;
    dst.i2 = (sin_phi*x + cos_phi*y) + ACTIVE_WIDTH/2;
    return dst;
}

void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
