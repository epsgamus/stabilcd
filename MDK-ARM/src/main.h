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
//#define STABILCD_LCD_VERBOSE

// gyro period, ODR=95HZ
#define GYRO_ODR95_PERIOD_USEC		10526ul

// 278*327@6Mhz
#define LCD_ILI9341_FPS_MAX					66.0022440762985941f
#define LCD_ILI9341_FPS_INT						33
#define LCD_ILI9341_PERIOD_USEC_MIN		15151ul
#define LCD_ILI9341_PERIOD_USEC		30000ul
#define IMG_BMP_ADDR    0x08100000UL

// source BMP sizes
#define  BMP_WIDTH    100
#define  BMP_HEIGHT   100

// rotatable sizes, BMP*sqrt(2)
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
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

static inline vec_2_d VectorSimpleRotation(vec_2_d src, float cos_phi, float sin_phi)
{
    float x = (src.i1 - ACTIVE_HEIGHT/2);
    float y = (src.i2 - ACTIVE_WIDTH/2);
    vec_2_d dst;
    dst.i1 = (cos_phi*x - sin_phi*y) + ACTIVE_HEIGHT/2;
    dst.i2 = (sin_phi*x + cos_phi*y) + ACTIVE_WIDTH/2;
    return dst;
}

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
