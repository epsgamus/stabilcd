/**
  ******************************************************************************
  * @file    MEMS_Example/main.h 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
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
#include "stm32f429i_discovery_l3gd20.h"
#include "stdio.h"


// 278*327@6Mhz
#define LCD_ILI9341_FPS						66.0022440762985941f
#define LCD_ILI9341_PERIOD_USEC		15151ul
#define IMG_BMP_ADDR    0x08100000UL

// source BMP sizes
#define  BMP_WIDTH    100
#define  BMP_HEIGHT   100

// rotatable sizes, BMP*sqrt(2)
#define  ACTIVE_WIDTH    150
#define  ACTIVE_HEIGHT   150

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
//void LCD_DrawActiveZone(uint16_t horz_pos, uint16_t vert_pos, uint8_t back_color);
//uint32_t LCD_ReadBMP(uint32_t BmpAddress, uint8_t *dst_image, uint32_t *width, uint32_t *height);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
