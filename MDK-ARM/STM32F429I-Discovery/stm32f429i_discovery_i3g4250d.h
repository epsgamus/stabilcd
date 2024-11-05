/**
  ******************************************************************************
  * @file    stm32f429i_discovery_i3g4250d.h
  * @author  MCD Application Team
  * @changed eg  
  * @version 
  * @date    
  * @brief   This file contains definitions for stm32f429i_discovery_i3g4250d.c 
  *          firmware driver.
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
#ifndef __STM32F429I_DISCOVERY_I3G4250D_H
#define __STM32F429I_DISCOVERY_I3G4250D_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup Utilities
   * @{
   */
   
/** @addtogroup STM32F4_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F429I_DISCOVERY
  * @{
  */
  
/** @addtogroup STM32F429I_DISCOVERY_I3G4250D
  * @{
  */
  
/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Exported_Types
  * @{
  */

/* I3G4250D struct */
typedef struct
{
  uint8_t Power_Mode;                         /* Power-down/Sleep/Normal Mode */
  uint8_t Output_DataRate;                    /* OUT data rate */
  uint8_t Axes_Enable;                        /* Axes enable */
  uint8_t Band_Width;                         /* Bandwidth selection */
  uint8_t BlockData_Update;                   /* Block Data Update */
  uint8_t Endianness;                         /* Endian Data selection */
  uint8_t Full_Scale;                         /* Full Scale selection */
}I3G4250D_InitTypeDef;

/* I3G4250D High Pass Filter struct */
typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
}I3G4250D_FilterConfigTypeDef;

/* I3G4250D Interrupt struct */
typedef struct
{
  uint8_t Latch_Request;                      /* Latch interrupt request into CLICK_SRC register */
  uint8_t Interrupt_Axes;                     /* X, Y, Z Axes Interrupts */ 
  uint8_t Interrupt_ActiveEdge;               /*  Interrupt Active edge */
}I3G4250D_InterruptConfigTypeDef;  

/**
  * @}
  */ 


/**
  * @}
  */
  
/** @defgroup STM32F429I-DISCO_I3G4250D_Exported_Constants
  * @{
  */

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/* Uncomment the following line to use the default I3G4250D_TIMEOUT_UserCallback() 
   function implemented in stm32f429i_discovery_lgd20.c file.
   I3G4250D_TIMEOUT_UserCallback() function is called whenever a timeout condition 
   occure during communication (waiting transmit data register empty flag(TXE)
   or waiting receive data register is not empty flag (RXNE)). */   
/* #define USE_DEFAULT_TIMEOUT_CALLBACK */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define I3G4250D_FLAG_TIMEOUT             ((uint32_t)0x1000)

// scale specific const
#define L3G_Sensitivity_245dps     (float)114.285f        /*!< typical sensitivity 8.75 [mdps/lsb]  */
#define L3G_Sensitivity_500dps     (float)57.1429f        /*!< typical sensitivity 17.5 [mdps/lsb]  */
#define L3G_Sensitivity_2000dps    (float)14.285f         /*!< typical sensitivity 70 [mdps/lsb] */

#define L3G_245dps_ST_VALUE     	 (float)130.0f
#define L3G_500dps_ST_VALUE     	 (float)200.0f
#define L3G_2000dps_ST_VALUE     	 (float)530.0f

// FIFO depth used, samples of ODR
#define I3G4250D_FIFO_WM_LEVEL    3

// calib samples
#define I3G4250D_CALIB_SAMPLES      300


/**
  * @brief  I3G4250D SPI Interface pins
  */
#define I3G4250D_SPI                       SPI5
#define I3G4250D_SPI_CLK                   RCC_APB2Periph_SPI5

#define I3G4250D_SPI_SCK_PIN               GPIO_Pin_7                  /* PF.07 */
#define I3G4250D_SPI_SCK_GPIO_PORT         GPIOF                       /* GPIOF */
#define I3G4250D_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOF
#define I3G4250D_SPI_SCK_SOURCE            GPIO_PinSource7
#define I3G4250D_SPI_SCK_AF                GPIO_AF_SPI5

#define I3G4250D_SPI_MISO_PIN              GPIO_Pin_8                  /* PF.08 */
#define I3G4250D_SPI_MISO_GPIO_PORT        GPIOF                       /* GPIOF */
#define I3G4250D_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOF
#define I3G4250D_SPI_MISO_SOURCE           GPIO_PinSource8
#define I3G4250D_SPI_MISO_AF               GPIO_AF_SPI5

#define I3G4250D_SPI_MOSI_PIN              GPIO_Pin_9                  /* PF.09 */
#define I3G4250D_SPI_MOSI_GPIO_PORT        GPIOF                       /* GPIOF */
#define I3G4250D_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOF
#define I3G4250D_SPI_MOSI_SOURCE           GPIO_PinSource9
#define I3G4250D_SPI_MOSI_AF               GPIO_AF_SPI5

#define I3G4250D_SPI_CS_PIN                GPIO_Pin_1                  /* PC.01 */
#define I3G4250D_SPI_CS_GPIO_PORT          GPIOC                       /* GPIOC */
#define I3G4250D_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define I3G4250D_SPI_INT1_PIN              GPIO_Pin_1                  /* PA.01 */
#define I3G4250D_SPI_INT1_GPIO_PORT        GPIOA                       /* GPIOA */
#define I3G4250D_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define I3G4250D_SPI_INT1_EXTI_LINE        EXTI_Line1
#define I3G4250D_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOA
#define I3G4250D_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource1
#define I3G4250D_SPI_INT1_EXTI_IRQn        EXTI1_IRQn 

#define I3G4250D_SPI_INT2_PIN              GPIO_Pin_2                  /* PA.02 */
#define I3G4250D_SPI_INT2_GPIO_PORT        GPIOA                       /* GPIOA */
#define I3G4250D_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define I3G4250D_SPI_INT2_EXTI_LINE        EXTI_Line2
#define I3G4250D_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOA
#define I3G4250D_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource2
#define I3G4250D_SPI_INT2_EXTI_IRQn        EXTI2_IRQn 

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define I3G4250D_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define I3G4250D_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define I3G4250D_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define I3G4250D_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define I3G4250D_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define I3G4250D_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define I3G4250D_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define I3G4250D_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define I3G4250D_STATUS_REG_ADDR        0x27  /* Status register */
#define I3G4250D_OUT_X_L_ADDR           0x28  /* Output Register X */
#define I3G4250D_OUT_X_H_ADDR           0x29  /* Output Register X */
#define I3G4250D_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define I3G4250D_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define I3G4250D_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define I3G4250D_OUT_Z_H_ADDR           0x2D  /* Output Register Z */ 
#define I3G4250D_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define I3G4250D_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define I3G4250D_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define I3G4250D_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define I3G4250D_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define I3G4250D_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define I3G4250D_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define I3G4250D_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define I3G4250D_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define I3G4250D_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define I3G4250D_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

/** @defgroup WHOAMI
  * @{
  */
#define I_AM_I3G4250D		    ((uint8_t)0xD3)
/**
  * @}
  */

/** @defgroup CTRL1 
  * @{
  */   
#define I3G4250D_CTRL1_PD       1 << 3
/**
  * @}
  */

/** @defgroup CTRL1: OutPut_DataRate_Selection 
  * @{
  */
#define I3G4250D_OUTPUT_DATARATE_105HZ     ((uint8_t)0x00)
#define I3G4250D_OUTPUT_DATARATE_208HZ    ((uint8_t)0x40)
#define I3G4250D_OUTPUT_DATARATE_420HZ    ((uint8_t)0x80)
#define I3G4250D_OUTPUT_DATARATE_840HZ    ((uint8_t)0xC0)
/**
  * @}
  */

/** @defgroup CTRL1: Axes_Selection 
  * @{
  */
#define I3G4250D_X_ENABLE            ((uint8_t)0x02)
#define I3G4250D_Y_ENABLE            ((uint8_t)0x01)
#define I3G4250D_Z_ENABLE            ((uint8_t)0x04)
#define I3G4250D_AXES_ENABLE         ((uint8_t)0x07)
#define I3G4250D_AXES_DISABLE        ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup CTRL1: BandWidth_Selection 
  * @{
  */
#define I3G4250D_ODR105_BANDWIDTH_12d5HZ       ((uint8_t)0x00)
#define I3G4250D_ODR105_BANDWIDTH_25HZ         ((uint8_t)0x30)
#define I3G4250D_ODR208_BANDWIDTH_12d5HZ       ((uint8_t)0x00)
#define I3G4250D_ODR208_BANDWIDTH_25HZ         ((uint8_t)0x10)
#define I3G4250D_ODR208_BANDWIDTH_50HZ         ((uint8_t)0x20)
#define I3G4250D_ODR208_BANDWIDTH_70HZ         ((uint8_t)0x30)
#define I3G4250D_ODR420_BANDWIDTH_20HZ         ((uint8_t)0x00)
#define I3G4250D_ODR420_BANDWIDTH_25HZ         ((uint8_t)0x10)
#define I3G4250D_ODR420_BANDWIDTH_50HZ         ((uint8_t)0x20)
#define I3G4250D_ODR420_BANDWIDTH_110HZ        ((uint8_t)0x30)
#define I3G4250D_ODR840_BANDWIDTH_30HZ         ((uint8_t)0x00)
#define I3G4250D_ODR840_BANDWIDTH_35HZ         ((uint8_t)0x10)
#define I3G4250D_ODR840_BANDWIDTH_50HZ         ((uint8_t)0x20)
#define I3G4250D_ODR840_BANDWIDTH_110HZ        ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup CTRL2: High_Pass_Filter_Mode 
  * @{
  */   
#define I3G4250D_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define I3G4250D_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define I3G4250D_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define I3G4250D_HPM_AUTORESET_INT           ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup CTRL2: High_Pass_CUT OFF_Frequency 
  * @{
  */   
#define I3G4250D_HPFCF_ODR105_8HZ    0x00
#define I3G4250D_HPFCF_ODR105_4HZ    0x01
#define I3G4250D_HPFCF_ODR105_2HZ    0x02
#define I3G4250D_HPFCF_ODR105_1HZ    0x03
#define I3G4250D_HPFCF_ODR105_0d5HZ   0x04
#define I3G4250D_HPFCF_ODR105_0d2HZ   0x05
#define I3G4250D_HPFCF_ODR105_0d1HZ   0x06
#define I3G4250D_HPFCF_ODR105_0d05HZ  0x07
#define I3G4250D_HPFCF_ODR105_0d02HZ  0x08
#define I3G4250D_HPFCF_ODR105_0d01HZ  0x09

#define I3G4250D_HPFCF_ODR208_15HZ   0x00
#define I3G4250D_HPFCF_ODR208_8HZ    0x01
#define I3G4250D_HPFCF_ODR208_4HZ    0x02
#define I3G4250D_HPFCF_ODR208_2HZ    0x03
#define I3G4250D_HPFCF_ODR208_1HZ    0x04
#define I3G4250D_HPFCF_ODR208_0d5HZ   0x05
#define I3G4250D_HPFCF_ODR208_0d2HZ   0x06
#define I3G4250D_HPFCF_ODR208_0d1HZ   0x07
#define I3G4250D_HPFCF_ODR208_0d05HZ  0x08
#define I3G4250D_HPFCF_ODR208_0d02HZ  0x09

#define I3G4250D_HPFCF_ODR420_30HZ     0x00
#define I3G4250D_HPFCF_ODR420_15HZ   0x01
#define I3G4250D_HPFCF_ODR420_8HZ    0x02
#define I3G4250D_HPFCF_ODR420_4HZ    0x03
#define I3G4250D_HPFCF_ODR420_2HZ    0x04
#define I3G4250D_HPFCF_ODR420_1HZ    0x05
#define I3G4250D_HPFCF_ODR420_0d5HZ   0x06
#define I3G4250D_HPFCF_ODR420_0d2HZ   0x07
#define I3G4250D_HPFCF_ODR420_0d1HZ   0x08
#define I3G4250D_HPFCF_ODR420_0d05HZ  0x09

#define I3G4250D_HPFCF_ODR840_56HZ   0x00
#define I3G4250D_HPFCF_ODR840_30HZ     0x01
#define I3G4250D_HPFCF_ODR840_15HZ   0x02
#define I3G4250D_HPFCF_ODR840_8HZ    0x03
#define I3G4250D_HPFCF_ODR840_4HZ    0x04
#define I3G4250D_HPFCF_ODR840_2HZ    0x05
#define I3G4250D_HPFCF_ODR840_1HZ    0x06
#define I3G4250D_HPFCF_ODR840_0d5HZ   0x07
#define I3G4250D_HPFCF_ODR840_0d2HZ   0x08
#define I3G4250D_HPFCF_ODR840_0d1HZ   0x09
/**
  * @}
  */

/** @defgroup CTRL3
  * @{
  */   
#define I3G4250D_CTRL3_I1_INT       1 << 7
#define I3G4250D_CTRL3_I1_BOOT      1 << 6
#define I3G4250D_CTRL3_H_LACTIVE    1 << 5
#define I3G4250D_CTRL3_PP_OD        1 << 4
#define I3G4250D_CTRL3_I2_DRDY      1 << 3
#define I3G4250D_CTRL3_I2_WTM       1 << 2
#define I3G4250D_CTRL3_I2_OVR       1 << 1
#define I3G4250D_CTRL3_I2_EMPTY     1 << 0
/**
  * @}
  */
	
/** @defgroup CTRL4 
  * @{
  */   
#define I3G4250D_CTRL4_ST_DIS           0 << 1
#define I3G4250D_CTRL4_ST_POS           1 << 1
#define I3G4250D_CTRL4_ST_NEG           3 << 1
/**
  * @}
  */

/** @defgroup CLTR4: Endian_Data_selection
  * @{
  */  
#define I3G4250D_BLE_LSB                     ((uint8_t)0x00)
#define I3G4250D_BLE_MSB	                   ((uint8_t)0x40)
/**
  * @}
  */

/** @defgroup CTRL4: Full_Scale_Selection 
  * @{
  */
#define I3G4250D_FULLSCALE_245               ((uint8_t)0x00)
#define I3G4250D_FULLSCALE_500               ((uint8_t)0x10)
#define I3G4250D_FULLSCALE_2000              ((uint8_t)0x20) 
/**
  * @}
  */
  
/** @defgroup CTRL5
  * @{
  */   
#define I3G4250D_CTRL5_BOOT           1 << 7
#define I3G4250D_CTRL5_FIFO_ENA       1 << 6
#define I3G4250D_CTRL5_HPF_ENA        1 << 4
#define I3G4250D_CTRL3_INT1_SEL1      1 << 3
#define I3G4250D_CTRL3_INT1_SEL0      1 << 2
#define I3G4250D_CTRL3_OUT_SEL1       1 << 1
#define I3G4250D_CTRL3_OUT_SEL0       1 << 0
/**
  * @}
  */

/** @defgroup INT1_CFG: Axes_Interrupt_Selection 
  * @{
  */
#define I3G4250D_X_INTERRUPT_ENABLE            ((uint8_t)0x02)
#define I3G4250D_Y_INTERRUPT_ENABLE            ((uint8_t)0x08)
#define I3G4250D_Z_INTERRUPT_ENABLE            ((uint8_t)0x20)
#define I3G4250D_AXES_INTERRUPT_ENABLE         ((uint8_t)0x2A)
/**
  * @}
  */

/** @defgroup STATUS_REG
  * @{
  */
#define I3G4250D_STATUS_ZYX_OVR						1 << 7
#define I3G4250D_STATUS_Z_OVR							1 << 6
#define I3G4250D_STATUS_Y_OVR							1 << 5
#define I3G4250D_STATUS_X_OVR							1 << 4
#define I3G4250D_STATUS_ZYX_DA						1 << 3
#define I3G4250D_STATUS_Z_DA							1 << 2
#define I3G4250D_STATUS_Y_DA							1 << 1
#define I3G4250D_STATUS_X_DA							1 << 0
/**
  * @}
  */

/** @defgroup FIFO_CTRL_REG
  * @{
  */   
#define I3G4250D_FIFO_MODE_POS        5
#define I3G4250D_FIFO_MODE_BYPASS         0 << I3G4250D_FIFO_MODE_POS
#define I3G4250D_FIFO_MODE_FIFO           1 << I3G4250D_FIFO_MODE_POS
#define I3G4250D_FIFO_MODE_STREAM         2 << I3G4250D_FIFO_MODE_POS
/**
  * @}
  */

/** @defgroup FIFO_SRC_REG
  * @{
  */   
#define I3G4250D_FIFO_SRC_WTM         1 << 7
#define I3G4250D_FIFO_SRC_OVRN        1 << 6
#define I3G4250D_FIFO_SRC_EMPTY       1 << 5
#define I3G4250D_FIFO_SRC_FSS_MASK    0x1F
/**
  * @}
  */



/** @defgroup STM32F429I-DISCO_I3G4250D_Exported_Macros
  * @{
  */
#define I3G4250D_CS_LOW()       GPIO_ResetBits(I3G4250D_SPI_CS_GPIO_PORT, I3G4250D_SPI_CS_PIN)
#define I3G4250D_CS_HIGH()      GPIO_SetBits(I3G4250D_SPI_CS_GPIO_PORT, I3G4250D_SPI_CS_PIN)
/**
  * @}
  */
 
/** @defgroup STM32F429I-DISCO_I3G4250D_Exported_Functions
  * @{
  */
/* Sensor Configuration Functions */ 
uint8_t I3G4250D_Init(void);
void I3G4250D_RebootCmd(void);

/*INT1 Interrupt Configuration Functions */
void I3G4250D_INT1InterruptCmd(uint8_t InterruptState);
void I3G4250D_INT2InterruptCmd(uint8_t InterruptState);
void I3G4250D_INT1InterruptConfig(uint8_t Interrupt_Axes);
void I3G4250D_INT2InterruptConfig(void);
uint8_t I3G4250D_GetDataStatus(void);

/* High Pass Filter Configuration Functions */
void I3G4250D_FilterConfig(I3G4250D_FilterConfigTypeDef *I3G4250D_FilterStruct);
void I3G4250D_FilterCmd(uint8_t HighPassFilterState);
void I3G4250D_FIFOEnaCmd(uint8_t FIFOState);
void I3G4250D_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void I3G4250D_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
int8_t I3G4250D_GetTemp(void);
uint8_t I3G4250D_GetFIFOStatus(void);
void I3G4250D_SetFIFOMode_WMLevel(uint8_t mode, uint8_t wmlevel);
void I3G4250D_INT2_EXTI_Config(void);

/* USER Callbacks: This is function for which prototype only is declared in
   MEMS accelerometre driver and that should be implemented into user applicaiton. */  
/* I3G4250D_TIMEOUT_UserCallback() function is called whenever a timeout condition 
   occure during communication (waiting transmit data register empty flag(TXE)
   or waiting receive data register is not empty flag (RXNE)).
   You can use the default timeout callback implementation by uncommenting the 
   define USE_DEFAULT_TIMEOUT_CALLBACK in stm32f429i_discovery_I3G4250D.h file.
   Typically the user implementation of this callback should reset MEMS peripheral
   and re-initialize communication or in worst case reset all the application. */
uint32_t I3G4250D_TIMEOUT_UserCallback(void);

void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F429I_DISCOVERY_I3G4250D_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
