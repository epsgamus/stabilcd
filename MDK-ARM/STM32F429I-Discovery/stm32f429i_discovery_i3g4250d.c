/**
  ******************************************************************************
  * @file    stm32f429i_discovery_i3g4250d.c
  * @brief   This file provides a set of functions needed to manage the i3g4250d
  *          MEMS three-axis digital output gyroscope available on 
  *          STM32F429I-DISCOVERY V1
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f429i_discovery_i3g4250d.h"
/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32F4_DISCOVERY
  * @{
  */ 
  
/** @addtogroup STM32429I_DISCO
  * @{
  */  

/** @addtogroup STM32F429I_DISCOVERY_I3G4250D
  * @{
  */


/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Private_Variables
  * @{
  */ 
__IO uint32_t  I3G4250DTimeout = I3G4250D_FLAG_TIMEOUT;  
/**
  * @}
  */

/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Private_FunctionPrototypes
  * @{
  */
static uint8_t I3G4250D_SendByte(uint8_t byte);
static void I3G4250D_LowLevel_Init(void);
static void I3G4250D_EXTI_Config(void);
static void I3G4250D_INT2_EXTI_Config(void);
/**
  * @}
  */

/** @defgroup STM32F429I_DISCOVERY_I3G4250D_Private_Functions
  * @{
  */

/**
  * @brief  Set I3G4250D Initialization.
  * @param  I3G4250D_InitStruct: pointer to a I3G4250D_InitTypeDef structure 
  *         that contains the configuration setting for the I3G4250D.
  * @retval None
  */
void I3G4250D_Init(I3G4250D_InitTypeDef *I3G4250D_InitStruct)
{  
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  I3G4250D_LowLevel_Init();
  
  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (I3G4250D_InitStruct->Power_Mode | I3G4250D_InitStruct->Output_DataRate | \
                    I3G4250D_InitStruct->Axes_Enable | I3G4250D_InitStruct->Band_Width);
  
  ctrl4 |= (uint8_t) (I3G4250D_InitStruct->BlockData_Update | I3G4250D_InitStruct->Endianness | \
                    I3G4250D_InitStruct->Full_Scale);
                    
  /* Write value to MEMS CTRL_REG1 regsister */
  I3G4250D_Write(&ctrl1, I3G4250D_CTRL_REG1_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG4 regsister */
  I3G4250D_Write(&ctrl4, I3G4250D_CTRL_REG4_ADDR, 1);
}

/**
  * @brief  Reboot memory content of I3G4250D
  * @param  None
  * @retval None
  */
void I3G4250D_RebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  I3G4250D_Read(&tmpreg, I3G4250D_CTRL_REG5_ADDR, 1);
  
  /* Enable the reboot memory */
  tmpreg |= I3G4250D_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG5 regsister */
  I3G4250D_Write(&tmpreg, I3G4250D_CTRL_REG5_ADDR, 1);
}


/**
  * @brief  Read and cnv current TEMP I3G4250D
  * @param  None
  * @retval None
  */
int8_t I3G4250D_GetTemp(void)
{
    uint8_t tmpreg;
  
    /* Read CTRL_REG5 register */
    I3G4250D_Read(&tmpreg, I3G4250D_OUT_TEMP_ADDR, 1);
    
    int8_t temp; 
    
    /*
    if (tmpreg < 128) 
    {
        temp = (int8_t)tmpreg;
    }
    else
    {
        temp = (int8_t)(tmpreg - 256);
    }
    */
       
    return tmpreg;
}


/**
  * @brief Set I3G4250D Interrupt configuration
  * @param  Interrupt_Axes: Axe on which interrupt is activated 
  *         This value could be:
  *           - I3G4250D_X_INTERRUPT_ENABLE: Interrupt on X is activated
  *           - I3G4250D_Y_INTERRUPT_ENABLE: Interrupt on Y is activated
  *           - I3G4250D_Z_INTERRUPT_ENABLE: Interrupt on Z is activated
  *           - I3G4250D_AXES_INTERRUPT_ENABLE: Interrupt on X, Y and Z are activated
  * @retval None
  */
void I3G4250D_INT1InterruptConfig(uint8_t Interrupt_Axes)
{
  uint8_t ctrl_cfr = 0x00;
  
  uint8_t regval[3] = {0x03, 0x00, 0x10};
  
  /* Configure the INT1 line as EXTI source */
  I3G4250D_EXTI_Config();  
  
  /* Configure axes interrupts */                   
  ctrl_cfr |= (uint8_t)(Interrupt_Axes);
  
  /* Write value to MEMS INT1_CFG register */
  I3G4250D_Write(&ctrl_cfr, I3G4250D_INT1_CFG_ADDR, 1);
  
  /* Configure selected axes */
  if(Interrupt_Axes == I3G4250D_X_INTERRUPT_ENABLE)
  {
    I3G4250D_Write(&regval[0], I3G4250D_INT1_TSH_XH_ADDR, 1); 
  }
  else if (Interrupt_Axes == I3G4250D_Y_INTERRUPT_ENABLE)
  {
    I3G4250D_Write(&regval[0], I3G4250D_INT1_TSH_YH_ADDR, 1); 
  }
  else if (Interrupt_Axes == I3G4250D_Z_INTERRUPT_ENABLE) 
  {
    I3G4250D_Write(&regval[0], I3G4250D_INT1_TSH_ZH_ADDR, 1); 
  }
  else
  {
    I3G4250D_Write(&regval[0], I3G4250D_INT1_TSH_XH_ADDR, 1); 
    I3G4250D_Write(&regval[0], I3G4250D_INT1_TSH_YH_ADDR, 1); 
    I3G4250D_Write(&regval[0], I3G4250D_INT1_TSH_ZH_ADDR, 1); 
  }
  I3G4250D_Write(&regval[1], I3G4250D_INT1_TSH_XL_ADDR, 1); 
  I3G4250D_Write(&regval[1], I3G4250D_INT1_TSH_YL_ADDR, 1); 
  I3G4250D_Write(&regval[1], I3G4250D_INT1_TSH_ZL_ADDR, 1);
  I3G4250D_Write(&regval[2], I3G4250D_INT1_DURATION_ADDR, 1);    
}

/**
  * @brief Set I3G4250D INT2/DRDY configuration
  * @param  
  * @retval None
  */
void I3G4250D_INT2InterruptConfig(void)
{
  uint8_t ctrl_cfr = I3G4250D_FIFO_MODE_FIFO | I3G4250D_FIFO_WM_LEVEL;

    /* Configure the INT2 line as EXTI source */
  I3G4250D_INT2_EXTI_Config();
  
  /* Set bypass FIFO mode */
  I3G4250D_Write(&ctrl_cfr, I3G4250D_FIFO_CTRL_REG_ADDR, 1);
}


/**
  * @brief  Enable or disable INT1 interrupt
  * @param  InterruptState: State of INT1 Interrupt 
  *      This parameter can be: 
  *        @arg I3G4250D_INT1INTERRUPT_DISABLE
  *        @arg I3G4250D_INT1INTERRUPT_ENABLE    
  * @retval None
  */
void I3G4250D_INT1InterruptCmd(uint8_t InterruptState)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  I3G4250D_Read(&tmpreg, I3G4250D_CTRL_REG3_ADDR, 1);
                  
  tmpreg |= 0x80 & (InterruptState << 7);
  
  /* Write value to MEMS CTRL_REG3 regsister */
  I3G4250D_Write(&tmpreg, I3G4250D_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Enable or disable INT2 interrupt
  * @param  InterruptState: State of INT2 Interrupt 
  *      This parameter can be: 
  *        @arg I3G4250D_INT2INTERRUPT_DISABLE
  *        @arg I3G4250D_INT2INTERRUPT_ENABLE    
  * @retval None
  */
void I3G4250D_INT2InterruptCmd(uint8_t InterruptState)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  I3G4250D_Read(&tmpreg, I3G4250D_CTRL_REG3_ADDR, 1);
                  
  tmpreg &= 0xF3;	
  tmpreg |= (InterruptState << 2) | (InterruptState << 3);
  
  /* Write value to MEMS CTRL_REG3 regsister */
  I3G4250D_Write(&tmpreg, I3G4250D_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  I3G4250D_FilterStruct: pointer to a I3G4250D_FilterConfigTypeDef structure 
  *         that contains the configuration setting for the I3G4250D.        
  * @retval None
  */
void I3G4250D_FilterConfig(I3G4250D_FilterConfigTypeDef *I3G4250D_FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  I3G4250D_Read(&tmpreg, I3G4250D_CTRL_REG2_ADDR, 1);
  
  tmpreg &= 0xC0;
  
  /* Configure MEMS: mode and cutoff frquency */
  tmpreg |= (uint8_t) (I3G4250D_FilterStruct->HighPassFilter_Mode_Selection |\
                      I3G4250D_FilterStruct->HighPassFilter_CutOff_Frequency);                             

  /* Write value to MEMS CTRL_REG2 regsister */
  I3G4250D_Write(&tmpreg, I3G4250D_CTRL_REG2_ADDR, 1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: I3G4250D_HIGHPASSFILTER_DISABLE 
  *         @arg: I3G4250D_HIGHPASSFILTER_ENABLE          
  * @retval None
  */
void I3G4250D_FilterCmd(uint8_t HighPassFilterState)
 {
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  I3G4250D_Read(&tmpreg, I3G4250D_CTRL_REG5_ADDR, 1);
                  
  tmpreg &= 0xEF;

  tmpreg |= HighPassFilterState;

  /* Write value to MEMS CTRL_REG5 regsister */
  I3G4250D_Write(&tmpreg, I3G4250D_CTRL_REG5_ADDR, 1);
}

/**
  * @brief  Enable or Disable FIFO
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: DISABLE 
  *         @arg: ENABLE          
  * @retval None
  */
void I3G4250D_FIFOEnaCmd(uint8_t FIFOState)
 {
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  I3G4250D_Read(&tmpreg, I3G4250D_CTRL_REG5_ADDR, 1);
                  
  tmpreg &= 0xBF;

  tmpreg |= (FIFOState << 6);

  /* Write value to MEMS CTRL_REG5 regsister */
  I3G4250D_Write(&tmpreg, I3G4250D_CTRL_REG5_ADDR, 1);
}



/**
  * @brief  Get status for I3G4250D data
  * @param  None         
  * @retval I3G4250D status
  */
uint8_t I3G4250D_GetDataStatus(void)
{
  uint8_t tmpreg;
  
  /* Read STATUS_REG register */
  I3G4250D_Read(&tmpreg, I3G4250D_STATUS_REG_ADDR, 1);
                  
  return tmpreg;
}

/**
  * @brief  Get FIFO status for I3G4250D data
  * @param  None         
  * @retval FIFO status
  */
uint8_t I3G4250D_GetFIFOStatus(void)
{
  uint8_t tmpreg;
  
  /* Read STATUS_REG register */
  I3G4250D_Read(&tmpreg, I3G4250D_FIFO_SRC_REG_ADDR, 1);
                  
  return tmpreg;
}

/**
  * @brief  Writes a block of data to the I3G4250D.
  * @param  pBuffer : pointer to the buffer containing the data to be written to the I3G4250D.
  * @param  WriteAddr : I3G4250D's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void I3G4250D_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  I3G4250D_CS_LOW();
  
  /* Send the Address of the indexed register */
  I3G4250D_SendByte(WriteAddr);

  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    I3G4250D_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  I3G4250D_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the I3G4250D.
  * @param  pBuffer : pointer to the buffer that receives the data read from the I3G4250D.
  * @param  ReadAddr : I3G4250D's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the I3G4250D.
  * @retval None
  */
void I3G4250D_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  I3G4250D_CS_LOW();
  
  /* Send the Address of the indexed register */
  I3G4250D_SendByte(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to I3G4250D (Slave device) */
    *pBuffer = I3G4250D_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  I3G4250D_CS_HIGH();
}  

/**
  * @brief  Initializes the low level interface used to drive the I3G4250D
  * @param  None
  * @retval None
  */
static void I3G4250D_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(I3G4250D_SPI_CLK, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(I3G4250D_SPI_SCK_GPIO_CLK | I3G4250D_SPI_MISO_GPIO_CLK | I3G4250D_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS GPIO clock */
  RCC_AHB1PeriphClockCmd(I3G4250D_SPI_CS_GPIO_CLK, ENABLE);
  
  GPIO_PinAFConfig(I3G4250D_SPI_SCK_GPIO_PORT, I3G4250D_SPI_SCK_SOURCE, I3G4250D_SPI_SCK_AF);
  GPIO_PinAFConfig(I3G4250D_SPI_MISO_GPIO_PORT, I3G4250D_SPI_MISO_SOURCE, I3G4250D_SPI_MISO_AF);
  GPIO_PinAFConfig(I3G4250D_SPI_MOSI_GPIO_PORT, I3G4250D_SPI_MOSI_SOURCE, I3G4250D_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = I3G4250D_SPI_SCK_PIN;
  GPIO_Init(I3G4250D_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  I3G4250D_SPI_MOSI_PIN;
  GPIO_Init(I3G4250D_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = I3G4250D_SPI_MISO_PIN;
  GPIO_Init(I3G4250D_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(I3G4250D_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz) 
     to verify these constraints:
        - ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
        - I3G4250D SPI interface max baudrate is 10MHz for write/read
        - PCLK2 frequency is set to 90 MHz 
    */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(I3G4250D_SPI, &SPI_InitStructure);

  /* Enable I3G4250D_SPI  */
  SPI_Cmd(I3G4250D_SPI, ENABLE);
  
  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = I3G4250D_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(I3G4250D_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(I3G4250D_SPI_CS_GPIO_PORT, I3G4250D_SPI_CS_PIN);

}  

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t I3G4250D_SendByte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  I3G4250DTimeout = I3G4250D_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(I3G4250D_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((I3G4250DTimeout--) == 0) return I3G4250D_TIMEOUT_UserCallback();
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(I3G4250D_SPI, (uint16_t)byte);
  /* Wait to receive a Byte */
  I3G4250DTimeout = I3G4250D_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(I3G4250D_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((I3G4250DTimeout--) == 0) return I3G4250D_TIMEOUT_UserCallback();
  }
  
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(I3G4250D_SPI);
}

/**
  * @brief  Configure the INT1 MEMS Interrupt line and GPIO in EXTI mode.
  * @param  None        
  * @retval None
  */
static void I3G4250D_EXTI_Config(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* Enable INT1 GPIO clock */
  // PA1 pin
  RCC_AHB1PeriphClockCmd(I3G4250D_SPI_INT1_GPIO_CLK, ENABLE);
  
  /* Enable INT2 GPIO clock */
  // PA2 pin
  RCC_AHB1PeriphClockCmd(I3G4250D_SPI_INT2_GPIO_CLK, ENABLE);
     
  /* Configure Interrupt INT1 pin  */
  GPIO_InitStructure.GPIO_Pin = I3G4250D_SPI_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(I3G4250D_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure Interrupt INT2 pin  */  
  GPIO_InitStructure.GPIO_Pin = I3G4250D_SPI_INT2_PIN;
  GPIO_Init(I3G4250D_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
  /* Connect Button EXTI Line to INT1 GPIO Pin */
  SYSCFG_EXTILineConfig(I3G4250D_SPI_INT1_EXTI_PORT_SOURCE, I3G4250D_SPI_INT1_EXTI_PIN_SOURCE);  
  
  /* Configure INT1 EXTI line */
  EXTI_InitStructure.EXTI_Line = I3G4250D_SPI_INT1_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set INT1 EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = I3G4250D_SPI_INT1_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}

/**
  * @brief  Configure the INT2/DRDY MEMS Interrupt line and GPIO in EXTI mode.
  * @param  None        
  * @retval None
  */
static void I3G4250D_INT2_EXTI_Config(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* Enable INT2 GPIO clock */
  // PA2 pin
  RCC_AHB1PeriphClockCmd(I3G4250D_SPI_INT2_GPIO_CLK, ENABLE);
     
  /* Configure Interrupt INT2 pin  */  
  GPIO_InitStructure.GPIO_Pin = I3G4250D_SPI_INT2_PIN;
  GPIO_Init(I3G4250D_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
  /* Connect Button EXTI Line to INT2 GPIO Pin */
  SYSCFG_EXTILineConfig(I3G4250D_SPI_INT2_EXTI_PORT_SOURCE, I3G4250D_SPI_INT2_EXTI_PIN_SOURCE);  
  
  /* Configure INT2 EXTI line */
  EXTI_InitStructure.EXTI_Line = I3G4250D_SPI_INT2_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set INT2 EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = I3G4250D_SPI_INT2_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}


#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t I3G4250D_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {   
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

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
