/**
  ******************************************************************************
  * @file    stm32_bluenrg_ble.c
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 
  
/* Includes ------------------------------------------------------------------*/
#include "stm32_bluenrg_ble.h"
#include "stm32l4xx_nucleo_bluenrg.h"

#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255
#define TIMEOUT_DURATION -1 //15

#define ENABLE_SPI_FIX

SPI_HandleTypeDef SpiHandle;

/* Private function prototypes -----------------------------------------------*/
static void us150Delay(void);
void set_irq_as_output(void);
void set_irq_as_input(void);

/**
* @brief  Initializes the SPI communication with the BlueNRG Shield.
* @param  None
* @retval None
*/
void BNRG_SPI_Init(void)
{
  SpiHandle.Instance = BNRG_SPI_INSTANCE;
  SpiHandle.Init.Mode = BNRG_SPI_MODE;
  SpiHandle.Init.Direction = BNRG_SPI_DIRECTION;
  SpiHandle.Init.DataSize = BNRG_SPI_DATASIZE;
  SpiHandle.Init.CLKPolarity = BNRG_SPI_CLKPOLARITY;
  SpiHandle.Init.CLKPhase = BNRG_SPI_CLKPHASE;
  SpiHandle.Init.NSS = BNRG_SPI_NSS;
  SpiHandle.Init.FirstBit = BNRG_SPI_FIRSTBIT;
  SpiHandle.Init.TIMode = BNRG_SPI_TIMODE;
  SpiHandle.Init.CRCPolynomial = BNRG_SPI_CRCPOLYNOMIAL;
  SpiHandle.Init.BaudRatePrescaler = BNRG_SPI_BAUDRATEPRESCALER;
  SpiHandle.Init.CRCCalculation = BNRG_SPI_CRCCALCULATION;
  
  HAL_SPI_Init(&SpiHandle);
}

/**
 * @brief  Resets the BlueNRG.
 * @param  None
 * @retval None
 */
void BlueNRG_RST(void)
{
  BNRG_SPI_RESET_PORT->ODR &= ~BNRG_SPI_RESET_PIN;
  HAL_Delay(5);
  BNRG_SPI_RESET_PORT->ODR |= BNRG_SPI_RESET_PIN;
  HAL_Delay(5);
}

/**
* @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
* @param  buffer   : Buffer where data from SPI are stored
* @param  buff_size: Buffer size
* @retval int32_t  : Number of read bytes
*/
uint8_t BlueNRG_SPI_Read_All(uint8_t *buffer,
                             uint8_t buff_size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;
  
#if 0
  int32_t spi_fix_enabled = 0; /* FIXME: To be used only if SYSCLK_FREQ=32MHz */
  
#ifdef ENABLE_SPI_FIX
  spi_fix_enabled = 1;
#endif //ENABLE_SPI_FIX
#endif 
  
  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];
    
#if 0
  /*
   If the SPI_FIX is enabled the IRQ is set in Output mode, then it is pulled
   high and, after a delay of at least 112us, the CS line is asserted and the
   header transmit/receive operations are started.
   After these transmit/receive operations the IRQ is reset in input mode.
  */
  if (spi_fix_enabled) {
    set_irq_as_output();
    
    /* Assert CS line after at least 112us */
    us150Delay();
  }
#endif 
  //us150Delay();
  
  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
  
  //us150Delay();

  /* Read the header */  
  if (HAL_SPI_TransmitReceive(&SpiHandle, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION) != HAL_OK) {
    for(;;);
  }
   
#if 0
  if (spi_fix_enabled) {
    set_irq_as_input();
  }
#endif
  
  if (header_slave[0] == 0x02) {
    /* device is ready */
    byte_count = (header_slave[4]<<8)|header_slave[3];
    
    if (byte_count > 0) {
      
      /* avoid to read more data that size of the buffer */
      if (byte_count > buff_size){
        byte_count = buff_size;
      }
      
      // we're reading in buffer, first wipe it out, and fill it with data from the bus
      memset(buffer, 0xFF, byte_count);
      if (HAL_SPI_TransmitReceive(&SpiHandle, buffer, buffer, byte_count, TIMEOUT_DURATION) != HAL_OK) {
        for(;;);
      }
      len = byte_count;
      /*
      for (len = 0; len < byte_count; len++){
        HAL_SPI_TransmitReceive(&SpiHandle, &char_ff, (uint8_t*)&read_char, 1, TIMEOUT_DURATION);
        buffer[len] = read_char;
      }
      */
    }    
  }
  /* Release CS line */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  
  // Add a small delay to give time to the BlueNRG to set the IRQ pin low
  // to avoid a useless SPI read at the end of the transaction
  for(volatile int i = 0; i < 2; i++)__NOP();

  return len;   
}

/**
* @brief  Writes data from local buffer to SPI.
* @param  data1    : First data buffer to be written
* @param  data2    : Second data buffer to be written
* @param  Nb_bytes1: Size of first data buffer to be written
* @param  Nb_bytes2: Size of second data buffer to be written
* @retval Number of read bytes
*/
uint8_t BlueNRG_SPI_Write(uint8_t* data1,
                          uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{  
  uint8_t result = 0;
  
  int32_t spi_fix_enabled = 0; /* FIXME: To be used only if SYSCLK_FREQ=32MHz */
  
#ifdef ENABLE_SPI_FIX
  spi_fix_enabled = 1;
#endif //ENABLE_SPI_FIX
  
  unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};
  
  unsigned char read_char_buf[MAX_BUFFER_SIZE];
  
  Disable_SPI_IRQ(); 
  
  /*
   If the SPI_FIX is enabled the IRQ is set in Output mode, then it is pulled
   high and, after a delay of at least 112us, the CS line is asserted and the
   header transmit/receive operations are started.
   After these transmit/receive operations the IRQ is reset in input mode.
  */
  if (spi_fix_enabled) {
    set_irq_as_output();
    
    /* Assert CS line after at least 112us */
    us150Delay();
    
    // avoid glitch
    Clear_SPI_EXTI_Flag();
  }
  
  //us150Delay();
  
  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
  
  //us150Delay();
  
  /* Exchange header */  
  if (HAL_SPI_TransmitReceive(&SpiHandle, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION) != HAL_OK) {
    for(;;);
  }
  
  if (spi_fix_enabled) {
    set_irq_as_input();
  }
  
  if (header_slave[0] == 0x02) {
    /* SPI is ready */
    if (header_slave[1] >= (Nb_bytes1+Nb_bytes2)) {
      
      /*  Buffer is big enough */
      if (Nb_bytes1 > 0) {
        if (HAL_SPI_TransmitReceive(&SpiHandle, data1, read_char_buf, Nb_bytes1, TIMEOUT_DURATION) != HAL_OK) {
          for(;;);
        }
      }
      if (Nb_bytes2 > 0) {
        if(HAL_SPI_TransmitReceive(&SpiHandle, data2, read_char_buf, Nb_bytes2, TIMEOUT_DURATION) != HAL_OK) {
          for(;;);
        }
      }
            
    } else {
      /* Buffer is too small */
      result = -2;
    }
  } else {
    /* SPI is not ready */
    result = -1;
  }
  
  /* Release CS line */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  
  Enable_SPI_IRQ();
  
  return result;
}

/**
 * @brief  Set in Output mode the IRQ.
 * @param  None
 * @retval None
 */
void set_irq_as_output()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Pull IRQ high */
  GPIO_InitStructure.Pin = BNRG_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Set the IRQ in input mode.
 * @param  None
 * @retval None
 */
void set_irq_as_input()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* IRQ input */  
  GPIO_InitStructure.Pin = BNRG_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode = BNRG_SPI_IRQ_MODE;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
  GPIO_InitStructure.Alternate = BNRG_SPI_IRQ_ALTERNATE;    
  HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pull = BNRG_SPI_IRQ_PULL;
  HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Utility function for delay
  * @param  None 
  * @retval None
  * NOTE: TODO: implement with clock-independent function.
  */
static void us150Delay()
{ 
  volatile int i = 10;
  while(i--) __NOP();
#if 0
previous version from ST
#if SYSCLK_FREQ == 4000000
  for(volatile int i = 0; i < 35; i++)__NOP();
#elif SYSCLK_FREQ == 32000000
  for(volatile int i = 0; i < 420; i++)__NOP();
#else
#error Implement delay function.
#endif
#endif 
}

/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
void Enable_SPI_IRQ(void)
{  
  HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);  
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
void Disable_SPI_IRQ(void)
{  
  HAL_NVIC_DisableIRQ(BNRG_SPI_EXTI_IRQn);
}

/**
 * @brief  Clear Pending SPI IRQ.
 * @param  None
 * @retval None
 */
void Clear_SPI_IRQ(void)
{
  HAL_NVIC_ClearPendingIRQ(BNRG_SPI_EXTI_IRQn);
}

/**
 * @brief  Clear EXTI (External Interrupt) line for SPI IRQ.
 * @param  None
 * @retval None
 */
void Clear_SPI_EXTI_Flag(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(BNRG_SPI_EXTI_PIN);
}
















    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
