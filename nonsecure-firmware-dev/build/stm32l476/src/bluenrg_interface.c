/**
  ******************************************************************************
  * @file    bluenrg_interface.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   This file provides code for the BlueNRG Shield driver
  *          based on STM32Cube HAL for STM32 Nucleo boards.
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
#include "hal.h"
#include "hci.h"

#include "ble_status.h"
#include "hci.h"
#include "stm32_bluenrg_ble.h"
#include "gp_timer.h"
#include "stm32l4xx_nucleo.h"

#if 0
void Hal_IRQ_Serial(void) {
  /* 
  * No need to call Clear_SPI_EXTI_Flag() here as
  * HAL_GPIO_EXTI_IRQHandler() already does it
  */
  
  tHciDataPacket * hciReadPacket = NULL;
  uint8_t data_len;
  
  
  while (HAL_GPIO_ReadPin(BNRG_SPI_EXTI_PORT, BNRG_SPI_EXTI_PIN) == GPIO_PIN_SET) {
    if (list_is_empty (&hciReadPktPool) == FALSE){
      /* enqueueing a packet for read */
      list_remove_head (&hciReadPktPool, (tListNode **)&hciReadPacket);
      data_len = BlueNRG_SPI_Read_All(&SpiHandle, hciReadPacket->dataBuff, HCI_READ_PACKET_SIZE);
      
      if(data_len > 0){
        /* Packet will be inserted to the correct queue */
        HCI_Input(hciReadPacket);
      } else {
        /* Insert the packet back into the pool */
        list_insert_head(&hciReadPktPool, (tListNode *)hciReadPacket);
      }
      
    } else{
      /* TODO: HCI Read Packet Pool is empty, wait for a free packet */
    }
    
    // trigger hci process
    SCB->ICSR |= 1<<28;
    
    Clear_SPI_EXTI_Flag();
  }
  
  //Clear_SPI_EXTI_Flag();
}
#endif 

/**
 * @brief  Writes data to a serial interface.
 * @param  data1   :  1st buffer
 * @param  data2   :  2nd buffer
 * @param  n_bytes1: number of bytes in 1st buffer
 * @param  n_bytes2: number of bytes in 2nd buffer
 * @retval None
 */
void Hal_Write_Serial(const void* data1, const void* data2, uint16_t n_bytes1,
                      uint16_t n_bytes2)
{
  struct timer t;
  
  Timer_Set(&t, CLOCK_SECOND/10);
  
  while(1){
    if(BlueNRG_SPI_Write((uint8_t *)data1,(uint8_t *)data2, n_bytes1, n_bytes2)==0) break;
    if(Timer_Expired(&t)){
      break;
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
