/**
  ******************************************************************************
  * @file    sample_service.h 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SAMPLE_SERVICE_H_
#define _SAMPLE_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
#include "hal_types.h"
#include "string.h"
#include "gp_timer.h"
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "bluenrg_gap.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_gap_aci.h"
#include "hci_const.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_aci_const.h"

/** 
* @brief  Handle of TX Characteristic on the Server. The handle should be
*         discovered, but it is fixed only for this demo.
*/ 
#define TX_HANDLE 0x0011

#define RX_HANDLE   0x0014

tBleStatus Add_Sample_Service(void);
void Make_Connection(void);
void receiveData(uint8_t* data_buffer, uint8_t Nb_bytes);
void sendData(uint8_t* data_buffer, uint8_t Nb_bytes);
void enableNotification(void);
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length,
                           uint8_t *att_data);
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void GAP_DisconnectionComplete_CB(void);
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len,
                          uint8_t *attr_value);
void HCI_Event_CB(void *pckt);

#ifdef __cplusplus
}
#endif

#endif /* _SAMPLE_SERVICE_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

