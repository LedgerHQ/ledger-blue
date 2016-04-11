/**
  ******************************************************************************
  * @file           : usbd_conf.c
  * @brief          : This file implements the board support package for the USB device library
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
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
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"

#include "seproxyhal.h"
//#include "usbd_hid.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

//#define USB_LOCAL

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/* Exported function prototypes -----------------------------------------------*/
extern USBD_StatusTypeDef USBD_LL_BatteryCharging(USBD_HandleTypeDef *pdev);
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN 1 */
static void SystemClockConfig_Resume(void);
/* USER CODE END 1 */
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state);
extern void SystemClock_Config(void);

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* MSP Init */

void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hpcd->Instance==USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

  /* USER CODE END USB_OTG_FS_MspInit 0 */
  
    /**USB_OTG_FS GPIO Configuration    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __USB_OTG_FS_CLK_ENABLE();

    /* Enable VDDUSB */
    if(__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWREx_EnableVddUSB();
      __HAL_RCC_PWR_CLK_DISABLE();
    }
    else
    {
      HAL_PWREx_EnableVddUSB();
    }

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

  /* USER CODE END USB_OTG_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB_OTG_FS)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

  /* USER CODE END USB_OTG_FS_MspDeInit 0 */
    /* Peripheral clock disable */
    __USB_OTG_FS_CLK_DISABLE();
  
    /**USB_OTG_FS GPIO Configuration    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* Disable VDDUSB */
    if(__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWREx_DisableVddUSB();
      __HAL_RCC_PWR_CLK_DISABLE();
    }
    else
    {
      HAL_PWREx_DisableVddUSB();
    }

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

  /* USER CODE END USB_OTG_FS_MspDeInit 1 */
  }
}

/**
  * @brief  Setup Stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */

extern void bootloader_erase_appconf(void);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  if (G_io_usb.bootloader) {
    USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
  }
  else {
    /*
    // hacky hax
    if (hpcd->Setup[0]&0xFFFF == 0xB160) {
      // jump failed (or loaded code returned) go to bootloader
      bootloader_erase_appconf();

      // ensure to reset the display of the bootloader prompt
      NVIC_SystemReset();
    }
    */

    // notify setup to the ST31
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_SETUP;
    /* don't clear previous status notification
    // clear ep transfer notification 
    G_io_usb.ep_out &= ~1;
    G_io_usb.ep_in &= ~1;
    */
  }
}

/**
  * @brief  Data Out Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  if (G_io_usb.bootloader) {
    USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
  } 
  else {
    G_io_usb.ep_out |= 1<<epnum;
    // buffer is set when preparing OUT
    G_io_usb.ep_out_len[epnum] = hpcd_USB_OTG_FS.OUT_ep[epnum].xfer_count;
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_XFER_OUT;
  }
}

/**
  * @brief  Data In Stage callback..
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  if (G_io_usb.bootloader) {
    USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
  }
  else {
    G_io_usb.ep_in |= 1<<epnum;
    G_io_usb.ep_in_len[epnum] = hpcd_USB_OTG_FS.IN_ep[epnum].xfer_count;
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_XFER_IN;
  }
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  if (G_io_usb.bootloader) {
    USBD_LL_SOF(hpcd->pData);
  }
  else {
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_SOF;
  }
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{  
  /*Reset Device*/
  USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);  
  USBD_LL_Reset(hpcd->pData);

  if (!G_io_usb.bootloader) {
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_RESET;
  }
}

/**
  * @brief  Suspend callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  if (G_io_usb.bootloader) {
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
    /* Inform USB library that core enters in suspend Mode */
    USBD_LL_Suspend(hpcd->pData);
    /* USER CODE BEGIN 3 */
    if (hpcd->Init.low_power_enable)
    {    
      /* Reset SLEEPDEEP bit of Cortex System Control Register */
      SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }  
    /* USER CODE END 3 */
  }
  else {
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
    if (hpcd->Init.low_power_enable)
    {
      /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register */
      SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
    //G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_SUSPENDED;
  }
}

/**
  * @brief  Resume callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  if (G_io_usb.bootloader) {
    __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
    /* USER CODE BEGIN 3 */
    if (hpcd->Init.low_power_enable)
    {    
      /* Reset SLEEPDEEP bit of Cortex System Control Register */
      SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));    
    }  
    /* USER CODE END 3 */
    USBD_LL_Resume(hpcd->pData);
  }
  else {
    __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
    if (hpcd->Init.low_power_enable)
    {    
      /* Reset SLEEPDEEP bit of Cortex System Control Register */
      SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));    
    }
    //G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_RESUMED;
  }
}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  if (G_io_usb.bootloader) {
    USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
  }
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  if (G_io_usb.bootloader) {
    USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
  }
}

/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev)
{ 
  /* Init USB_IP */
  if (pdev->id == DEVICE_FS) {
    // KTHX
    memset(&hpcd_USB_OTG_FS, 0, sizeof(hpcd_USB_OTG_FS));

    /* enable USB power on Pwrctrl CR2 register */
    /* Link The driver to the stack */	
    hpcd_USB_OTG_FS.pData = pdev;
    pdev->pData = &hpcd_USB_OTG_FS;
    
    hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints = 7;
    hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
    hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE; // disabled for setup rx to occur without a debug stall
    HAL_PCD_Init(&hpcd_USB_OTG_FS);

    // do the minimum for first setup handling (redone upon endpoint configure)
    HAL_PCD_SetRxFiFo(&hpcd_USB_OTG_FS, 0x100);
    HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
  }
  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK; 
}

/**
  * @brief  Starts the Low Level portion of the Device driver. 
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK; 
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK; 
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  ep_type,
                                      uint16_t ep_mps)
{

  HAL_PCD_EP_Open(pdev->pData, 
                  ep_addr, 
                  ep_mps, 
                  ep_type);

  // ensure fifo is configured for this endpoint
  // DESIGN NOTE: USB endpoint must be configured in order else the allocation macro from st is pure trash.
  if (ep_mps >= 16*4 && (ep_addr & 0x80)) {
    HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_FS, ep_addr&0x7F, ep_mps/4);
  }
  
  return USBD_OK; 
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);  
  return USBD_OK; 
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  PCD_HandleTypeDef *hpcd = pdev->pData; 
  
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall; 
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall; 
  }
}
/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr)   
{
  
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK; 
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size    
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  *pbuf,
                                      uint16_t  size)
{

  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;   
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, 
                                           uint8_t  ep_addr,                                      
                                           uint16_t  size)
{

  HAL_PCD_EP_Receive(pdev->pData, ep_addr, G_io_usb.ep_out_buff[ep_addr&0x7F], size);
  return USBD_OK;   
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr)  
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
  * @brief  GPIO EXTI Callback function
  *         Handle USB VBUS detection upon External interrupt
  * @param  GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_9)
  {
    HAL_PCDEx_BCD_VBUSDetect (&hpcd_USB_OTG_FS);
  }
}

#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  HAL_PCDEx_LPM_Callback : Send LPM message to user layer
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval HAL status
  */
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg)
{
  switch ( msg)
  {
  case PCD_LPM_L0_ACTIVE:
    if (hpcd->Init.low_power_enable)
    {
      SystemClock_Config();
      
      /* Reset SLEEPDEEP bit of Cortex System Control Register */
      SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }
    __HAL_PCD_UNGATE_PHYCLOCK(hpcd);
    USBD_LL_Resume(hpcd->pData);    
    break;
    
  case PCD_LPM_L1_ACTIVE:
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
    USBD_LL_Suspend(hpcd->pData);
    
    /*Enter in STOP mode */
    if (hpcd->Init.low_power_enable)
    {   
      /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register */
      SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    }     
    break;   
  }
}
#endif
/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void  USBD_LL_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);  
}

#if 0
/**
  * @brief  static single allocation.
  * @param  size: size of allocated memory
  * @retval None
  */
void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_HID_HandleTypeDef)/4)+1];//On 32-bit boundary
  return mem;
}

/**
  * @brief  Dummy memory free
  * @param  *p pointer to allocated  memory address
  * @retval None
  */
void USBD_static_free(void *p)
{

}
#endif 

#if 0
/* USER CODE BEGIN 5 */
/**
  * @brief  Configures system clock after wake-up from USB Resume CallBack: 
  *         enable HSI, PLL and select PLL as system clock source.
  * @param  None
  * @retval None
  */
static void SystemClockConfig_Resume(void)
{
  SystemClock_Config();
}
/* USER CODE END 5 */
#endif // 0

/**
* @brief Software Device Connection
* @param hpcd: PCD handle
* @param state: connection state (0 : disconnected / 1: connected) 
* @retval None
*/
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state)
{
/* USER CODE BEGIN 6 */
  if (state == 1)
  {
    /* Configure Low Connection State */
	
  }
  else
  {
    /* Configure High Connection State */
   
  } 
/* USER CODE END 6 */
}

/**
  * @brief  Verify if the Battery Charging Detection mode (BCD) is used :
  *         return USBD_OK if true
  *         else return USBD_FAIL if false
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_BatteryCharging(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*)pdev->pData;
  if (hpcd->Init.battery_charging_enable == ENABLE) 
  {
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

#define IO_HID_EP_LENGTH 64

#define CHANNEL_APDU 0
#define CHANNEL_KEYBOARD 1
#define CHANNEL_SPI 2
#define IO_RESET_AFTER_REPLIED 0x80
#define IO_RECEIVE_DATA 0x40
#define IO_RETURN_AFTER_TX 0x20
#define IO_FLAGS 0xF0

typedef enum io_usb_hid_receive_status_e {
  IO_USB_APDU_RESET,
  IO_USB_APDU_MORE_DATA,
  IO_USB_APDU_RECEIVED,
} io_usb_hid_receive_status_t;

typedef void (*io_send_t)(unsigned char* buffer, unsigned short length);

typedef unsigned short (*io_recv_t)(unsigned char* buffer, unsigned short maxlenth);



extern volatile unsigned short G_io_apdu_length;
unsigned char G_io_hid_chunk[IO_HID_EP_LENGTH];

/**
 *  Ledger Protocol with FIDO injection
 *  HID Report Content
 *  [______________________________]
 *   CCCC TT VVVV.........VV FILL..
 *
 *  All fields are big endian encoded.
 *  CCCC: 2 bytes channel identifier (when multi application are processing).
 *  TT: 1 byte content tag
 *  VVVV..VV: variable length content
 *  FILL..: 00's to fillup the HID report length
 *
 *  LL is at most the length of the HID Report.
 *
 *  Command/Response APDU are split in chunks.
 * 
 *  Filler only allowed at the end of the last hid report of a apdu chain in each direction.
 * 
 *  APDU are using either standard or extended header. up to the application to check the total received length and the lc field
 *
 *  Tags:
 *  Direction:Host>Token T:0x00 V:no  Get protocol version big endian encoded. Replied with a protocol-version. Channel identifier is ignored for this command.
 *  Direction:Token>Host T:0x00 V:yes protocol-version-4-bytes-big-endian. Channel identifier is ignored for this reply.
 *  Direction:Host>Token T:0x01 V:no  Allocate channel. Replied with a channel identifier. Channel identifier is ignored for this command.
 *  Direction:Token>Host T:0x01 V:yes channel-identifier-2-bytes. Channel identifier is ignored for this reply.
 *  Direction:*          T:0x02 V:no  Ping. replied with a ping. Channel identifier is ignored for this command.
 *  NOTSUPPORTED Direction:*          T:0x03 V:no  Abort. replied with an abort if accepted, else not replied.
 *  Direction:*          T:0x05 V=<sequence-idx-U16><seq==0?totallength:NONE><apducontent> APDU (command/response) packet.
 */

volatile unsigned int   G_io_usb_hid_total_length;
volatile unsigned int   G_io_usb_hid_remaining_length;
volatile unsigned int   G_io_usb_hid_sequence_number;
volatile unsigned char* G_io_usb_hid_current_buffer;


void io_usb_hid_init(void) {
  G_io_usb_hid_sequence_number = 0; 
  //G_io_usb_hid_remaining_length = 0; // not really needed
  //G_io_usb_hid_total_length = 0; // not really needed
  //G_io_usb_hid_current_buffer = G_io_apdu_buffer; // not really needed
}

io_usb_hid_receive_status_t io_usb_hid_receive (io_send_t sndfct, unsigned char* buffer, unsigned short l) {
  // avoid over/under flows
  memset(G_io_hid_chunk, 0, sizeof(G_io_hid_chunk));
  memmove(G_io_hid_chunk, buffer, MIN(l, sizeof(G_io_hid_chunk)));

  // process the chunk content
  switch(G_io_hid_chunk[2]) {
  case 0x05:
    // ensure sequence idx is 0 for the first chunk ! 
    if (G_io_hid_chunk[3] != (G_io_usb_hid_sequence_number>>8) || G_io_hid_chunk[4] != (G_io_usb_hid_sequence_number&0xFF)) {
      // ignore packet
      goto apdu_reset;
    }
    // cid, tag, seq
    l -= 2+1+2;
    
    // append the received chunk to the current command apdu
    if (G_io_usb_hid_sequence_number == 0) {
      /// This is the apdu first chunk
      // total apdu size to receive
      G_io_usb_hid_total_length = (G_io_hid_chunk[5]<<8)+(G_io_hid_chunk[6]&0xFF);
      // check for invalid length encoding (more data in chunk that announced in the total apdu)
      if (G_io_usb_hid_total_length > sizeof(G_io_apdu_buffer)) {
        goto apdu_reset;
      }
      // seq and total length
      l -= 2;
      // compute remaining size to receive
      G_io_usb_hid_remaining_length = G_io_usb_hid_total_length;
      G_io_usb_hid_current_buffer = G_io_apdu_buffer;

      if (l > G_io_usb_hid_remaining_length) {
        l = G_io_usb_hid_remaining_length;
      }
      // copy data
      memmove(G_io_usb_hid_current_buffer, G_io_hid_chunk+7, l);
    }
    else {
      // check for invalid length encoding (more data in chunk that announced in the total apdu)
      if (l > G_io_usb_hid_remaining_length) {
        l = G_io_usb_hid_remaining_length;
      }

      /// This is a following chunk
      // append content
      memmove(G_io_usb_hid_current_buffer, G_io_hid_chunk+5, l);
    }
    // factorize (f)
    G_io_usb_hid_current_buffer += l;
    G_io_usb_hid_remaining_length -= l;
    G_io_usb_hid_sequence_number++;
    break;

  case 0x00: // get version ID
    // do not reset the current apdu reception if any
    memset(G_io_hid_chunk+3, 0, 4); // PROTOCOL VERSION is 0
    // send the response
    sndfct(G_io_hid_chunk, IO_HID_EP_LENGTH);
    // await for the next chunk
    goto apdu_reset;

  case 0x01: // ALLOCATE CHANNEL
    // do not reset the current apdu reception if any
    memset(G_io_hid_chunk+3, 0, 4);
    // send the response
    sndfct(G_io_hid_chunk, IO_HID_EP_LENGTH);
    // await for the next chunk
    goto apdu_reset;

  case 0x02: // ECHO|PING
    // do not reset the current apdu reception if any
    // send the response
    sndfct(G_io_hid_chunk, IO_HID_EP_LENGTH);
    // await for the next chunk
    goto apdu_reset;
  }

  // if more data to be received, notify it
  if (G_io_usb_hid_remaining_length) {
    return IO_USB_APDU_MORE_DATA;
  }

  // reset sequence number for next exchange
  io_usb_hid_init();
  return IO_USB_APDU_RECEIVED;

apdu_reset:
  io_usb_hid_init();
  return IO_USB_APDU_RESET;
}

unsigned short io_usb_hid_exchange(io_send_t sndfct, unsigned short sndlength,
                                   unsigned char flags) {
  unsigned char l;

  // perform send
  if (sndlength) {
    G_io_usb_hid_sequence_number = 0; 
    G_io_usb_hid_current_buffer = G_io_apdu_buffer;
    // consume previous command apdu
    G_io_apdu_length = 0;
    while(sndlength) {

      // fill the chunk
      memset(G_io_hid_chunk+2, 0, IO_HID_EP_LENGTH-2);

      // keep the channel identifier
      G_io_hid_chunk[2] = 0x05;
      G_io_hid_chunk[3] = G_io_usb_hid_sequence_number>>8;
      G_io_hid_chunk[4] = G_io_usb_hid_sequence_number;

      if (G_io_usb_hid_sequence_number == 0) {
        l = ((sndlength>IO_HID_EP_LENGTH-7) ? IO_HID_EP_LENGTH-7 : sndlength);
        G_io_hid_chunk[5] = sndlength>>8;
        G_io_hid_chunk[6] = sndlength;
        memmove(G_io_hid_chunk+7, G_io_usb_hid_current_buffer, l);
        G_io_usb_hid_current_buffer += l;
        sndlength -= l;
        l += 7;
      }
      else {
        l = ((sndlength>IO_HID_EP_LENGTH-5) ? IO_HID_EP_LENGTH-5 : sndlength);
        memmove(G_io_hid_chunk+5, G_io_usb_hid_current_buffer, l);
        G_io_usb_hid_current_buffer += l;
        sndlength -= l;
        l += 5;
      }
      // prepare next chunk numbering
      G_io_usb_hid_sequence_number++;
      // send the chunk
      // always pad :)
      sndfct(G_io_hid_chunk, sizeof(G_io_hid_chunk));
    }

    //G_io_apdu_length = 0;

    // prepare for next apdu
    io_usb_hid_init();
  }

  if (flags & IO_RESET_AFTER_REPLIED) {
    NVIC_SystemReset();
  }

  if (flags & IO_RETURN_AFTER_TX ) {
    return 0;
  }

  return G_io_apdu_length;
}


extern USBD_HandleTypeDef USBD_Device;
void io_usb_send_apdu_data(unsigned char* buffer, unsigned short length) {
  USBD_LL_Transmit(&USBD_Device, 0x82, buffer, length);
}

int8_t CUSTOM_HID_OutEvent  (uint8_t* hid_report)
{ 
  if (hid_report) {
    // add to the hid transport
    switch(io_usb_hid_receive(io_usb_send_apdu_data, G_io_usb.ep_out_buff[2], hpcd_USB_OTG_FS.OUT_ep[2].xfer_count)) {
      default:
        break;

      case IO_USB_APDU_RECEIVED:
        G_io_apdu_length = G_io_usb_hid_total_length;
        break;
    }
  }
  return (0);
}


unsigned short io_exchange(unsigned char channel_and_flags, unsigned short tx_length) {
  return io_usb_hid_exchange(io_usb_send_apdu_data, tx_length, channel_and_flags);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
