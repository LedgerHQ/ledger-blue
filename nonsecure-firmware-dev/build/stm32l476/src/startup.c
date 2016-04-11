/*******************************************************************************
*   Ledger Blue - Non secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
********************************************************************************/

void Reset_Handler(volatile unsigned int button_press_duration);
void NMI_Handler(void);
void HardFault_Handler(void);
void Handler(void);
extern void SystemInit (void);

#include "bootloader.h"

bootloader_configuration_t N_bootloader_configuration;

extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;
extern unsigned long _estack;

void NMI_Handler (void) __attribute__ ((weak, alias ("Handler")));
void HardFault_Handler (void) __attribute__ ((weak, alias ("Handler")));
void MemManage_Handler (void) __attribute__ ((weak, alias ("Handler")));
void BusFault_Handler (void) __attribute__ ((weak, alias ("Handler")));
void UsageFault_Handler (void) __attribute__ ((weak, alias ("Handler")));
void SVC_Handler (void) __attribute__ ((weak, alias ("Handler")));
void DebugMon_Handler (void) __attribute__ ((weak, alias ("Handler")));
void PendSV_Handler (void) __attribute__ ((weak, alias ("Handler")));
void SysTick_Handler (void) __attribute__ ((weak, alias ("Handler")));
void WWDG_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void PVD_PVM_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TAMP_STAMP_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void RTC_WKUP_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void FLASH_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void RCC_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI0_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI4_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel4_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel5_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel6_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel7_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void ADC1_2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void CAN1_TX_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void CAN1_RX0_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void CAN1_RX1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void CAN1_SCE_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI9_5_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM1_BRK_TIM15_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM1_UP_TIM16_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM1_CC_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM4_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void I2C1_EV_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void I2C1_ER_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void I2C2_EV_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void I2C2_ER_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SPI1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SPI2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void USART1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void USART2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void USART3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void EXTI15_10_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void RTC_Alarm_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DFSDM3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM8_BRK_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM8_UP_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM8_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM8_CC_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void ADC3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void FMC_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SDMMC1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM5_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SPI3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void UART4_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void UART5_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM6_DAC_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TIM7_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel3_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel4_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel5_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DFSDM0_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DFSDM1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DFSDM2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void COMP_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void LPTIM1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void LPTIM2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void OTG_FS_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel6_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void DMA2_Channel7_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void LPUART1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void QUADSPI_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void I2C3_EV_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void I2C3_ER_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SAI1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SAI2_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void SWPMI1_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void TSC_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void LCD_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void RNG_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));
void FPU_IRQHandler (void) __attribute__ ((weak, alias ("Handler")));


void
Reset_Handler(volatile unsigned int button_press_duration)
{
    unsigned long *pulSrc, *pulDest;

#ifdef HAVE_BL
    // this is a boot sector to allow for bootloader upgrade
    if (N_bootloader_configuration.bootsector_magic == BOOTSECTOR_MAGIC) {
      // delegate boot to bootsector if magic is valid
      N_bootloader_configuration.bootsector_main();
    }
#endif // HAVE_BL

    // no init values in tacos, done explicitely by modules
    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

    // OTO: if not done, then the whole ST hal is pure junk, *Init are definitely well written ...
    //
    // Zero fill the bss segment.
    //
    for(pulDest = &_bss; pulDest < &_ebss; )
    {
        *pulDest++ = 0;
    }    

    // arm semihosting initialise_monitor_handles();

    //
    // Call the application's entry point.
    //
    main(button_press_duration);
}

void
Handler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
  &_estack
  ,Reset_Handler
  ,NMI_Handler
  ,HardFault_Handler
  ,MemManage_Handler
  ,BusFault_Handler
  ,UsageFault_Handler
  ,0
  ,0
  ,0
  ,0
  ,SVC_Handler
  ,DebugMon_Handler
  ,0
  ,PendSV_Handler
  ,SysTick_Handler
  ,WWDG_IRQHandler
  ,PVD_PVM_IRQHandler
  ,TAMP_STAMP_IRQHandler
  ,RTC_WKUP_IRQHandler
  ,FLASH_IRQHandler
  ,RCC_IRQHandler
  ,EXTI0_IRQHandler
  ,EXTI1_IRQHandler
  ,EXTI2_IRQHandler
  ,EXTI3_IRQHandler
  ,EXTI4_IRQHandler
  ,DMA1_Channel1_IRQHandler
  ,DMA1_Channel2_IRQHandler
  ,DMA1_Channel3_IRQHandler
  ,DMA1_Channel4_IRQHandler
  ,DMA1_Channel5_IRQHandler
  ,DMA1_Channel6_IRQHandler
  ,DMA1_Channel7_IRQHandler
  ,ADC1_2_IRQHandler
  ,CAN1_TX_IRQHandler
  ,CAN1_RX0_IRQHandler
  ,CAN1_RX1_IRQHandler
  ,CAN1_SCE_IRQHandler
  ,EXTI9_5_IRQHandler
  ,TIM1_BRK_TIM15_IRQHandler
  ,TIM1_UP_TIM16_IRQHandler
  ,TIM1_TRG_COM_TIM17_IRQHandler
  ,TIM1_CC_IRQHandler
  ,TIM2_IRQHandler
  ,TIM3_IRQHandler
  ,TIM4_IRQHandler
  ,I2C1_EV_IRQHandler
  ,I2C1_ER_IRQHandler
  ,I2C2_EV_IRQHandler
  ,I2C2_ER_IRQHandler
  ,SPI1_IRQHandler
  ,SPI2_IRQHandler
  ,USART1_IRQHandler
  ,USART2_IRQHandler
  ,USART3_IRQHandler
  ,EXTI15_10_IRQHandler
  ,RTC_Alarm_IRQHandler
  ,DFSDM3_IRQHandler
  ,TIM8_BRK_IRQHandler
  ,TIM8_UP_IRQHandler
  ,TIM8_TRG_COM_IRQHandler
  ,TIM8_CC_IRQHandler
  ,ADC3_IRQHandler
  ,FMC_IRQHandler
  ,SDMMC1_IRQHandler
  ,TIM5_IRQHandler
  ,SPI3_IRQHandler
  ,UART4_IRQHandler
  ,UART5_IRQHandler
  ,TIM6_DAC_IRQHandler
  ,TIM7_IRQHandler
  ,DMA2_Channel1_IRQHandler
  ,DMA2_Channel2_IRQHandler
  ,DMA2_Channel3_IRQHandler
  ,DMA2_Channel4_IRQHandler
  ,DMA2_Channel5_IRQHandler
  ,DFSDM0_IRQHandler
  ,DFSDM1_IRQHandler
  ,DFSDM2_IRQHandler
  ,COMP_IRQHandler
  ,LPTIM1_IRQHandler
  ,LPTIM2_IRQHandler
  ,OTG_FS_IRQHandler
  ,DMA2_Channel6_IRQHandler
  ,DMA2_Channel7_IRQHandler
  ,LPUART1_IRQHandler
  ,QUADSPI_IRQHandler
  ,I2C3_EV_IRQHandler
  ,I2C3_ER_IRQHandler
  ,SAI1_IRQHandler
  ,SAI2_IRQHandler
  ,SWPMI1_IRQHandler
  ,TSC_IRQHandler
  ,LCD_IRQHandler
  ,0
  ,RNG_IRQHandler
  ,FPU_IRQHandler
};
