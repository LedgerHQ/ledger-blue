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


#include "clock.h"
#include "hal.h"

// The value that will be load in the SysTick value register.
#define RELOAD_VALUE        (SYSCLK_FREQ/1000)-1   // One clock each 1 ms
#define RELOAD_VALUE_SLEEP  (SYSCLK_FREQ_SLEEP/1000)-1   // One clock each 1 ms


//const uint32_t CLOCK_SECOND = 1000;

extern __IO uint32_t uwTick;

#if 0
static volatile tClockTime count = 0;
/*---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
  count++;
}
#endif 

/*---------------------------------------------------------------------------*/

void Clock_Init(void)
{
  /*
  ATOMIC_SECTION_BEGIN();

  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_SetReload(RELOAD_VALUE);
  SysTick_CounterCmd(SysTick_Counter_Enable);
  SysTick_ITConfig(ENABLE);

  ATOMIC_SECTION_END();
  */
}

void Clock_Suspend(void)
{
  /*
  SysTick->CTRL = 0;
  SysTick_CounterCmd(SysTick_Counter_Clear);
  */
}

/*---------------------------------------------------------------------------*/

tClockTime Clock_Time(void)
{
  /*
  return count;
  */
  return uwTick;
}

/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 ms.
 *
 */
void Clock_Wait(uint32_t i)
{
  tClockTime start;

  start = Clock_Time();
  while(Clock_Time() - start < (tClockTime)i);
}
/*---------------------------------------------------------------------------*/

