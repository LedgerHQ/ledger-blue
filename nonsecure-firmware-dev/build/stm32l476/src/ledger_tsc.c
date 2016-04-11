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

#include "stm32l4xx_nucleo_bluenrg.h"


#include "stm32l4xx_hal.h"

#ifdef HAVE_TSC

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

#define KEY_MAPPING_CANCEL '\r'
#define KEY_MAPPING_OK '\n'
// maximum delay before a pressed/released touch key is sensed
#define TSC_ACQUISITION_LATENCY_MS 5UL


#ifndef NUCLEO_BASE
typedef struct tsc_bank_s {
  unsigned int channelIOs;
  unsigned int samplingIOs;
  unsigned int shieldIOs;
  unsigned int groupEnabled;
} tsc_bank_t;

/*
// 3*4 acquisitions
const tsc_bank_t tsc_banks[] = {
  {
    TSC_GROUP2_IO1|TSC_GROUP3_IO1|TSC_GROUP6_IO1|TSC_GROUP8_IO1, 
    TSC_GROUP2_IO4|TSC_GROUP3_IO4|TSC_GROUP6_IO4|TSC_GROUP8_IO4, 
    0, 
    TSC_GROUP2|TSC_GROUP3|TSC_GROUP6|TSC_GROUP8
  },
  {
    TSC_GROUP2_IO2|TSC_GROUP3_IO2|TSC_GROUP6_IO2|TSC_GROUP8_IO2, 
    TSC_GROUP2_IO4|TSC_GROUP3_IO4|TSC_GROUP6_IO4|TSC_GROUP8_IO4, 
    0, 
    TSC_GROUP2|TSC_GROUP3|TSC_GROUP6|TSC_GROUP8
  },
  {
    TSC_GROUP2_IO3|TSC_GROUP3_IO3|TSC_GROUP6_IO3|TSC_GROUP8_IO3, 
    TSC_GROUP2_IO4|TSC_GROUP3_IO4|TSC_GROUP6_IO4|TSC_GROUP8_IO4, 
    0, 
    TSC_GROUP2|TSC_GROUP3|TSC_GROUP6|TSC_GROUP8
  },
};
#define TSC_BANK_COUNT 3
*/

const tsc_bank_t tsc_banks[] = {
  {
    TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2, 
    TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3, 
    0, 
    TSC_GROUP1|TSC_GROUP2|TSC_GROUP3
  },
};
#define TSC_BANK_COUNT 1

#define TSC_KEY_MAPPING_NONE '\0'

#define TSC_DEBOUNCE_PER1000_DEFAULT 100 // 100 ok for st disco vampire with green pinpad, without ESD resistors

// define the hw channel to getc key mapping (all channels in to simplify the decoding)
const unsigned char tsc_key_mapping[] = {
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  '1',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  '2',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  '3',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  '4',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  '5',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  '6',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  '7',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  '8',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  '9',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
  KEY_MAPPING_OK,       TSC_DEBOUNCE_PER1000_DEFAULT,
  '0',                  TSC_DEBOUNCE_PER1000_DEFAULT,
  KEY_MAPPING_CANCEL,   TSC_DEBOUNCE_PER1000_DEFAULT,
  TSC_KEY_MAPPING_NONE, TSC_DEBOUNCE_PER1000_DEFAULT,
};

TSC_HandleTypeDef htsc;

struct tsc_s {
  unsigned int current_bank;
  #define TSC_CALIBRATION_CYCLES 10
  unsigned int calibration_cycles;

  
  // number of consecutive sampling before considering the channel value as effective
  #define TSC_DEBOUNCE_COUNT 3
  struct tsc_value_s {
    // TODO assert TSC_DEBOUNCE_COUNT <= 255
    unsigned char higher_debounce_count; // number of times a higher capacity has been presented consecutively
    unsigned char lower_debounce_count; // number of times a lower capacity has been presented consecutively
    unsigned short value; // last stable value
    unsigned short tmp_value; // last acquired value // DEBUG only
  } lastvalue[32]; // 32 channels at most
  // on bit per channel, 32 channels, 24 max possibl channels active
  unsigned int hw_cx_state;
} tsc;

void TSC_power(unsigned char powered) {
  GPIO_InitTypeDef GPIO_InitStruct;
  
#error ONOES

  if (powered) {
  /* USER CODE END TSC_MspInit 0 */
    /* Peripheral clock enable */
    __TSC_CLK_ENABLE();

  
    /**TSC GPIO Configuration    
    PA4     ------> TSC_G2_IO1
    PA5     ------> TSC_G2_IO2
    PA6     ------> TSC_G2_IO3
    PA7     ------> TSC_G2_IO4 samp
    PC5     ------> TSC_G3_IO1
    PB0     ------> TSC_G3_IO2
    PB1     ------> TSC_G3_IO3
    PB2     ------> TSC_G3_IO4 samp
    PB11     ------> TSC_G6_IO1
    PB12     ------> TSC_G6_IO2
    PB13     ------> TSC_G6_IO3
    PB14     ------> TSC_G6_IO4 samp
    PC6     ------> TSC_G8_IO1
    PC7     ------> TSC_G8_IO2
    PC8     ------> TSC_G8_IO3
    PC9     ------> TSC_G8_IO4 samp 
    */
/*
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12
                         |GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
*/
__GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*##-2- Configure Sampling Capacitor IOs (Alternate-Function Open-Drain) ###*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  /* Channel Sampling Capacitor IO : TSC_GROUP1_IO4 = PA3, TSC_GROUP2_IO4 = PA7 */
  GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* Channel Sampling Capacitor IO : TSC_GROUP3_IO3 = PB1 */
  GPIO_InitStruct.Pin       = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*##-3- Configure Channel IOs (Alternate-Function Output PP) ######*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  /* Channel IO = TSC_GROUP1_IO3 = PA2 */
  /* Channel IO = TSC_GROUP2_IO3 = PA6 */
  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* Channel IO = TSC_GROUP3_IO2 = PB0 */
  GPIO_InitStruct.Pin       = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /**Configure the TSC peripheral */
    htsc.Instance = TSC;
    htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
    /* 
    // around 0x130 for values, work for orig STM8L-EV1 board
    htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
    htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
    htsc.Init.MaxCountValue = TSC_MCV_511;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV2;
    */
    
    // around 0x120 on the blue
    htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
    htsc.Init.CTPulseLowLength = TSC_CTPL_4CYCLES;
    htsc.Init.MaxCountValue = TSC_MCV_16383;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV1;
    /*
    + MIN_DELTA = 0
    */

    /*
    // around 0xC0 for values, work for shitty double sided tape + petg 0.5mm
    // + MIN_DELTA = 1
    htsc.Init.CTPulseHighLength = TSC_CTPH_4CYCLES;
    htsc.Init.CTPulseLowLength = TSC_CTPL_4CYCLES;
    htsc.Init.MaxCountValue = TSC_MCV_2047;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV2;
    */
    htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
    htsc.Init.SpreadSpectrum = DISABLE;
    htsc.Init.MaxCountInterrupt = ENABLE;
    htsc.Init.ChannelIOs = 0; 
    htsc.Init.SamplingIOs = 0; 
    htsc.Init.ShieldIOs = 0;
    
    /*
    // STM32L053 demonstration
    TscHandle.Init.AcquisitionMode         = TSC_ACQ_MODE_NORMAL;
    TscHandle.Init.CTPulseHighLength       = TSC_CTPH_2CYCLES;
    TscHandle.Init.CTPulseLowLength        = TSC_CTPL_2CYCLES; 
    TscHandle.Init.IODefaultMode           = TSC_IODEF_IN_FLOAT;
    TscHandle.Init.MaxCountInterrupt       = DISABLE;
    TscHandle.Init.MaxCountValue           = TSC_MCV_8191;
    TscHandle.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV32; 
    TscHandle.Init.SpreadSpectrum          = DISABLE;
    TscHandle.Init.SpreadSpectrumDeviation = 127;
    TscHandle.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
    TscHandle.Init.SynchroPinPolarity      = TSC_SYNC_POL_FALL;
    */
    htsc.Init.SpreadSpectrumDeviation = 127;
    htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
    htsc.Init.SynchroPinPolarity      = TSC_SYNC_POL_FALL;

    htsc.Init.CTPulseHighLength       = 1; //TSC_CTPH_1CYCLES;
    htsc.Init.CTPulseLowLength        = 1; //TSC_CTPL_1CYCLES; 
    htsc.Init.MaxCountInterrupt       = DISABLE;
    htsc.Init.MaxCountValue           = TSC_MCV_16383;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV8; 
    //htsc.Init.IODefaultMode           = TSC_IODEF_IN_FLOAT;

    htsc.Init.CTPulseHighLength       = 2; //TSC_CTPH_1CYCLES;
    htsc.Init.CTPulseLowLength        = 2; //TSC_CTPL_1CYCLES; 
    htsc.Init.MaxCountInterrupt       = DISABLE;
    htsc.Init.MaxCountValue           = TSC_MCV_16383;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV16; 
    //htsc.Init.IODefaultMode           = TSC_IODEF_IN_FLOAT;


    htsc.Init.CTPulseHighLength       = TSC_CTPH_2CYCLES;
    htsc.Init.CTPulseLowLength        = TSC_CTPL_2CYCLES; 
    htsc.Init.MaxCountInterrupt       = DISABLE;
    htsc.Init.MaxCountValue           = TSC_MCV_16383;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV32; 
    //htsc.Init.IODefaultMode           = TSC_IODEF_IN_FLOAT;

    HAL_TSC_Init(&htsc);
    
    // init TSC software state
    memset(&tsc,0,sizeof(tsc));
    // enable end of acq interrupt
    NVIC_EnableIRQ(TSC_IRQn);
  }
  else {
    NVIC_DisableIRQ(TSC_IRQn);
    
    __TSC_CLK_DISABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // ensure avoiding wasting energy
    UI_led_power(0);
  }
}


// to be called by the global TSC ticker to proceed with a new acquisition
void tsc_next_acq(void) {
  // setup channel, sampling and shield ios according to current tick value to select a group accordingly
  
  // aquisition not finished, can't start a new one, the hardware will tell
  if (htsc.Instance->CR & TSC_CR_START) {
    return;
  }
  
  __HAL_LOCK(&htsc);
  
  
  /* Disable Schmitt trigger hysteresis on all used TSC IOs */
  htsc.Instance->IOHCR = (uint32_t)(~(tsc_banks[tsc.current_bank].channelIOs | tsc_banks[tsc.current_bank].samplingIOs | tsc_banks[tsc.current_bank].shieldIOs));

  /* Set channel and shield IOs */
  htsc.Instance->IOCCR = (tsc_banks[tsc.current_bank].channelIOs | tsc_banks[tsc.current_bank].shieldIOs);
  
  /* Set sampling IOs */
  htsc.Instance->IOSCR = tsc_banks[tsc.current_bank].samplingIOs;
  
  /* Set the groups to be acquired */
  htsc.Instance->IOGCSR = tsc_banks[tsc.current_bank].groupEnabled;
  
  // prepare for next bank
  tsc.current_bank = (tsc.current_bank+1)%TSC_BANK_COUNT;
  
  // setup calibration 
  if (tsc.current_bank == 0) {
    // end of calibration phase
    if (tsc.calibration_cycles < TSC_CALIBRATION_CYCLES) {
      tsc.calibration_cycles++;
    }
    else {
    // TODO if in burst mode:
    // bank0bank1bank...bankN [wait] bank0bank1bank...bankN ... 
    // then space the timer
    }
  }
  else {
    if (tsc.calibration_cycles >= TSC_CALIBRATION_CYCLES) {
    // TODO if in burst mode:
    // bank0bank1bank...bankN [wait] bank0bank1bank...bankN [wait] ...
    // then reduce the timer latency (we're within a burst)      
    }
  }
  
  // prepare the hard macrocell it acquisition mode
  //HAL_TSC_Start_IT(&htsc);  
  
  /* Process locked */
  
  /* Change TSC state */
  htsc.State = HAL_TSC_STATE_BUSY;
  
  /* Clear flags */
  __HAL_TSC_CLEAR_FLAG(&htsc, (TSC_FLAG_EOA | TSC_FLAG_MCE));

  /* Enable end of acquisition interrupt */
  __HAL_TSC_ENABLE_IT(&htsc, TSC_IT_EOA);

  /* Enable max count error interrupt (optional) */
  if (htsc.Init.MaxCountInterrupt == ENABLE)
  {
    __HAL_TSC_ENABLE_IT(&htsc, TSC_IT_MCE);
  }
  else
  {
    __HAL_TSC_DISABLE_IT(&htsc, TSC_IT_MCE);
  }

  /* Stop discharging the IOs */
  __HAL_TSC_SET_IODEF_INFLOAT(&htsc);

  /* Launch the acquisition */
  __HAL_TSC_START_ACQ(&htsc);

  /* Process unlocked */
  __HAL_UNLOCK(&htsc);
}

void tsc_channel_event(void);

void tsc_end_acq(void) {

  __HAL_LOCK(&htsc);

  // output pp low to discharge all capacitors
  __HAL_TSC_SET_IODEF_OUTPPLOW(&htsc);


  // browse each channel of each group, computing delta with previous value and determining if channel is on of off.
  for (unsigned int group=0;group<8;group++) {
    // group is not involved in the acquisition round
    if (! (htsc.Instance->IOGCSR & (1<<group)) ) {
      continue;
    }
    
    unsigned int value = HAL_TSC_GroupGetValue(&htsc, group);
    // determine which channel was currently acquired in that group
    for (unsigned int channel=0; channel<4; channel++) {
      unsigned int hw_cx = ((1<<channel)<<(group*4));
      if(htsc.Instance->IOCCR & hw_cx) {
        unsigned int sw_cx = channel+(group*4);
        unsigned int lastvalue = tsc.lastvalue[sw_cx].value;
        tsc.lastvalue[sw_cx].tmp_value = value;
        if (lastvalue == 0 || tsc.calibration_cycles < TSC_CALIBRATION_CYCLES) {
          tsc.hw_cx_state &= ~(hw_cx);
          // TODO averaging of calibration cycles
        update_channel_value:
          tsc.lastvalue[sw_cx].value = value;
          tsc.lastvalue[sw_cx].higher_debounce_count = 0;
          tsc.lastvalue[sw_cx].lower_debounce_count = 0;
          
          //callback after a channel change
          tsc_channel_event();
        }
        else if (value < (1000-tsc_key_mapping[sw_cx*2+1] )*lastvalue/1000){
          tsc.lastvalue[sw_cx].higher_debounce_count++;
          tsc.lastvalue[sw_cx].lower_debounce_count=0;
          // value considered ok
          if(tsc.lastvalue[sw_cx].higher_debounce_count >= TSC_DEBOUNCE_COUNT) {
            tsc.hw_cx_state |= hw_cx;
            goto update_channel_value;
          }
          /* don't change state until debounced
          else {
            tsc.hw_cx_state &= ~(hw_cx);
          }
          */
        }
        else if (value > (1000+tsc_key_mapping[sw_cx*2+1])*lastvalue/1000) {
          tsc.lastvalue[sw_cx].higher_debounce_count=0;
          tsc.lastvalue[sw_cx].lower_debounce_count++;
          // value considered ok
          if(tsc.lastvalue[sw_cx].lower_debounce_count >= TSC_DEBOUNCE_COUNT) {
            tsc.hw_cx_state &= ~(hw_cx);
            goto update_channel_value;
          }
          /* don't change state until debounced
          else {
            tsc.hw_cx_state |= hw_cx;
          }
          */
        }
        
#ifdef TSC_SINGLE_CHANNEL_PER_GROUP_AT_ONCE
        break;
#endif // TSC_SINGLE_CHANNEL_PER_GROUP_AT_ONCE
      }
    }
  }
  __HAL_UNLOCK(&htsc);
}


void TSC_IRQHandler(void) {
  HAL_TSC_IRQHandler(&htsc);
}

// called upon TSC acquisition finished
void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc) {
  tsc_end_acq();
}

void HAL_TSC_ErrorCallback(TSC_HandleTypeDef* htsc) {
  tsc_end_acq();
}

/**
 * Touch sensing key detection 
 * DESIGN NOTE: new pressed keys are appended to the getc buffer, long press are not meaningfull
 */
unsigned int previous_tcs_hw_cx_state;
//unsigned char toggle;
//unsigned int tsc_ignored_channels;

void tsc_channel_event(void) {
  
  /*
  // mask ignored channels for the current session (easier to select set of allowed touchs)
  tsc.hw_cx_state &= ~tsc_ignored_channels;
  */
  
  // compure the newly pressed and released channels
  unsigned int pressed_tsc = (tsc.hw_cx_state ^ previous_tcs_hw_cx_state) & tsc.hw_cx_state;
  //unsigned int released_tsc = (tsc.hw_cx_state ^ previous_tcs_hw_cx_state) & previous_tcs_hw_cx_state;

  /*
  if (tsc.hw_cx_state) {
    UI_led_power(toggle);
    toggle ^= 1;
  }
  */
  
  // if any newly pressed touch, then blink
  if (pressed_tsc) {
#ifdef UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES
    UI_led_remaining_ticks = UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES;
    UI_led_power(1);
#endif //UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES

    // execute key mapping
    for(unsigned int i = 0; i < 32; i++) {
      if (pressed_tsc & (1<<i)) {
        unsigned char key = tsc_key_mapping[i*2];
        UI_keyboard_press(key);
      }
    }
  }
  
  // remember the last processed channels state
  previous_tcs_hw_cx_state =  tsc.hw_cx_state;
}
#endif // NUCLEO_BASE
 

#endif // HAVE_TSC
