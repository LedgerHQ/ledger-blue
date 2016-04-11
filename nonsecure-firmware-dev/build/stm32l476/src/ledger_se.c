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

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

#include "stm32l4xx_nucleo_bluenrg.h"


#include "stm32l4xx_hal.h"

 // not used for now
#define THROW(x) for(;;);

//#define USE_USART_DMA
#define SC_PPS_SUPPORT

#ifdef HAVE_SE
extern USART_HandleTypeDef G_io_se_usart;
extern volatile unsigned char G_io_se_powered;

extern unsigned char G_io_apdu_buffer[260];
extern unsigned short G_io_apdu_length;

#ifndef USE_USART_DMA
unsigned int G_io_se_discard_rx;
#endif

void SE_iso_disable_rx(void) {
#ifdef USE_USART_DMA
  DMA1_Channel5->CCR &= ~DMA_CCR_EN;
#else
  G_io_se_usart.Instance->CR1 &= ~USART_CR1_UE;
  G_io_se_usart.Instance->CR1 = (G_io_se_usart.Instance->CR1&(~USART_CR1_RE))|USART_CR1_UE;
  //G_io_se_discard_rx = 1;
#endif
}

void SE_iso_enable_rx(void) {
  // consume overrun byte if any
  volatile unsigned char c = G_io_se_usart.Instance->RDR;
#ifdef USE_USART_DMA
  DMA1_Channel5->CCR |= DMA_CCR_EN;
#else
  G_io_se_usart.Instance->CR1 &= ~USART_CR1_UE;
  G_io_se_usart.Instance->CR1 |= USART_CR1_RE|USART_CR1_UE;
  //G_io_se_discard_rx = 0;
#endif
}

// =========================================================================================
// ISO7816

// interrupt method for usart reply of the SE
#ifdef USARTINT
void SE_usart_interrupt(unsigned char c) {
  // until L field is received (offset == 0, construct the )
  switch(G_io_se_apdu_transport_offset) {
    case 0:
      G_io_se_apdu_transport_remlen = c;
      break;
    case 1:
      // compute transport remaining length
      G_io_se_apdu_transport_remlen = (G_io_se_apdu_transport_remlen<<8) | c&0xFF;
      // compute apdu length
      G_io_apdu_length = G_io_se_apdu_transport_remlen;
      break;
    default:
      G_io_apdu_buffer[G_io_se_apdu_transport_offset-2]=c;
      G_io_se_apdu_transport_remlen--;
      break;
  }
  G_io_se_apdu_transport_offset++;
}

#endif // USARTINT

//#define SE_PWR(x) BB_OUT(GPIOC,10 /*PC10*/, x)
#define SE_RST(x) BB_OUT(GPIOC,12 /*PC12*/, x)

/**
 * This function set the current active Smartcard link within low power (disable the clock generation and so on).
 */
void SE_lowpower(unsigned char lowpowerenabled) {
  if (lowpowerenabled) {
    __USART1_CLK_DISABLE();
  }
  else {
    __USART1_CLK_ENABLE();
  }
}

// convert Fi into ETU's tick count
const unsigned short ISO7816_TA_F[] = {
  372,
  372,
  558,
  744,
  1116,
  1488,
  1860,
  0,
  0,
  512,
  768,
  1024,
  1536,
  2048,
  2304, // out of spec, just for etu9 (0xE9)
  0,
};
// convert TA to ETU clk ticks
#define SE_baudrate_ETU(TA) ((ISO7816_TA_F[TA>>4]) >> ((TA&0x0F)-1))

unsigned char G_io_se_state;
unsigned char G_io_se_atr[40];
unsigned short G_io_se_atr_length;
unsigned char G_io_se_PPS[4];
unsigned char G_io_se_PPSR[4];

void SE_update_baudrate(unsigned char TA) {
  __HAL_USART_DISABLE(&G_io_se_usart);
    // set baudrate
  // multuply by prescaler to take into account real clock on the clk line
  G_io_se_usart.Instance->BRR = (SE_baudrate_ETU(TA)<<1 /*etu=8 <-> 0x10*/) * ((G_io_se_usart.Instance->GTPR&0xFF)<<1);  
  /* Enable the Peripharal */
  __HAL_USART_ENABLE(&G_io_se_usart); 
}

extern volatile unsigned int frequency_hz;
void SE_set_link_speed(unsigned int mhz, unsigned int etu) {
  __HAL_USART_DISABLE(&G_io_se_usart);
  //G_io_se_usart.Instance->GTPR = (frequency_hz/2)/mhz/1000000; // compute link frequency from sysclk and requested clock
  etu = (etu<<1/*etu=8 <-> 0x10*/) * ((G_io_se_usart.Instance->GTPR&0xFF)<<1);
  G_io_se_usart.Instance->BRR = etu; //(etu&0xFFF0) | (etu&0xF)>>1;
  __HAL_USART_ENABLE(&G_io_se_usart); 
}

#ifndef USE_USART_DMA
volatile unsigned int G_io_se_offset_write;
volatile unsigned int G_io_se_length;
volatile unsigned int G_io_se_offset_read;
#else
volatile unsigned int G_io_se_last_cndtr;
#endif // 
volatile unsigned char G_io_se_buffer[300];

#ifndef USE_USART_DMA
void USART1_IRQHandler(void) {
  
  // mark isr as serviced
  NVIC_ClearPendingIRQ(USART1_IRQn);

  // clear all it flags.
  G_io_se_usart.Instance->ICR = 0xFFFFFFFF;
  
  // store the byte
  if (
#ifndef USE_USART_DMA
      !G_io_se_discard_rx && 
#endif
      G_io_se_length < sizeof(G_io_se_buffer)) {

    // DEBUG toggle on data
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));

    G_io_se_buffer[G_io_se_offset_write] = G_io_se_usart.Instance->RDR;
    G_io_se_offset_write = (G_io_se_offset_write+1)%sizeof(G_io_se_buffer);
    G_io_se_length++;
  }
  else {
    // else ignore the new byte
    volatile unsigned char c = G_io_se_usart.Instance->RDR;
  }
}
#endif // USE_USART_DMA

unsigned short io_seproxyhal_rx_available(void) {
  #ifdef USE_USART_DMA
  return ((sizeof(G_io_se_buffer)-DMA1_Channel5->CNDTR) + sizeof(G_io_se_buffer) - G_io_se_last_cndtr) % (sizeof(G_io_se_buffer));
  #else
  return G_io_se_length;
  #endif // USE_USART_DMA
}

#define NO_TIMEOUT (-1UL)
void SE_iso_recv(unsigned char* buffer, unsigned short length, unsigned int timeout) {
  // well, too bad for wrongly formatted packets length = ((G_io_se_length)<(length))?G_io_se_length:length;
  while(length--) {
    // wait for a byte
    #ifdef USE_USART_DMA
    while(DMA1_Channel5->CNDTR == G_io_se_last_cndtr && ( (timeout== -1) || --timeout ));
    if (timeout == 0) {
      return 0;
    }
    // compute next value (avoid )
    G_io_se_last_cndtr = (G_io_se_last_cndtr-1)%sizeof(G_io_se_buffer);

    *buffer++ = G_io_se_buffer[sizeof(G_io_se_buffer)-G_io_se_last_cndtr-1];

    // reset the cndtr for next read (consider autoreload of the DMA register)
    if (G_io_se_last_cndtr==0) {
      G_io_se_last_cndtr=sizeof(G_io_se_buffer);
    }
    #else
    // TODO low power
    while(G_io_se_length == 0 && ( (timeout== -1) || --timeout ));
    if (timeout == 0) {
      return 0;
    }
    __asm("cpsid i");
    G_io_se_length--;
    __asm("cpsie i");

    *buffer++ = G_io_se_buffer[G_io_se_offset_read];
    G_io_se_offset_read = (G_io_se_offset_read+1)%sizeof(G_io_se_buffer);
    #endif // USE_USART_DMA

  }

  return 1;
}


unsigned short io_seproxyhal_recv(unsigned char* buffer, unsigned short length) {
  SE_iso_recv(buffer, 3, NO_TIMEOUT);
  //PRINTF("iso recv: HDE  %.*H\n", 3, buffer);
  length = (buffer[1]<<8)|(buffer[2]&0xFF);
  SE_iso_recv(buffer+3, length, NO_TIMEOUT);
  //PRINTF("iso recv: DATA %.*H ", length, buffer+3);
  //screen_printf("%.*H ", length+3, buffer);
  return length + 3;
}

void SE_iso_send(unsigned char* buffer, unsigned short length) {
  unsigned short l = length;
  // TODO low power

  //screen_printf("%.*H ", length, buffer);

  //HAL_NVIC_DisableIRQ(USART1_IRQn);
  
  // avoid killing RE if no byte to be sent
  if (length) {
    SE_iso_disable_rx();
    
    while(length--) {
      while(!(G_io_se_usart.Instance->ISR & USART_FLAG_TXE));
      G_io_se_usart.Instance->TDR = *buffer++;
      /*
      // read the sent byte
      while(!(G_io_se_usart.Instance->ISR & USART_FLAG_RXNE));
      volatile unsigned char c = G_io_se_usart.Instance->TDR;
      NVIC_ClearPendingIRQ(USART1_IRQn);
      G_io_se_usart.Instance->ICR = 0xFFFFFFFF;
      HAL_NVIC_EnableIRQ(USART1_IRQn);
      */
    }

    // this trick avoid to wait for guard time on last param, 
    // and be sure to receive the first character coming from 
    // the SE, even after 2 stop bits.
    while(!(G_io_se_usart.Instance->ISR & USART_FLAG_TC));

    SE_iso_enable_rx();
  }

#ifdef USE_USART_DMA
  G_io_se_last_cndtr = (sizeof(G_io_se_buffer)+G_io_se_last_cndtr-length)%sizeof(G_io_se_buffer);
#endif // USE_USART_DMA

}

void io_seproxyhal_send(unsigned char* buffer, unsigned short length) {

  /* can be sent in multiple times
  // check packet consistency
  if (length != ((buffer[1]<<8)|(buffer[2]&0xFF))+3) {
    return;
  }
  */
  
  //PRINTF("iso send: %.*H\n", length, buffer);
  SE_iso_send(buffer, length);
}

/* @param force_ta1: 0 means auto, 1 means force ATR TA1, other fixes the TA1 maximal value */
unsigned char SE_iso_power_up(unsigned char force_ta1) {

  // request ATR of the second chip
  unsigned char T_not_0 = 0;
  unsigned char lastTD = 0;
  unsigned char TA[2];
  unsigned char i;

  G_io_se_state = 0;
  G_io_se_atr_length = 2; // ensure debugging status to display the 2 first bytes received in case of delay
  memset(G_io_se_atr, 0, sizeof(G_io_se_atr));

  // upon invalid TCK, then desactivate bridge ? NO ! could be useful to have a non iso reader :)
  SE_update_baudrate(0x11); // TA=11 <=> ETU=372
    
  // set power and reset pins to high level
  
  SE_RST(0);
  //SE_PWR(1);
  // valid delay @ 4mhz
  HAL_Delay(100);
  SE_RST(1);

  // receive TS and T0
  SE_iso_recv(G_io_se_atr, 2, 0x10000);
  lastTD = G_io_se_atr[1];
  G_io_se_atr_length=2;

  if (G_io_se_atr[0] != 0x3b && G_io_se_atr[0] != 0x3f) {
    return 0;
  }

  // receive all atr T* bytes (decoding it :p)
  // note i are translated for -1.
  i = 0;
  TA[0] = 0;
  TA[1] = 0;
  while(lastTD&0xF0) {
    // TAi
    if(lastTD&0x10) {
      SE_iso_recv(G_io_se_atr+G_io_se_atr_length, 1, NO_TIMEOUT);
      // store TA
      if (i < 2) {
        TA[i] = G_io_se_atr[G_io_se_atr_length];
      }
      G_io_se_atr_length++;
    }
    // TBi
    if(lastTD&0x20) {
      SE_iso_recv(G_io_se_atr+G_io_se_atr_length, 1, NO_TIMEOUT);
      G_io_se_atr_length++;
    }
    // TCi
    if(lastTD&0x40) {
      SE_iso_recv(G_io_se_atr+G_io_se_atr_length, 1, NO_TIMEOUT);
      G_io_se_atr_length++;
    }
    // TDi
    if(lastTD&0x80) {
      SE_iso_recv(G_io_se_atr+G_io_se_atr_length, 1, NO_TIMEOUT);
      lastTD = G_io_se_atr[G_io_se_atr_length];
      G_io_se_atr_length++;
      i++;

      // check need for TCK (not in T0, which encode K instead of a protocol)
      if (! T_not_0 && (lastTD&0xF) != 0) {
        T_not_0 = 1;
      }
    }
    else {
      // no TDi+1, therefore stop receiving TD
      lastTD = 0;
    }
  }

  G_io_se_state = 1;

  // receive historical bytes
  if ((G_io_se_atr[1] & 0xF)) {
    SE_iso_recv(G_io_se_atr+G_io_se_atr_length, G_io_se_atr[1] & 0xF, NO_TIMEOUT);
    G_io_se_atr_length += (G_io_se_atr[1] & 0xF);
  }

  G_io_se_state = 2;

  // receive TCK byte if mandatory
  if (T_not_0) {
    uint8_t tck;
    SE_iso_recv(G_io_se_atr+G_io_se_atr_length, 1, NO_TIMEOUT);
    G_io_se_atr_length++;

    // check TCK
    tck = 0;
    for (lastTD = 0; lastTD < G_io_se_atr_length ; lastTD++) {
      tck ^= G_io_se_atr[lastTD];
    }

    if (tck != 0) {
      // invalid tck use ccid atr and disable bridge
      // TODO // THROW(NOT_SUPPORTED);
    }
  }

  G_io_se_state = 3;

  #define TA1 TA[0]
  #define TA2 TA[1]
  if (TA2 != 0 && TA2 != 0x80) {
    G_io_se_state = 4;
                
    // unsupported ATR
    return 0;
  }
  else if (TA2) {
    // specific mode, apply TA1 (never issue a pps even if forced)
    SE_update_baudrate(TA1);
  }
#ifdef SC_PPS_SUPPORT
  else if ((TA1 != 0x11 && TA1 != 0) || force_ta1) {

    // use the higher etu in case requested (baudrate is not baudrate but ETU)
    if (SE_baudrate_ETU(force_ta1) > SE_baudrate_ETU(TA1)) {
      TA1 = force_ta1;
    }

    // negociable mode (PPS exchange)
    G_io_se_PPS[0] = 0xFF;
    G_io_se_PPS[1] = 0x10;
    G_io_se_PPS[2] = TA1;
    G_io_se_PPS[3] = G_io_se_PPS[0]^G_io_se_PPS[1]^G_io_se_PPS[2];

    // send PPS
    SE_iso_send(G_io_se_PPS, 4);

    G_io_se_state = 5;

    // receive PPS answer
    // PPSS/PPS0
    SE_iso_recv(G_io_se_PPSR, 2, NO_TIMEOUT);
    // check PPSS of the reply
    if (G_io_se_PPSR[0] != 0xFF) {
      return 0;
    }
    // interpret PPS0
    TA1 = 0x11; // in case error, use default PPS
    switch(G_io_se_PPSR[1]) {
      
      default:
        G_io_se_state = 6;
        return 0;

      // PPS refused
      case 0:
        G_io_se_state = 7;
        // PCK
        SE_iso_recv(G_io_se_PPSR+2, 1, NO_TIMEOUT);
        break;
      
      // PPS accepted
      case 0x10:
        G_io_se_state = 8;
        // PPSA/PCK
        SE_iso_recv(G_io_se_PPSR+2, 2, NO_TIMEOUT);
        TA1 = G_io_se_PPSR[2];
        break;
    }
    SE_update_baudrate(TA1);
  }
#endif // SC_PPS_SUPPORT
  
  // TODO wait some times before sending the first command

  return 1; // success
}

unsigned char SE_iso_power(unsigned char powered) {
  GPIO_InitTypeDef GPIO_InitStruct;
  
  if (powered) {
    // firstly setup the usart peripheral to avoid invalid gpio line state
    // setup gpio dedication to usart
    /* Configure USART1 CLOCK */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; // high speed for ETU16/8
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure USART1 RX */
    /* Configure USART1 TX */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // use PP to lower the rise effect and the need for a strong pull with ETU16/8
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // DESIGN NOTE: all A output bits are low at startup to avoid extra boots
    
    /* Configure USART1 RESET */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    //SE_PWR(0);
    SE_RST(0);
    
    // power the USART peripheral
    __USART1_CLK_ENABLE();
    
    // ensure to avoid invalid values into the USART config
    memset(&G_io_se_usart, 0, sizeof(G_io_se_usart));
    
    // start usart1 operation
    G_io_se_usart.Instance = USART1;
    //G_io_se_usart.Init.BaudRate = 115200;
    G_io_se_usart.Init.WordLength = USART_WORDLENGTH_8B;
    G_io_se_usart.Init.StopBits = USART_STOPBITS_1_5;
    G_io_se_usart.Init.Parity = USART_PARITY_EVEN;
    G_io_se_usart.Init.Mode = USART_MODE_TX_RX;
    G_io_se_usart.Init.CLKPolarity = USART_POLARITY_LOW;
    G_io_se_usart.Init.CLKPhase = USART_PHASE_1EDGE;
    G_io_se_usart.Init.CLKLastBit = USART_LASTBIT_ENABLE;
    HAL_USART_Init(&G_io_se_usart);
    
    /* Disable the Peripheral */
    __HAL_USART_DISABLE(&G_io_se_usart);

    /* set the USART in SMARTCARD mode */ 
    /* setup Wakeup upon RXNE */
    G_io_se_usart.Instance->CR1 |= USART_CR1_M0 /*| USART_CR1_OVER8*/;
    G_io_se_usart.Instance->CR2 |= USART_CR2_CLKEN; 
    //G_io_se_usart.Instance->CR2 &= ~ (USART_CR2_TXINV|USART_CR2_RXINV|USART_CR2_SWAP|USART_CR2_DATAINV);
    G_io_se_usart.Instance->CR3 |= /*USART_CR3_NACK |*/ USART_CR3_SCEN | USART_CR3_WUS_0 | USART_CR3_WUS_1 | USART_CR3_ONEBIT; 
    
    // depending on the clock source (usb enabled or not) the gtpr prescaler must be set to avoid overclocking
    // OK MSI // G_io_se_usart.Instance->GTPR = 1; // divide clock by 2 with no guard time value // 5 -> ck = 3.2mhz
    //G_io_se_usart.Instance->GTPR = 3; // /6  2.6mhz
    //G_io_se_usart.Instance->GTPR = 2; // /4  4mhz
    //G_io_se_usart.Instance->GTPR = 1; // /2  8mhz // OK 
#ifdef HAVE_BL
    G_io_se_usart.Instance->GTPR = 3; // /6  10mhz // OK ETU16
#else
    G_io_se_usart.Instance->GTPR = 3;
#endif // HAVE_BL
    //G_io_se_usart.Instance->GTPR = 2; // /4  15mhz // OK ETU8 (even if out of spec)
    //G_io_se_usart.Instance->GTPR = 1; // /2  20mhz // FEIL ETU32


#ifdef USE_USART_DMA
    __DMA1_CLK_ENABLE();

    // channel 5 map onto USART1_RX
    DMA1_CSELR->CSELR |= DMA_REQUEST_2 << 16;

    G_io_se_usart.Instance->CR3 |= USART_CR3_DMAR;

    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_1;
    // transfer at most size of SE buffer
    DMA1_Channel5->CNDTR = sizeof(G_io_se_buffer);
    // from RX
    DMA1_Channel5->CPAR = &USART1->RDR;
    // to buffer
    DMA1_Channel5->CMAR = G_io_se_buffer;

    // enable dma
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    G_io_se_last_cndtr = sizeof(G_io_se_buffer);
#else
    // setup interrupt upon reception
    __HAL_USART_ENABLE_IT(&G_io_se_usart, USART_IT_RXNE);
    
    // setup interruption for usart
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    G_io_se_discard_rx = 0;
    G_io_se_length = G_io_se_offset_write = G_io_se_offset_read = 0;
#endif
    
    /* Enable the Peripharal */
    __HAL_USART_ENABLE(&G_io_se_usart);
    
    // grab ATR (force pps even if not needed)
    //return SE_iso_power_up(0x95); // ETU32
#ifdef HAVE_BL
    return SE_iso_power_up(0x96); // ETU16 // OK tested
#else // HAVE_BL
    //return SE_iso_power_up(0x96); // ETU16 // OK tested
    return SE_iso_power_up(0x97); // ETU8 // some feil with invalid byte read from the SE
#endif // HAVE_BL
    //return SE_iso_power_up(0xC6); // ETU16
    //return SE_iso_power_up(0xC8); // ETU12
  }
  else {

#ifdef USE_USART_DMA
    DMA1_Channel5->CCR = 0;
#endif // USE_USART_DMA

    // power off the usart, set all gpios to analog input
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    
    // set power and reset pins to high level
    SE_RST(0);
    //SE_PWR(0);
    
    __USART1_CLK_DISABLE();
    
    // deconfigure pins
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    return 1; // always executed
  }
}


#define STATE_COMMAND 0
#define STATE_SW2 1
/**
 * Exchange an APDU with the secure element
 */
unsigned short SE_iso_exchange_apdu(unsigned char* apdu, unsigned short length) {

  unsigned char cmd;
  unsigned char state;
  unsigned char ins;
  
  if (length < 5) {
    THROW(NOT_SUPPORTED);
  }

  // get the command size
  state = STATE_COMMAND;
  // default is case 1 / 2 (reception only)
  ins = apdu[1];

  // forward header to the ISOMaster
  SE_iso_send(apdu, 5);
  // consume header
  length-=5;
  // default is case 1
  G_io_apdu_length = 0;
  
  // execute protocol
  while(1) {
    // TODO timeout instead of 1
    
    // sw1 / ins
    SE_iso_recv(&cmd, 1, NO_TIMEOUT);

    // TODO upon timeout, return fake SW and disable bridging

    switch(state) {
      case STATE_COMMAND:
        switch (cmd & 0xF0) {
          case 0x90:
          case 0x60:
            // wait extension (TODO reset the timeout)
            if (cmd == 0x60) {
              continue;
            }
            G_io_apdu_buffer[G_io_apdu_length++] = cmd;
            state = STATE_SW2;
            // switch to SW2 reception
            continue;
          default:
            if (cmd == ins) {
              state = STATE_COMMAND;
              if (length > 0) {
                // P3 is Lc
                SE_iso_send(apdu+5,apdu[4]);
              }
              else {
                G_io_apdu_length = apdu[4]; // P3 is Le
                SE_iso_recv(G_io_apdu_buffer, G_io_apdu_length, NO_TIMEOUT);
              }
              // switch to COMMAND reception
              continue;
            }
            else {
              // invalid command, byte per byte not supported yet
              THROW(NOT_SUPPORTED);
            }
            break;
        }
        break;
      case STATE_SW2:
        G_io_apdu_buffer[G_io_apdu_length++] = cmd;
        return G_io_apdu_length;
    }
  }
}

#if 0
// =========================================================================================
// ISO2C
unsigned char G_seproxyhal_buffer[300];

#define SE_ISO2C_RST_HIGH() BB_OUT(GPIOC, 12, 1)
#define SE_ISO2C_RST_LOW() BB_OUT(GPIOC, 12, 0)

#define SE_ISO2C_CLK_WAIT_HIGH() while(!BB_IN(GPIOA, 8))

#define SE_ISO2C_CLK_WAIT_LOW() while(BB_IN(GPIOA, 8))

void SE_ISO2C_CLK_MODE_ISR(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Configure USART1 CK (PA8) IT RISING*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SE_ISO2C_CLK_MODE_INPUT(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Configure USART1 CK (PA8) IT RISING*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SE_ISO2C_CLK_MODE_USART(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Configure USART1 CK (PA8) */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SE_iso2c_isr_clk_rise(void) {
  unsigned short offset=0;
  unsigned short length=300;
  // perform reception of the whole packet
  // TODO DMA here for low power

  // wait slave ack our notification
  //  assert for ir triggering // SE_ISO2C_CLK_WAIT_HIGH();
  // prepare to iart
  SE_ISO2C_CLK_MODE_USART();

  do {
    // receive a byte
    while(!(G_io_se_usart.Instance->ISR & USART_FLAG_RXNE));
    G_io_apdu_buffer[offset++] = G_io_se_usart.Instance->RDR;
    // decode length, if it's time to
    switch(offset) {
      case 3:
        // setup length
        length = (G_io_apdu_buffer[1]<<8) | (G_io_apdu_buffer[2]&0xFF);
        break;
    }
  }
  while(offset < length);

  // prepare for SE transfer notification
  SE_ISO2C_CLK_MODE_INPUT();
  // wait until CLK goes low (slave end of packet notification) to avoid race conditions
  SE_ISO2C_CLK_WAIT_LOW();
  // prepare for SE transfer notification
  SE_ISO2C_CLK_MODE_ISR();

  //seproxyhal_packet(G_io_apdu_buffer, length);
}

void SE_iso2c_send(unsigned char* buffer, unsigned short length) {
  // avoid interrupt
  SE_ISO2C_CLK_MODE_INPUT();
  // notify we have data to send
  SE_ISO2C_RST_HIGH();
  // wait slave ack our notification
  SE_ISO2C_CLK_WAIT_HIGH();
  // prepare to iart
  SE_ISO2C_CLK_MODE_USART();

  // prepare to avoid waiting for guard time
  G_io_se_usart.Instance->CR1 &= ~USART_CR1_RE;
  
  // here, ignore the packet length encoding, just pour all the bytes to be sent, no check...
  while(length--) {
    // wait for a usart tx room
    while(!(G_io_se_usart.Instance->ISR & USART_FLAG_TXE));
    G_io_se_usart.Instance->TDR = *buffer++;
  }
  
  // this trick avoid to wait for guard time on last param, 
  // and be sure to receive the first character coming from 
  // the SE, even after 2 stop bits.
  while(!(G_io_se_usart.Instance->ISR & USART_FLAG_TC));
  G_io_se_usart.Instance->CR1 |= USART_CR1_RE;

  // prepare for SE transfer notification
  SE_ISO2C_CLK_MODE_INPUT();
  // ensure data received at slave endpoint
  SE_ISO2C_CLK_WAIT_LOW();
  // prepare for SE transfer notification
  SE_ISO2C_CLK_MODE_ISR();
  // transfer ended
  SE_ISO2C_RST_LOW();
}

// SE_iso2c_recv performed under interruption and pushing the event in the seproxyhal directly

void SE_iso2c_power(unsigned char powered) {
  if (powered) {
    // gpio pins setup
    GPIO_InitTypeDef GPIO_InitStruct;
    // Don't configure CK (PA8) now, to avoid clock output glitch
    /* Configure USART1 TX (PA9) */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure USART1 RESET (PC12) */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // power the USART peripheral
    __USART1_CLK_ENABLE();
    
    // ensure to avoid invalid values into the USART config
    memset(&G_io_se_usart, 0, sizeof(G_io_se_usart));
    
    // dummy usart1 initialization
    G_io_se_usart.Instance = USART1;
    //G_io_se_usart.Init.BaudRate = 115200;
    G_io_se_usart.Init.WordLength = USART_WORDLENGTH_8B;
    G_io_se_usart.Init.StopBits = USART_STOPBITS_1_5;
    G_io_se_usart.Init.Parity = USART_PARITY_EVEN;
    G_io_se_usart.Init.Mode = USART_MODE_TX_RX;
    G_io_se_usart.Init.CLKPolarity = USART_POLARITY_LOW;
    G_io_se_usart.Init.CLKPhase = USART_PHASE_1EDGE;
    //G_io_se_usart.Init.CLKLastBit = USART_LASTBITDISABLE;
    HAL_USART_Init(&G_io_se_usart);
    
    /* Disable the Peripheral */
    __HAL_USART_DISABLE(&G_io_se_usart);

    /* set the USART in SMARTCARD mode */ 
    // over8 to reach super speeds
    G_io_se_usart.Instance->CR1 |= USART_CR1_M0 | USART_CR1_OVER8; 
    G_io_se_usart.Instance->CR2 |= USART_CR2_CLKEN; 
    //G_io_se_usart.Instance->CR2 &= ~ (USART_CR2_TXINV|USART_CR2_RXINV|USART_CR2_SWAP|USART_CR2_DATAINV);
    G_io_se_usart.Instance->CR3 |= /*USART_CR3_NACK |*/ USART_CR3_SCEN | USART_CR3_WUS_0 | USART_CR3_WUS_1; 
    
    // depending on the clock source (usb enabled or not) the gtpr prescaler must be set to avoid overclocking
    G_io_se_usart.Instance->GTPR = 1; // divide clock by 2 with no guard time value // 5 -> ck = 3.2mhz
    //G_io_se_usart.Instance->GTPR = 3; // divide clock by 2 with no guard time value // 5 -> ck = 3.2mhz
    G_io_se_usart.Instance->BRR = 0x20;

    /* Enable the Peripharal */
    __HAL_USART_ENABLE(&G_io_se_usart);

    //SE_update_baudrate(0x10);

    SE_ISO2C_RST_LOW();
    
    SE_ISO2C_CLK_MODE_ISR();
  }
  else {
    // analog all pins
    // gpio pins setup
    GPIO_InitTypeDef GPIO_InitStruct;
    /* Configure USART1 CK (PA8) and USART1 TX (PA9)*/
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure USART1 RESET (PC12) */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // power OFF USART peripheral
    __USART1_CLK_DISABLE();
  }
}
#endif // 0

#endif // HAVE_SE
