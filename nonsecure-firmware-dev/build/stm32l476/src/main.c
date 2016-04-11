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


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_nucleo_bluenrg.h"

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

/************************* Miscellaneous Configuration ************************/

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

#include "osal.h"
#include "sample_service.h"
#include "hci.h"
#include "hal.h"
#include "bluenrg_interface.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h" 

#include <stdio.h>

#include "bagl.h"
#include "seproxyhal.h"

#include "bootloader.h"

// not used for now
#define THROW(x) for(;;);

#define SYSTICK_MS 8

#define UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES 25

#define BACKLIGHT_FULL_LEVEL  70
#define BACKLIGHT_DIM_LEVEL   20

//#define DEMO_LOGO

//#define TEST_KEYBOARD

const uint32_t CLOCK_SECOND = 1000/SYSTICK_MS;

volatile unsigned int frequency_hz;

SPI_HandleTypeDef hspi;
volatile unsigned short G_io_apdu_length;
USART_HandleTypeDef G_io_se_usart;
#ifndef HAVE_BL
unsigned int G_io_se_link_next_mhz;
unsigned int G_io_se_link_next_etu;
#endif // HAVE_BL

volatile unsigned char G_io_se_powered = 0;
volatile unsigned char G_io_apdu_buffer[260];
extern unsigned char G_io_se_atr[40];
extern unsigned short G_io_se_atr_length;

volatile unsigned int G_io_seproxyhal_ticker_interval_ms;
volatile unsigned int G_io_seproxyhal_ticker_current_ms;
volatile unsigned int G_io_seproxyhal_ticker_enabled;

volatile unsigned int G_screen_brightness_test_ms;
volatile unsigned int G_battery_test_ms;
volatile unsigned int G_watchdog_ms;
volatile unsigned int G_poweroff_ms;
volatile unsigned int G_backlight_autooff_ms;
volatile unsigned int G_backlight_autodim_ms;
#define BATTERY_CHECK_INTERVAL_MS (10*1000)

volatile struct {
  uint32_t duration_ms;
  uint32_t pressed;
  #define MODE_SE_RECOVERY    1
  #define MODE_MCU_BOOTLOADER 2
  #define MODE_POWER_OFF      4
  #define MODE_POWER_ON       8
  #define MODE_BOOT          16
  uint32_t displayed_mode;

  uint32_t boot_moment;
} G_io_button;

#define BUTTON_PRESS_DURATION_BOOT_POWER_OFF      10000 // long enough to avoid people being troubled when trying to go bootloader
#define BUTTON_PRESS_DURATION_BOOT_CTRL_BOOTLOADER 5000
#define BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY     2000
#define BUTTON_PRESS_DURATION_BOOT_POWER_ON         100
#define BUTTON_PRESS_DURATION_POWER_OFF            1000

volatile struct touch_state_s G_io_touch;
volatile struct ble_state_s G_io_ble;
volatile struct usb_state_s G_io_usb;
volatile int G_io_seproxyhal_state;
volatile unsigned char G_io_ble_apdu_protocol_enabled;
volatile unsigned char G_io_seproxyhal_ble_handles[IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT];
volatile unsigned char G_io_seproxyhal_ble_last_read_request_handle;
volatile unsigned int G_io_seproxyhal_events;
volatile unsigned char G_io_seproxyhal_buffer[IO_SEPROXYHAL_BUFFER_SIZE_B];

extern unsigned char _signed;
extern unsigned char _esigned;
extern unsigned char _esignature;
volatile struct {
  unsigned int size;
  unsigned int offset;
} G_io_unsec_chunk;

// dummy in case no screen HW declared
__weak void screen_init(unsigned char reinit) {}
__weak void screen_clear(void) {}
__weak void screen_poweroff(void) {}
__weak void screen_update(void) {}
__weak void screen_printf(const char* format,...) {}
__weak void screen_xy(unsigned short x, unsigned short y, unsigned short rotation) {}

const char ESC_DEVICE_NAME[] = "$DEVICENAME";
const char DEVICE_NAME_NA[] = "Device Name not available";

unsigned int backlight_level;
unsigned int backlight_is_enabled(void) {
  #ifdef BACKLIGHT_AUTOOFF_MS
  return G_backlight_autooff_ms < BACKLIGHT_AUTOOFF_MS;
  #else // if no auto off, the backlight is always on
  return 1;
  #endif //
}

void backlight_dim(void) {
  backlight_level = BACKLIGHT_DIM_LEVEL;
  screen_brightness(BACKLIGHT_DIM_LEVEL);
}

void backlight_enable(unsigned int enable) {
  if (enable) {

#ifdef BACKLIGHT_AUTOOFF_MS
    // don't re-enable, to avoid too much delay (due to printf) and no glitch
    if (backlight_is_enabled() && G_backlight_autodim_ms < BACKLIGHT_AUTODIM_MS) {
      G_poweroff_ms = 0;
      G_backlight_autooff_ms = 0;
      G_backlight_autodim_ms = 0;
      if (backlight_level != BACKLIGHT_FULL_LEVEL) {
        goto full;
      }
      return;
    }
#endif // BACKLIGHT_AUTOOFF_MS

#ifdef BACKLIGHT_AUTODIM_MS
    if (G_backlight_autodim_ms < BACKLIGHT_AUTODIM_MS) {
      G_poweroff_ms = 0;
      G_backlight_autooff_ms = 0;
      G_backlight_autodim_ms = 0;
      if (backlight_level != BACKLIGHT_FULL_LEVEL) {
        goto full;
      }
      return; 
    }
#endif // BACKLIGHT_AUTODIM_MS

    // DON'T CARE DIMING, REENABLE FULL

    // reenable backlight
    G_poweroff_ms = 0;
    G_backlight_autooff_ms = 0;
    G_backlight_autodim_ms = 0;
  full:
    backlight_level = BACKLIGHT_FULL_LEVEL;
    screen_brightness(BACKLIGHT_FULL_LEVEL);
  }
  else {
    screen_brightness(0);   
  }
}

void pre_harakiri(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0); // in case coming from an output state // avoid glitch
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

  // disable aux power (screen etc)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
}


void harakiri(void) {

  __asm volatile("cpsid i");
  
  pre_harakiri();

  /*
  // force button it to 0, to avoid retrig
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // OK GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);

  // 12 ms @ 64mhz
  {volatile unsigned int i = 0x40000; while (i--);}
  */

  // harakiri
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);

  // + reset in case protoboard
  for (;;) {
    NVIC_SystemReset();
  }
}


/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

char BLE_protocol_send(unsigned char* response_apdu, unsigned short response_apdu_length);


/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */
#ifdef UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES
unsigned char UI_led_remaining_ticks;
#endif // UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES

void UI_led_power(unsigned char powered) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = 0; // GPIO
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, powered&0x1);

  // setup led timeout
  if (powered) {
    UI_led_remaining_ticks = UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES;
  }
}

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

void UI_power(unsigned char powered) {
  if (powered) {
    // initialize screen printing
    screen_init(0);
    
#ifdef HAVE_LPTIM
    // Select Peripheral clock for Low Power Timer 1
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    RCC_PeriphCLKInitStruct.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;  
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  
    /* ## - 1 - Enable LPTIM clock ############################################ */
    __LPTIM1_CLK_ENABLE();
    
#if 0
    /* ## - 2 - Force & Release the LPTIM Periheral Clock Reset ############### */  
    /* Force the LPTIM Periheral Clock Reset */ 
    __LPTIM1_FORCE_RESET();  
    /* Release the LPTIM Periheral Clock Reset */  
    __LPTIM1_RELEASE_RESET();
#endif // 0
    
    // internal clock, prescaler 128
    LPTIM1->CFGR = LPTIM_CFGR_PRESC;
    
    // firstly enable the timer to allow for register modification
    LPTIM1->CR   = LPTIM_CR_ENABLE;
    
    // TSC_ACQUISITION_LATENCY_MS/1000 = 1/(FREQUENCY_HZ/128/ARR)
    // ARR = (TSC_ACQUISITION_LATENCY_MS/1000)*(FREQUENCY_HZ/128)
#if ((frequency_hz/128)*TSC_ACQUISITION_LATENCY_MS)/1000 <= 0
#error invalid ticking interval
#endif 
    LPTIM1->ARR  = ((frequency_hz/128)*TSC_ACQUISITION_LATENCY_MS)/1000;
    // enable compare match on reload value
    LPTIM1->IER  = LPTIM_IER_ARRMIE;
    // clear flag in case already present
    LPTIM1->ICR  = LPTIM_ICR_ARRMCF;
    // start the timer in free run
    LPTIM1->CR  |= LPTIM_CR_CNTSTRT;
    
    /* Enable and set LPTIM Interrupt to the highest priority */
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn); 
#endif // HAVE_LPTIM
  }
  else {
    screen_poweroff();
    
#ifdef HAVE_LPTIM
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn); 
    __LPTIM1_CLK_DISABLE();
#endif //HAVE_LPTIM
  }
}

// 32 bytes + validation/cancel
#define GETC_BUFFER_SIZE_B 33
volatile struct io_keyboard_s {
  unsigned char action;
  unsigned char getc_buffer[GETC_BUFFER_SIZE_B];
  unsigned int getc_idx;
} G_io_keyboard;

/*
extern void UI_keyboard_validated(void);
extern void UI_keyboard_cancelled(void);
void UI_keyboard_press(unsigned char key) {
  // avoid buffer overflow
  if (G_io_keyboard.getc_idx>=GETC_BUFFER_SIZE_B) {
    G_io_keyboard.action = 1;
    return;
  }
  
  // append the key to the getc buffer
  G_io_keyboard.getc_buffer[G_io_keyboard.getc_idx++] = key;
        
  // DESIGN NOTE: it could have been done in a user called function, but this is already a user called function for the TSC module.
  if (key == KEY_MAPPING_OK) {
    G_io_keyboard.action = 1;
  }
  else if (key == KEY_MAPPING_CANCEL) {
    G_io_keyboard.action = 1;
  } 
}
*/

void KBD_power(unsigned char powered) {
  // reset keyboard state
  G_io_keyboard.action = 0;
  G_io_keyboard.getc_idx = 0;

  //TODO power on the touch screen
}

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

void SYSTICK_power(unsigned char powered) {
  if (powered) {
    /*Configure the SysTick to have interrupt in 1ms time basis*/
    // SYSTICK_MS/1000 = 1/(frequency_hz/systickreload)
    // systickreload = (SYSTICK_MS/1000)*(frequency_hz)
    /*
#if ((frequency_hz/1000)*SYSTICK_MS) <= 0 || ((frequency_hz/1000)*SYSTICK_MS) > (1<<24)
#error invalid ticking interval
#endif 
    */
/*
    volatile unsigned int v = frequency_hz;
    v /= 1000;
    v *= SYSTICK_MS;
    HAL_SYSTICK_Config(v);
    */
    HAL_SYSTICK_Config(((frequency_hz/1000UL)*SYSTICK_MS));
  }  
  else {
    SysTick->CTRL = 0;
  }
}

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

#ifndef HAVE_BL
/**
  * @brief  PendSV_Handler This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
  // process bluetooth IO state machin more prioritized than nomal process (main)
  HCI_Process();
}
#endif // HAVE_BL

/**
* @brief This function handles USB OTG FS global interrupt.
*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief  SysTick_Handler This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern __IO uint32_t uwTick;
int brightness;
void SysTick_Handler(void)
{
  // increment the systick of the number of milliseconds elapsed since previous tick
  // NOTE: HAX of Hal_IncTick, to support all HAl_Delay callers (shall be avoided in the end)
  // TODO: remove me to detect HAL_Delay loops
  uwTick+=SYSTICK_MS;
  
#if 0
#ifdef POWEROFF_TIMEOUT
  if (poweroff_timeout_ms <= SYSTICK_MS) {
    // TODO poweroff
  }
  poweroff_timeout_ms -= SYSTICK_MS;
#endif // POWEROFF_TIMEOUT
  
  if (G_io_ble.connection_timeout_enabled) {
    if (G_io_ble.connection_timeout_ms <= SYSTICK_MS) {
      G_io_ble.connection_reset = 1;
      G_io_ble.connection_timeout_ms = 0;
    }
    else {
      G_io_ble.connection_timeout_ms -= SYSTICK_MS;
    }
  }
#endif 
  
#ifdef UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES
  // unpower leds if countdown reached
  if (UI_led_remaining_ticks > 0) {
    UI_led_remaining_ticks--;
    if (UI_led_remaining_ticks == 0) {
      UI_led_power(0);
    }
  }
#endif // UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES

  // update the ticker event
  if (G_io_seproxyhal_ticker_enabled) {
    G_io_seproxyhal_ticker_current_ms += SYSTICK_MS;
    if (G_io_seproxyhal_ticker_current_ms >= G_io_seproxyhal_ticker_interval_ms) {
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_TICKER;
      G_io_seproxyhal_ticker_current_ms = 0;
    }
  }

  if (G_io_button.pressed) {
    G_io_button.duration_ms += SYSTICK_MS;

    if (G_io_button.displayed_mode & MODE_BOOT) {
      if (G_io_button.duration_ms > BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY && ! (G_io_button.displayed_mode & MODE_SE_RECOVERY)) {
        G_io_button.displayed_mode |= MODE_SE_RECOVERY;
        screen_printf("Secure bootloader\n");
      }

      if (G_io_button.duration_ms > BUTTON_PRESS_DURATION_BOOT_CTRL_BOOTLOADER && ! (G_io_button.displayed_mode & MODE_MCU_BOOTLOADER)) {
        G_io_button.displayed_mode |= MODE_MCU_BOOTLOADER;
        screen_printf("Bootloader\n");
      }

      if (G_io_button.duration_ms > BUTTON_PRESS_DURATION_BOOT_POWER_OFF && ! (G_io_button.displayed_mode & MODE_POWER_OFF)) {
        G_io_button.displayed_mode |= MODE_POWER_OFF;
        PRINTF("Mode power off\n");

  #ifndef DEBUG_BUTTON_ALWAYS_PUSHED
        // disable aux power (screen etc) for haptic return
        pre_harakiri();

        // TODO engage watchdog
  #endif // DEBUG_BUTTON_ALWAYS_PUSHED
      }
    }
    else {
      if (G_io_button.duration_ms > BUTTON_PRESS_DURATION_POWER_OFF && ! (G_io_button.displayed_mode & MODE_BOOT) && ! (G_io_button.displayed_mode & MODE_POWER_OFF)) {
        G_io_button.displayed_mode |= MODE_POWER_OFF;
        PRINTF("Mode power off\n");

  #ifndef DEBUG_BUTTON_ALWAYS_PUSHED
        // disable aux power (screen etc) for haptic return
        pre_harakiri();

        // TODO engage watchdog
  #endif // DEBUG_BUTTON_ALWAYS_PUSHED
      }
    }

  }

#if 0
  // screen brightness test
  if (uwTick > G_screen_brightness_test_ms) {
    G_screen_brightness_test_ms = uwTick + 512;

    brightness += 5;
    brightness %= 100;

    // 40 -> 90
    screen_printf("%d\n", brightness);
    screen_brightness(brightness);
  }
#endif 

#if 1
  // battery level check (only when not button pressed, to avoid problem during boot)
  if (uwTick > G_battery_test_ms 
      && G_io_usb.bootloader // only display battery over bootloader
#ifdef HAVE_DRAW
    && G_battery_test_ms != -1 
#endif // HAVE_DRAW
    && !G_io_button.pressed) {
    unsigned char battery_text [64];
    G_battery_test_ms = uwTick + BATTERY_CHECK_INTERVAL_MS;

    // perform measurement
    strcpy(battery_text, "Battery level: ");
    unsigned int b = battery_get_level_mv();
    battery_text[15] = (b/1000)%10 + '0';
    battery_text[15+1] = '.';
    battery_text[15+2] = (b/100)%10 + '0';
    battery_text[15+3] = (b/10)%10 + '0';
    battery_text[15+4] = 'V';
    battery_text[15+5] = '\0';
    strcat(battery_text, ", Version: " VERSION);

    bagl_component_t batt_comp = 
    {BAGL_LABEL                          , 0x00,   0, 438, 320,  14, 0, 0, BAGL_FILL, 0x999999, 0xF9F9F9, BAGL_FONT_OPEN_SANS_LIGHT_13px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0 };

    bagl_draw_with_context(&batt_comp, battery_text, strlen(battery_text), BAGL_ENCODING_LATIN1);

    //PRINTF("%d.%03d V %s\n", b/1000, b%1000, (b < 3700 ? "VERY LOW BATT !!":(b < 3900? "low battery, charge me !":"")) );
  }
#endif 

#ifdef WATCHDOG_POWEROFF_MS
  G_poweroff_ms += SYSTICK_MS;
  if (G_poweroff_ms > WATCHDOG_POWEROFF_MS) {
    harakiri();
  }
#endif 

#ifdef BACKLIGHT_AUTODIM_MS
  // auto dim off the backlight when no activity after some times
  if (G_backlight_autodim_ms < BACKLIGHT_AUTODIM_MS) {
    G_backlight_autodim_ms += SYSTICK_MS;

    if (G_backlight_autodim_ms >= BACKLIGHT_AUTODIM_MS) {
      backlight_dim();
    }
  }
#endif // BACKLIGHT_AUTODIM_MS

#ifdef BACKLIGHT_AUTOOFF_MS
  // auto dim off the backlight when no activity after some times
  if (G_backlight_autooff_ms < BACKLIGHT_AUTOOFF_MS) {
    G_backlight_autooff_ms += SYSTICK_MS;

    if (G_backlight_autooff_ms >= BACKLIGHT_AUTOOFF_MS) {
      backlight_enable(0);
    }
  }
#endif // BACKLIGHT_AUTOOFF_MS
}

/**
 * Handle the button interrupt
 */
void EXTI2_IRQHandler(void) {
  // detect BLE request to read
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_2)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) {
      G_io_button.pressed = 1;
      G_io_button.duration_ms = 0;
      G_io_button.displayed_mode = 0;
      G_io_button.boot_moment = 0; // not needed

      // ensure backlight is enabled
      backlight_enable(1);
    }
    else if (G_io_button.pressed) {
#ifndef DEBUG_BUTTON_ALWAYS_PUSHED
      if (G_io_button.displayed_mode & MODE_BOOT) {
        // power off safe guard
        if (G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_POWER_OFF) {
          harakiri();
        }
      }
      else {
        // power off safe guard
        if (G_io_button.duration_ms >= BUTTON_PRESS_DURATION_POWER_OFF) {
          harakiri();
        }
      }
#endif // DEBUG_BUTTON_ALWAYS_PUSHED

      G_io_button.pressed = 0;
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BUTTON;
    }
  }
}

#ifndef HAVE_BL
/**
  *
  *
  */
void EXTI9_5_IRQHandler(void) {
  // detect BLE request to read
  if (BNRG_SPI_EXTI_PORT->IDR & BNRG_SPI_EXTI_PIN) {
    //Hal_IRQ_Serial();
    HCI_Isr();
  }
  // done after to avoid glitch interpretation during read command start
  __HAL_GPIO_EXTI_CLEAR_IT(BNRG_SPI_EXTI_PIN);
  #if 0
  extern void SE_iso2c_isr_clk_rise(void);
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_8)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8); 
    //Hal_IRQ_Serial();
    SE_iso2c_isr_clk_rise();
  }
  #endif // 0
}
#endif // HAVE_BL

#include "bagl.h"


/* Private function prototypes -----------------------------------------------*/
void User_Process(void);

void clock_low(void) {
  //RCC->ICSCR = ((RCC->ICSCR) & 0xFFFF1FFF) | 0x00000000;
}

void clock_high(void) {

  // MSI
  //RCC->ICSCR = ((RCC->ICSCR) & 0xFFFF1FFF) | (MSIRANGE<<13);
  // HSI
#if 0
  RCC_OscInitTypeDef RCC_OscInitStruct;
  //RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  //RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  //RCC_OscInitStruct.HSICalibrationValue = 0;
  //RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  //HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC->CR |= RCC_CR_HSION|RCC_CR_HSIDIVEN;
  while(!(RCC->CR & RCC_CR_HSIRDY));
  __HAL_RCC_PLL_CONFIG(RCC_OscInitStruct.PLL.PLLSource,
                       RCC_OscInitStruct.PLL.PLLMUL,
                       RCC_OscInitStruct.PLL.PLLDIV);
  /* Enable the main PLL. */
  __HAL_RCC_PLL_ENABLE();
  while(!(RCC->CR & RCC_CR_PLLRDY));
   RCC->CR &= ~RCC_CR_MSION;
   #endif
}

#include "stm32l4xx_hal_adc.h"
unsigned int battery_get_level_mv(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  volatile unsigned int mv;
  volatile unsigned int vdd;

  mv = 0;

  // power up the measurment bridge
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // drive current low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);

#define BAT_ADC_DELAY_MS 10
  volatile unsigned int i = ((frequency_hz/1000)*BAT_ADC_DELAY_MS)/1000/3;
  while(i--);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // compute Vdd value against VRefInt (optional, could use the LDO output 3.0v)
  vdd = 3100;

  // ADC aquisition
  ADC_HandleTypeDef             AdcHandle;
  ADC_ChannelConfTypeDef        sConfig;
  memset(&AdcHandle, 0, sizeof(AdcHandle));
  memset(&sConfig, 0, sizeof(sConfig));
  
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
  __HAL_RCC_ADC_CLK_ENABLE();
  AdcHandle.Instance                   = ADC1;

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;          /* Asynchronous clock mode, input ADC clock not divided */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* DMA one-shot mode selected (not applied to this example) */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle.Init.OversamplingMode      = ENABLE;                        /* Oversampling enabled */
  AdcHandle.Init.Oversampling.Ratio                 = ADC_OVERSAMPLING_RATIO_256;    /* Oversampling ratio */
  AdcHandle.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_4;         /* Right shift of the oversampled summation */
  AdcHandle.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;         /* Specifies whether or not a trigger is needed for each sample */
  AdcHandle.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE; /* Specifies whether or not the oversampling buffer is maintained during injection sequence */    
 
  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    return 0xFFFFFFFE;
  }
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
  {
    return 0xFFFFFFFD;
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  /* Select Channel 0 to be converted */
  sConfig.Channel      = ADC_CHANNEL_14 /*PC5*/;
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    return 0xFFFFFFFC;
  }
  
 /*##- 4- Start the conversion process #######################################*/  
  if(HAL_ADC_Start(&AdcHandle) != HAL_OK) {
    return 0xFFFFFFFB;
  }

  // wait for conversion to end
  HAL_ADC_PollForConversion(&AdcHandle, 10000);
  
  /* Check if the continous conversion of regular channel is finished */
  if(HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC)
  {
    mv = HAL_ADC_GetValue(&AdcHandle);
    //screen_printf(" %d ", mv);
    // mv is computed on a 1/2 voltage divider
    // range of the adc if [0 ; (65536-16)]
    mv = 2*mv *vdd/((1<<16)-16);

    /*
    // VREFINT is directly converted to obtain the effective vcc value
    if (io->adc_channel == 17) {
      #define VREFINT_CAL (*(uint16_t*)0x1FF80078)
      uint32_t val = 3000UL * (VREFINT_CAL<<4) / (value);
      //uint32_t val = (VREFINT_CAL) * (value) / 1200;
      //uint32_t val = 3000000UL / 1224UL * (value) / (VREFINT_CAL); // 1.224v for the vref gap
      value = val;
    }
    */
  }

  // stop the ADC
  HAL_ADC_Stop(&AdcHandle);

  HAL_ADC_DeInit(&AdcHandle);

  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  __HAL_RCC_ADC_CLK_DISABLE();
  // low power (high impedance)
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  return mv;
}


/**
 * @brief  Main function to show how to use the BlueNRG Bluetooth Low Energy 
 *          shield to exchange data between two Nucleo baords with their
 *          respective BlueNRG shields.
 *          One board will act as Server-Peripheral and the other as 
 *          Client-Central.
 *          After connection has been established, by pressing the USER button
 *          on one board, the LD2 LED on the other one gets toggled and
 *          viceversa.
 *          The communication is done using a vendor specific profile.
 * @param  None
 * @retval None
 */

USBD_HandleTypeDef USBD_Device;
PCD_HandleTypeDef hpcd_USB_OTG_FS;


void clock_config(void) {
  // setup clock for USB peripheral
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /*
  memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
  memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));
  memset(&PeriphClkInit, 0, sizeof(PeriphClkInit));
  */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1; // 1 to 8
  RCC_OscInitStruct.PLL.PLLN = 24; // 8 to 86 // equals MBit of SPI1 for screen
  //RCC_OscInitStruct.PLL.PLLN = 32; // 8 to 86 // equals MBit of SPI1 for screen
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7; // SAI1/SAI2
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // 48M1CLK
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4; // SYSCLK
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  // compute systick clock at once
  frequency_hz = 8000000*RCC_OscInitStruct.PLL.PLLN/RCC_OscInitStruct.PLL.PLLR;

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  // configure USART1 (se link) clock source
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}


#ifdef HAVE_BL
#define CLA 0xE0

#define INS_SECUINS 0
#define INS_GET_VERSION 0x01
#define INS_RESET 2

#define INS_VALIDATE_TARGET_ID 4

// entirely replied (length of Lc)
#define INS_ECHO 0xFF

// graphic debug for NBI
#define INS_DRAW_BMP4BPP          0x40
#define INS_DRAW_BMP4BPP_CONTINUE 0x41

// 0xE0 0xFE 0x00 0x00 0x06 <U4BE(@)> <U2BE(len)>
#define INS_DUMP 0xFE
#define INS_MCU_CMD 0xFD

#define SECUREINS_SELECT_SEGMENT 5
#define SECUREINS_LOAD 6
#define SECUREINS_FLUSH 7
#define SECUREINS_CRC 8
// start at given address (app main)
#define SECUREINS_BOOT 9
#define SECUREINS_TESTBOOT 0xA
#define SECUREINS_ERASE_BOOT 0xE

// within the data field
#define APDU_OFF_CLA 0
#define APDU_OFF_INS 1
#define APDU_OFF_LC 4  
#define APDU_OFF_SECINS 5

#define STATE_ID     0xFF00
#define STATE_UNAUTH 0xEE11
#define STATE_AUTH   0xDD22 

#define U2(hi,lo) ((((hi)&0xFF)<<8) | ((lo)&0xFF))
#define U4(hi3, hi2, lo1,lo0) ((((hi3)&0xFF)<<24) | (((hi2)&0xFF)<<16) | (((lo1)&0xFF)<<8) | ((lo0)&0xFF))
#define U2BE(buf, off) ((((buf)[off]&0xFF)<<8) | ((buf)[off+1]&0xFF) )
#define U4BE(buf, off) ((U2BE(buf, off)<<16) | (U2BE(buf, off+2)&0xFFFF))
#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))
#define os_memmove memmove
#define os_memset memset

// --------------------------------------------------------------------------
// -
// --------------------------------------------------------------------------

static unsigned short const cx_ccitt[] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};


// --------------------------------------------------------------------------
// -
// --------------------------------------------------------------------------

unsigned short cx_crc16_update(unsigned short crc, void const *buf, int len) {
#define buffer ((unsigned char const *)buf) 
  int i;
  unsigned char b;

  for (i = 0; i<len; i++) {
    b = buffer[i];
    b = b ^ (crc >> 8);
    crc = cx_ccitt[b] ^ (crc << 8);
  }
  return crc;
}


unsigned short  cx_crc16(void const *buf, int len) {
  return cx_crc16_update(0xFFFF, buf, len);
}

unsigned char nvm_page_D [NVM_PAGE_SIZE_B];

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}

// todo later
const unsigned char * nvm_write_page_address;
void nvm_write_page_flush(void) {

}

void nvm_write_page(const unsigned char* page) {
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
  /* Fill EraseInit structure*/
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(page);
  EraseInitStruct.Page        = GetPage(page);
  EraseInitStruct.NbPages     = 1;

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  unsigned int PAGEError;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
    // TODO : Halt, catch fire, explode 
  }

  const unsigned long long int* page_dword = page;
  const unsigned long long int* buffer_dword = nvm_page_D;
  while ((unsigned int)buffer_dword < ((unsigned int)nvm_page_D)+sizeof(nvm_page_D)) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, page_dword, *buffer_dword) != HAL_OK)
    {
      // TODO : Halt, catch fire, explode 
    }
    buffer_dword++;
    page_dword++;
  }

  HAL_FLASH_Lock();
}

void nvm_write(void * dst_adr, void* src_adr, unsigned short src_len) {
#define DST_ADR ((const unsigned char*) dst_adr)
#define SRC_ADR ((const unsigned char*) src_adr)
  const unsigned char* page;
  unsigned short len;

  if (src_len == 0) {
    return;
  }

  // head, align dst_adr on a page if not
  page = (const unsigned char*)(((unsigned int)DST_ADR) & ~(NVM_PAGE_SIZE_B-1));
  if(page != DST_ADR) {
    unsigned short page_off = (DST_ADR-page);
    memmove(nvm_page_D, page, NVM_PAGE_SIZE_B);
    len = NVM_PAGE_SIZE_B-page_off;
    if (len > src_len) {
      len = src_len;
    }
    memmove(nvm_page_D+page_off, SRC_ADR, len);
    src_adr = SRC_ADR + len;
    nvm_write_page(page);
    src_len -= len;
    page += NVM_PAGE_SIZE_B;
  }

  while(src_len > NVM_PAGE_SIZE_B) {
    memmove(nvm_page_D, SRC_ADR, NVM_PAGE_SIZE_B);
    src_adr = SRC_ADR + NVM_PAGE_SIZE_B;
    nvm_write_page(page);
    src_len -= NVM_PAGE_SIZE_B;
    page += NVM_PAGE_SIZE_B;
  }

  if (src_len) {
    memmove(nvm_page_D, page, NVM_PAGE_SIZE_B);
    memmove(nvm_page_D, SRC_ADR, src_len);
    nvm_write_page(page);
  }
}

const unsigned char C_bootloader_version[] = {
  'B','L','U','0','1','0','0'
};


void bootloader_erase_appconf(void) {
  union {
    bootloader_configuration_t ramconf;
    unsigned char page_remaining[NVM_PAGE_SIZE_B];
  } bootpage;
  memset(&bootpage, 0, sizeof(NVM_PAGE_SIZE_B));

  // erase the whole page
  nvm_write(&N_bootloader_configuration, &bootpage, NVM_PAGE_SIZE_B);
  // ensure flushed to nvram
  nvm_write_page_flush();
}

extern unsigned int _text;
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;

void bootloader_apdu_interp(void) {
  volatile unsigned short rx = 0;
  volatile unsigned short tx = 0;
  volatile unsigned char iv[8];
  volatile unsigned char flags;
  volatile unsigned int load_address;
  volatile unsigned short state = STATE_ID;
  // reset variables to avoid subtil attack after returning from the loaded app
  rx = 0;
  tx = 0;
  state = STATE_ID;
  load_address = 0;
  flags = 0;
  memset(iv, 0, sizeof(iv));

  io_usb_hid_init();

  nvm_write_page_address = NULL;

  // DESIGN NOTE: the bootloader ignores the way APDU are fetched. The only goal is to retrieve APDU.
  // When APDU are to be fetched from multiple IOs, like NFC+USB+BLE, make sure the io_event is called with a 
  // switch event, before the apdu is replied to the bootloader. This avoid APDU injection faults.
        
  for (;;) {

    unsigned short sw;

    // process screen touch
    while ((rx = io_exchange(CHANNEL_APDU|flags, 0)) == 0 ) {

      screen_update_touch_event();

      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BUTTON;

        backlight_enable(1);
      }

      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_RELEASE) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_RELEASE;

        if (backlight_is_enabled()) {
          // reset power off, a new command has been received
          backlight_enable(1);
        }

        display_l4_mode_touch();
      }
    }

    // reset power off, a new command has been received
    backlight_enable(1);

    tx = 0; // ensure no race in catch_other if io_exchange throws an error

    // no apdu received, well, reset the session, and reset the bootloader configuration
    if (rx == 0) {
      sw = 0x6982; goto error;
    }

    if (G_io_apdu_buffer[APDU_OFF_CLA] != CLA) {
      sw = 0x6E00; goto error;
    }

    // unauthenticated instruction
    switch (G_io_apdu_buffer[APDU_OFF_INS]) {
      case INS_VALIDATE_TARGET_ID:
        if (U4BE(G_io_apdu_buffer, 5) == CONFIG_TARGET_ID) {
          state = STATE_UNAUTH;
          sw = 0x9000; goto error;
        }
        else {
          // id not valid
          state = STATE_ID;
          sw = 0x6A84; goto error;
        }
        break;

#ifdef HAVE_DRAW
      case INS_DRAW_BMP4BPP: {
        G_battery_test_ms = -1; // avoid disturbance during graphic tests
        unsigned short x = U2BE(G_io_apdu_buffer, 5);
        unsigned short y = U2BE(G_io_apdu_buffer, 7);
        unsigned short width = U2BE(G_io_apdu_buffer, 9);
        unsigned short height = U2BE(G_io_apdu_buffer, 11);
        unsigned short colors_count = G_io_apdu_buffer[13];
        // colors are LE encoded
        unsigned bit_per_pixel = 4;
        bagl_hal_draw_bitmap_within_rect(x, 
                                         y, 
                                         width, 
                                         height, 
                                         colors_count, 
                                         (unsigned int*)(G_io_apdu_buffer+14), // potentially unaligned // this is debugggg #yolo
                                         bit_per_pixel, 
                                         G_io_apdu_buffer+14+colors_count*4, 
                                         (rx - 5 - 9 - colors_count*4)*8);
        sw = 0x9000; goto error;
        break;
      }

      case INS_DRAW_BMP4BPP_CONTINUE: {
        bagl_hal_draw_bitmap_continue(4, G_io_apdu_buffer+5, (rx-5)*8);
        break;
      }
#endif // HAVE_DRAW

      case INS_RESET:
        flags |= IO_RESET_AFTER_REPLIED;
        sw = 0x9000; goto error;
        break;

      case INS_GET_VERSION:
        tx = sizeof(C_bootloader_version);
        os_memmove(G_io_apdu_buffer, &C_bootloader_version, sizeof(C_bootloader_version));
        sw = 0x9000; goto error;
        break;

      case INS_ECHO:
        tx = rx;
        sw = 0x9000; goto error;
        break;

      /*
      case INS_DUMP:
        #warning DUMP IS ACTIVATED
        if (G_io_apdu_buffer[APDU_OFF_LC] != 4+2) {
          sw = 0x6700; goto error;
        }
        // dump dump
        tx = MIN(256, U2BE(G_io_apdu_buffer, 9));
        os_memmove(G_io_apdu_buffer, U4BE(G_io_apdu_buffer, 5), tx);
        sw = 0x9000; goto error;
        break;
      */

      /*
      case 0x2:
        break;
      case 0x3:
        cx_des_init_key blablabla
        state = STATE_AUTH
        break;
      */

      // proceed to the secure instruction interpreter
      case INS_SECUINS:
        // retrieve data (all raw loader commands are case 2)
        rx = G_io_apdu_buffer[APDU_OFF_LC];

        // loader ID not yet presented
        if (state == STATE_ID) {
          sw = 0x6985 ; goto error;
        }

// SECURITY DEACTIVATED UNTIL CRYPTO IS READY
#if 0
        // kthx you can reset the plug, manually :)
        if (state != STATE_AUTH) {
          THROW(0x6982);
        }

        // decrypt and unpad data
        // CLA INS P1 P2 LC CIHERDATA
        rx = bootloader_unwrap(&(iv[0]), G_io_apdu_buffer+5, rx, &deskey);

        // check crc in clear text (crc are last 2 bytes, BE)
        // CLA INS P1 P2 LC { SECINS DATA CRC }
        rx-=2; // final crc not accounted in data length
        if (cx_crc16(G_io_apdu_buffer+5, rx) != U2BE(G_io_apdu_buffer+5, rx)) {
          THROW(0x6982);
        }
#endif // 0
        
        switch(G_io_apdu_buffer[APDU_OFF_SECINS]) {
          // all data are ciphered with the issuer key, no data is trusted from the
          // SCP as the customer key could be compromised
          case SECUREINS_SELECT_SEGMENT: {
            if (rx != 1+4) {
              sw = 0x6700; goto error;
            }

            load_address = U4BE(G_io_apdu_buffer, 6);
            // nothing to output
            break; 
          }
          case SECUREINS_LOAD: {
            // at least offset (2 bytes) + 1 byte to patch + 1 byte instruction
            if (rx < 1+2+1) {
              sw = 0x6700; goto error;
            }


            unsigned int chunk_address = ((unsigned int)load_address + (unsigned int)U2BE(G_io_apdu_buffer, 6));

            // feil the session, won't accept further command
            if (chunk_address <= &_etext) {
              sw = 0x6984; goto error;
            }

            // avoid loading when a code is ready for booting
            if (cx_crc16(&N_bootloader_configuration.appmain, sizeof(appmain_t)) == N_bootloader_configuration.crc) {
              bootloader_erase_appconf();
            }

            // protect against bootloader self modification
            // align to the next nvm page (could leave a blank page in between if already aligned)
            if (chunk_address >= (unsigned int)&_text && chunk_address < ((unsigned int)&_etext + (unsigned int)&_edata - (unsigned int)&_data)&(~(NVM_PAGE_SIZE_B-1)) + NVM_PAGE_SIZE_B ) {
              sw = 0x6985; goto error;
            }

            nvm_write((unsigned char const *)chunk_address, G_io_apdu_buffer+8, rx-3);
            break; 
          }
          case SECUREINS_FLUSH: {
            // not needed, will be done by the boot command // nvm_write_page_flush();
            break;
          }
          case SECUREINS_CRC: {
            if (rx != 1+2+4+2) {
              sw = 0x6700; goto error;
            }
            // check crc, OR security error
            if (cx_crc16((unsigned char const*)((unsigned int)load_address + (unsigned int)U2BE(G_io_apdu_buffer, 6)), U4BE(G_io_apdu_buffer, 8)) != U2BE(G_io_apdu_buffer, 12)) {
              sw = 0x6982; goto error;
            }
            break;
          }

          case SECUREINS_BOOT: {
            if (rx != 1+4) {
              sw = 0x6700; goto error;
            }

            union {
             bootloader_configuration_t ramconf;
             unsigned char page_remaining[NVM_PAGE_SIZE_B];
            } bootpage;
            memset(&bootpage, 0, NVM_PAGE_SIZE_B);

            // write the address of the main
            bootpage.ramconf.appmain = (appmain_t)U4BE(G_io_apdu_buffer, 6);
            // to be done in called code // ramconf.vtor = (appmain_t)U4BE(G_io_apdu_buffer, 6+4);

            // update the crc value
            bootpage.ramconf.crc = cx_crc16(&bootpage.ramconf.appmain, sizeof(appmain_t));
            // ensure page always filled with zeros, as per code signature generation (even if tearing during page)
            nvm_write(&N_bootloader_configuration, &bootpage, NVM_PAGE_SIZE_B);

            // ensure flushing the boot configuration into nvram
            nvm_write_page_flush();

            // from now on, the application can boot, boot now
            flags |= IO_RESET_AFTER_REPLIED;

            ///BEGIN WIPE STATE
            // invalidate the IV, make sure nothing can be validated after that
            os_memset(&(iv[0]), 0xFF, sizeof(iv));
            state = STATE_ID;
            ///END WIPE STATE
            break;
          }

          case SECUREINS_TESTBOOT: {
            if (rx != 1+4) {
              sw = 0x6700; goto error;
            }

            // prepare the VTOR
            // to be done in called code // SCB->VTOR = (appmain_t)U4BE(G_io_apdu_buffer, 6+4);
            // start the code
            ((appmain_t)(U4BE(G_io_apdu_buffer, 6)))(0);
          }

          case SECUREINS_ERASE_BOOT: {
            if (cx_crc16(&N_bootloader_configuration.appmain, sizeof(appmain_t)) == N_bootloader_configuration.crc) {
              bootloader_erase_appconf();
            }
          }

          default:
            sw = 0x6D00; goto error;
            break;
        }
        // ok no problem
        sw = 0x9000; goto error;
        break;

      // ensure INS is empty otherwise (use secure instruction)
      default:
        sw = 0x6D00; goto error;
    }

  error:
    // Unexpected exception => security erase
    G_io_apdu_buffer[tx] = sw>>8;
    G_io_apdu_buffer[tx+1] = sw;
    tx += 2;
    // error detected !!
    if (sw != 0x9000) {
      ///BEGIN WIPE STATE
      // invalidate the IV, make sure nothing can be validated after that
      os_memset(&(iv[0]), 0xFF, sizeof(iv));
      state = STATE_ID;
      /// END WIPE STATE
    }

    // reply response apdu
    io_exchange(CHANNEL_APDU|flags|IO_RETURN_AFTER_TX, tx);
  }
}

void bootloader_delegate_boot(uint32_t button_press_duration) {

  // if boot address is valid, then boot it
  if (cx_crc16(&N_bootloader_configuration.appmain, sizeof(appmain_t)) == N_bootloader_configuration.crc) {

    // disable interruption before changing VTOR and delegate to application code
    __asm("cpsid i");

    // delegate interrupts (per arch style, to avoid strange compat depending on the chip ABI)
    // to be done in called code // SCB->VTOR = N_bootloader_configuration.vtor;

    // jump into the application code
    N_bootloader_configuration.appmain(button_press_duration);

    __asm("cpsid i");

    // jump failed (or loaded code returned) go to bootloader
    nvm_write_page_address = NULL; // no write yet
    bootloader_erase_appconf();

    // ensure to reset the display of the bootloader prompt
    NVIC_SystemReset();
  }
}
#endif // HAVE_BL

void SystemClock_Config(void) {
  // TODO re apply the best clock scheme according to the current power mode
}

typedef struct bagl_element_e {
  bagl_component_t component;
  const char* text;
} bagl_element_t;


const bagl_element_t screen_boot_l4[] = {
  // type                                 id    x    y    w    h    s  r  fill       fg        bg        font                                                                                icon   text, out, over, touch
  // erase bootloader logo (center of screen)
  {{BAGL_ICON                           , 0x00, 109, 188, 101, 104, 0, 0, BAGL_FILL, 0,         0,          0                                                                                 , BAGL_GLYPH_LOGO_LEDGER_100  }, NULL},
  {{BAGL_LABEL                          , 0x00,   0, 438, 320,  14, 0, 0, BAGL_FILL, 0x999999, 0xF9F9F9, BAGL_FONT_OPEN_SANS_LIGHT_13px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0 }, "Version: " VERSION },
};

void display_l4_boot(void) {
  unsigned int i;

  for (i=0 ; i < sizeof(screen_boot_l4) / sizeof(bagl_element_t); i++) {
    unsigned char* text = screen_boot_l4[i].text;
    bagl_draw_with_context(&screen_boot_l4[i].component, text, text?strlen(text):0, text?BAGL_ENCODING_LATIN1:0);
  }
}

const bagl_element_t screen_mode_l4[] = {
  // type                                 id    x    y    w    h    s  r  fill       fg        bg        font                                                                                icon   text, out, over, touch
  // erase bootloader logo (center of screen)
  {{BAGL_RECTANGLE                      , 0x00, 109, 188, 101, 104, 0, 0, BAGL_FILL, 0xF9F9F9, 0xF9F9F9, 0                                                                                 , 0   }, NULL},

  // erase current mode selection (bottom of screen)
  {{BAGL_RECTANGLE                      , 0x00,   0, 455, 320,  25, 0, 0, BAGL_FILL, 0xF9F9F9, 0xF9F9F9, 0                                                                                 , 0   }, NULL },


  // bootloader screen

  // title bar
  {{BAGL_RECTANGLE                      , 0x00,   0,   0, 320,  60, 0, 0, BAGL_FILL, 0x1d2028, 0x000000, 0                                                                                 , 0   }, NULL },
  {{BAGL_LABEL                          , 0x00,  19,  21, 260,  20, 0, 0, BAGL_FILL, 0xFFFFFF, 0x1d2028, BAGL_FONT_OPEN_SANS_LIGHT_16px|BAGL_FONT_ALIGNMENT_MIDDLE                         , 0   }, 1 },

  // logo
  {{BAGL_ICON                           , 0x00, 135, 163,  50,  50, 0, 0, BAGL_FILL, 0xCCCCCC, 0xF9F9F9, 0                                                                                 , BAGL_GLYPH_ICON_GEARS_50 } , NULL, },
  //{{BAGL_RECTANGLE | FLAG_COLORARRAY    , 0x00, 135, 163,  50,  50, 0, 0, BAGL_FILL,        4,        0, 0                                                                                     }, {0xCCCCCC, }, C_bitmaps[ICON_GEARS], NULL, NULL, NULL, NULL},


  {{BAGL_LABEL                          , 0x00,   0, 231, 320,  14, 0, 0, BAGL_FILL, 0x000000, 0xF9F9F9, BAGL_FONT_OPEN_SANS_BOLD_13px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0  }, 2 },
  {{BAGL_LABEL                          , 0x00,   0, 258, 320,  14, 0, 0, BAGL_FILL, 0x000000, 0xF9F9F9, BAGL_FONT_OPEN_SANS_LIGHT_13px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0 }, "No loaded firmware." },

  // for buttons, the background is the label color
//{{BAGL_RECTANGLE                      , 0x00,  90, 300, 140,  40, 0, 6, 0        , 0x41ccb4, 0xF9F9F9, 0                                                                                 , 0   }, NULL },
//{{BAGL_LABEL                          , 0x00,  90, 300, 140,  40, 0, 0, BAGL_FILL, 0xFFFFFF, 0x41ccb4, BAGL_FONT_OPEN_SANS_LIGHT_14px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0 }, "POWER OFF" },

  {{BAGL_LABEL                          , 0x00,   0, 438, 320,  14, 0, 0, BAGL_FILL, 0x999999, 0xF9F9F9, BAGL_FONT_OPEN_SANS_LIGHT_13px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0 }, "Version: " VERSION},
};


unsigned char screen_mode_l4_string_buffer[32];

void display_l4_mode(unsigned char* title, unsigned char* msg) {
  unsigned int i;

  for (i=0 ; i < sizeof(screen_mode_l4) / sizeof(bagl_element_t); i++) {
    if (screen_mode_l4[i].text != NULL) {
      unsigned char* text;
      switch((unsigned int)screen_mode_l4[i].text) {
        case 1:
          // argument 1
          strcpy(screen_mode_l4_string_buffer, title);
          text = screen_mode_l4_string_buffer;
          break;
        case 2:
          // argument 2
          strcpy(screen_mode_l4_string_buffer, msg);
          text = screen_mode_l4_string_buffer;
          break;
        default:
          text = screen_mode_l4[i].text;
          break;
      }
      bagl_draw_with_context(&screen_mode_l4[i].component, text, strlen(text), BAGL_ENCODING_LATIN1);
    }
    else {
      bagl_draw(&screen_mode_l4[i].component);
    }
  }
}

void display_l4_mode_touch(void) {
  // button has been removed
  /*
  if (G_io_touch.ts_last_x >= 80 
    && G_io_touch.ts_last_x <= 80+160
    && G_io_touch.ts_last_y >= 290
    && G_io_touch.ts_last_y <= 290+60) {
    harakiri();
  }
  */
}


extern unsigned int g_pfnVectors;

void main(unsigned int button_press_duration)
{ 
  GPIO_InitTypeDef GPIO_InitStruct;


#ifdef HAVE_BL
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set MSION bit */
  RCC->CR |= RCC_CR_MSION;

  frequency_hz = 4000000;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON , HSION, and PLLON bits */
  RCC->CR &= (uint32_t)0xEAF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x00000800;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIER = 0x00000000;

  /* Configure the Vector Table location add offset address ------------------*/
  SCB->VTOR = &g_pfnVectors; /* Vector Table Relocation in Internal FLASH */
  

  G_io_button.displayed_mode = 0;
  G_io_button.pressed = 0;
  G_io_button.duration_ms = 0; 


  __asm("cpsie i");

  // loop forever (in the final version)
  // TODO wait until power button to wake up all subsys (only activate sys wakeup 2 and power off everything then deep sleep.
  
  
  /* Configure the system clock */
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();
  __SYSCFG_CLK_ENABLE(); 

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  __GPIOA_CLK_ENABLE();
  // backlight full off during screen init to avoid blink
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

  __GPIOC_CLK_ENABLE();
  /* PC9 AUX_PWR_enable, PC8 button_enable */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // keep power on
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0); 
  
  clock_config();


  // low powering the whole chip by disabling all possible boot pull resistors
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9/*|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14*/ /*|GPIO_PIN_15*/;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|/*GPIO_PIN_8|GPIO_PIN_9|*/GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  __GPIOD_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  __GPIOH_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  clock_high();

  // sort out interrupt priority to avoid complete lockup
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 4, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 4, 0);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_SetPriority(BNRG_SPI_EXTI_IRQn, 3, 0);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0); // avoid overrun error on the SE iso link when receiving

  // initialize the button press duration
  // button_it configuration for press delay determination
  GPIO_InitStruct.Pin = /*GPIO_PIN_3|*/GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  

  // pwr
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

  // enable detection of release (in case releasing after power off duration)
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  // start the ticker for the tsc to work correctly (warning ARM hal set the systick prio !!)
  G_io_seproxyhal_ticker_current_ms = 0;
  G_io_seproxyhal_ticker_enabled = 0;
  G_battery_test_ms = 0;
  G_watchdog_ms = 0;
  G_poweroff_ms = 0;
  G_backlight_autooff_ms = 0;
  G_backlight_autodim_ms = 0;
  SYSTICK_power(1);

  // initialize screen session and charge pump, but no disoplay yet
  screen_init(0);
  
  G_io_button.boot_moment = 1;
  G_io_button.duration_ms = 0; // approx
  G_io_button.displayed_mode = MODE_BOOT;
  G_io_button.pressed = 1; // fake pressed (booting means pressed)

  // wait until button it is released (or the last click length has been reached)
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) && ! (G_io_button.displayed_mode & MODE_POWER_OFF)) {
    if (G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_POWER_ON 
      && !(G_io_button.displayed_mode & MODE_POWER_ON)) {

      // display boot screen as the delay is ok
      display_l4_boot();

      // only lit screen when logo is on screen
      screen_on();

    #if 1
      /* Peripheral clock enable */
      __TIM2_CLK_ENABLE();

      screen_brightness(BACKLIGHT_FULL_LEVEL);

      /**TIM2 GPIO Configuration    
      PA15     ------> TIM2_CH1 
      */
      GPIO_InitStruct.Pin = GPIO_PIN_15;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    #endif

      G_io_button.displayed_mode |= MODE_POWER_ON;
    }
  }

#if defined(DEBUG_BUTTON_NEVER_PUSHED) || defined (DEBUG_BUTTON_NOT_REQUIRED_FOR_POWER_ON)

  // display boot screen as the delay is ok
  display_l4_boot();

  // only lit screen when logo is on screen
  screen_on();

#if 1
  /* Peripheral clock enable */
  __TIM2_CLK_ENABLE();

  screen_brightness(BACKLIGHT_FULL_LEVEL);

  /**TIM2 GPIO Configuration    
  PA15     ------> TIM2_CH1 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

  G_io_button.displayed_mode |= MODE_POWER_ON;
#else
  // power off if not pushed long enough,
  if (!(G_io_button.displayed_mode & MODE_POWER_ON)) {
    harakiri();
  }
#endif // DEBUG_BUTTON_NEVER_PUSHED

  G_io_button.pressed = 0;
  G_io_button.boot_moment = 0;

#ifdef DEMO_LOGO
  G_io_button.displayed_mode = 0;
  for(;;);
#endif

  if (!SE_iso_power(1)) {
    // try to power up the SE, if we cannot, then stay in loader (special patch for nicolas)
    G_io_button.displayed_mode = MODE_MCU_BOOTLOADER;
  }
  SE_iso_power(0);

  __asm("cpsid i");

  // power on for short and medium press
  if (/*BUTTON_PRESS_DURATION_CTRL_BOOTLOADER != 0xFFFFFFFFUL use a nvram struct for these params (with antitearing)
    && */
    G_io_button.duration_ms < BUTTON_PRESS_DURATION_BOOT_CTRL_BOOTLOADER
    && ! (G_io_button.displayed_mode & MODE_MCU_BOOTLOADER)
#ifdef DEBUG_BUTTON_ALWAYS_PUSHED
    || (G_io_button.displayed_mode & MODE_POWER_OFF) 
#endif // DEBUG_BUTTON_ALWAYS_PUSHED
    ) {
    G_io_usb.bootloader = 0;
    G_io_button.displayed_mode = 0;

    __asm("cpsie i");

    // delegate by default (with indication of the timing for recovery/standard boot as the info is lost after delegation)
    bootloader_delegate_boot(G_io_button.duration_ms);

  }
  // else long press stays in L4 bootloader mode

  __asm("cpsid i");

  G_io_usb.bootloader = 1;
#else 
  // reset button state in seproxyhal (not bootloader) mode
  G_io_button.boot_moment = 0;
  G_io_button.pressed = 0;
  G_io_button.displayed_mode = 0;

  __asm("cpsid i");
  //clock_config(); // to setup frequency_hz, strangely it cannot be done ;( it seems like systicks becomes crazy after ward
  frequency_hz = 48000000;
  /* Configure the Vector Table location add offset address ------------------*/
  SCB->VTOR = &g_pfnVectors; /* Vector Table Relocation in Internal FLASH */
#endif // HAVE_BL

  __asm("cpsie i");

//#define SCREEN_TEST_AT_BOOT
#ifdef SCREEN_TEST_AT_BOOT

  /*
  {
    unsigned char buffer[32];
    HAL_I2C_Master_Receive(&hi2c, MSG2133_I2C_ADDR, buffer, 8, 1024);  // no timeout
  }
  */


  #ifdef DEMO_BAGL
  demo_bagl();
  #endif // DEMO_BAGL
  
  /*
  screen_printf("Test\nString\n value ");
  screen_printf("azertyuiopqsdfghjklmwxcvbn,;:!?./§ù*%%µ12345678901234567890)=°+~#{[|`\\^$@]}ABCDEFGHIJKLMNOPQRSTUVWXYZ\n%04d\nBOOT SCREEN TEST\n",2763);
  
  screen_printf("%.*H\n", 8, g_pcHex);
  */
  // force refresh and close screen session
  PRINTF("Ledger $lue\n");

#endif // SCREEN_TEST_AT_BOOT

  // test arm semihosting printf("FROM STM32L4\n");

reboot:

  // initialize screen session and charge pump
  screen_init(1);
  // modify screen "image"  
  screen_clear();


#ifdef HAVE_BL
  if (G_io_usb.bootloader) {
    HAL_Delay(500);


    display_l4_mode("Ledger Blue", "BOOTLOADER MODE");

    /* Init Device Library */
    USBD_Init(&USBD_Device, &HID_Desc, 0);
    
    /* Register the HID class */
    USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);

    USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_template_fops);
    
    /* Start Device Process */
    USBD_Start(&USBD_Device);

    // run the bootloader main
    bootloader_apdu_interp();
  }
  /*
  else {
    bootloader_delegate_boot(1);
  }
  */
#else
  // default is BLE managed asynch by the SE
  G_io_ble_apdu_protocol_enabled = 0;
  BLE_power(0, NULL);
  //KBD_power(0);
  // enable the UI
  //UI_led_power(0);
  //UI_power(1);

  //clock_config();

  PRINTF("Power up ST31\n");

  // USART test with hsi clock
  // open iso clock
  SE_iso_power(1);

  PRINTF("ATR: %.*H\n", 20, G_io_se_atr);

  // ATR for ST31 Ledger BL/BOLOS
  if (G_io_se_atr_length != 5) {   


    #warning THIS MODE MUST BE REMOVED LATER ON, replace with a script on a computer instead

    #if 0
    st31_loader_flash_bl();
    #else
    const char recovery_name [] = {AD_TYPE_COMPLETE_LOCAL_NAME,'L','e','d','g','e','r',' ', 'B','l','u','e' /*,' ', 'B','o','o','t','l','o','a','d','e','r'*/, '\0'};
    // request standard ISO apdu transport between the SE and the MCU instead of raw SPI packet (to save overhead)
    G_io_ble_apdu_protocol_enabled = 1;

    // ######### ENABLE BLE TRANSPORT
    // await for BLE connection to process APDU through ST ISO bootloader
    //BLE_power(1, recovery_name);
    BLE_power(0, recovery_name);

    HAL_Delay(500);
    display_l4_mode("Ledger Blue", "MANUFACTURER MODE");

    // ######### ENABLE USB TRANSPORT
    G_io_usb.bootloader = 1; // use the BL transport for apdu over usbhid
    /* Init Device Library */
    USBD_Init(&USBD_Device, &HID_Desc, 0);
    /* Register the HID class */
    USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);
    USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_template_fops);
    /* Start Device Process */
    USBD_Start(&USBD_Device);
    io_usb_hid_init();


    while(1) {

      screen_update_touch_event();

      // stop ble upon disconnection
      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_DISCONNECT) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BLE_DISCONNECT;

        BLE_power(0, NULL);
        BLE_power(1, recovery_name);
      }

      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_TOUCH) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_TOUCH;
        PRINTF("TT. ");
      }

      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BUTTON;

        // push event is not userful but for awakening
        backlight_enable(1);
      }

      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_RELEASE) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_RELEASE;

        // undim
        if (backlight_is_enabled()) {
          backlight_enable(1);
        }

        display_l4_mode_touch();
      }

      // check for an apdu via ble
      if (G_io_ble.apdu_available) {
        backlight_enable(1);

        // consume apdu availability under critical section
        __asm("cpsid i");
        G_io_ble.apdu_available = 0;
        __asm("cpsie i");
        __asm volatile ("nop"); 
        __asm volatile ("nop"); 

        // special reset command
        if (G_io_apdu_buffer[0] == 0xFE
          && G_io_apdu_buffer[1] == 0xFE
          && G_io_apdu_buffer[2] == 0xFE
          && G_io_apdu_buffer[3] == 0xFE) {
          G_io_apdu_buffer[0] = 0x90;
          G_io_apdu_buffer[1] = 0x00;
          BLE_protocol_send(G_io_apdu_buffer, 2);
          // just in case
          for (;;) {
            NVIC_SystemReset();
          }
        }
      
        // process and reply the apdu  
        BLE_protocol_send(G_io_apdu_buffer, SE_iso_exchange_apdu(G_io_apdu_buffer, G_io_ble.apdu_length)); 
        
        // process ble activity if any
        if (HCI_require_processing()) {
          SCB->ICSR |= 1<<28;
        }
      }

      // check for an apdu via usb
      if ((io_exchange(CHANNEL_APDU, 0))) {

        backlight_enable(1);

        // special reset command
        if (G_io_apdu_buffer[0] == 0xFE
          && G_io_apdu_buffer[1] == 0xFE
          && G_io_apdu_buffer[2] == 0xFE
          && G_io_apdu_buffer[3] == 0xFE) {
          G_io_apdu_buffer[0] = 0x90;
          G_io_apdu_buffer[1] = 0x00;
          io_exchange(CHANNEL_APDU|IO_RETURN_AFTER_TX|IO_RESET_AFTER_REPLIED, 2);
          // just in case
          for (;;) {
            NVIC_SystemReset();
          }
        }
        io_exchange(CHANNEL_APDU|IO_RETURN_AFTER_TX, SE_iso_exchange_apdu(G_io_apdu_buffer, G_io_apdu_length));
      }
    }
    #endif

    // reset the chip to check out the new ATR
    NVIC_SystemReset();
  }

  // session start the SE
  G_io_se_powered = 0;

  G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_SESSION_START_EVENT;
  G_io_seproxyhal_buffer[1] = 0;
  G_io_seproxyhal_buffer[2] = 1;
  // ask SE recovery when button press is longer than the required delay
#ifdef FORCE_ST31_RECOVERY
  G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_SESSION_START_EVENT_RECOVERY; 
#else
  G_io_seproxyhal_buffer[3] = button_press_duration >= BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY ? SEPROXYHAL_TAG_SESSION_START_EVENT_RECOVERY:0;
#endif
#ifdef FORCE_ST31_BACK_TO_STLOADER
  G_io_seproxyhal_buffer[3] = 4;
#endif // FORCE_ST31_BACK_TO_STLOADER
#ifdef DEBUG_BUTTON_ALWAYS_PUSHED
  G_io_seproxyhal_buffer[3] = 0;
#endif // DEBUG_BUTTON_ALWAYS_PUSHED

  io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
  G_io_seproxyhal_state = WAIT_COMMAND;

  volatile unsigned short rx;
  volatile unsigned short tx;
  volatile unsigned int i;
  volatile unsigned int l;
  volatile unsigned int x,y;

#ifdef IO_USB_HID
  io_usb_hid_init();
#endif // IO_USB_HID

  union {
    bagl_component_t c;
    unsigned char raw[128];
  } bagl_e;
  unsigned int bagl_l;

  // init unsecure firm signature check
  G_io_unsec_chunk.offset = 0;

  for(;;) {     
    switch (G_io_seproxyhal_state) {
      case WAIT_EVENT:

        // check if any message is comming from the SE, it is unexpected but for printf

        // avoid polling, stay low power
        rx = io_seproxyhal_rx_available();
        if (rx >= 3) {
          // receive tlv header
          rx = io_seproxyhal_recv(G_io_seproxyhal_buffer, sizeof(G_io_seproxyhal_buffer));
          l = (G_io_seproxyhal_buffer[1]<<8)|(G_io_seproxyhal_buffer[2]&0xFF);

          // reset power off, a new command has been received (even if not expected)
          backlight_enable(1);

          switch(G_io_seproxyhal_buffer[0]) {
            default:
            case SEPROXYHAL_TAG_BLE_RADIO_POWER:
            case SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS:
            case SEPROXYHAL_TAG_SCREEN_POWER:
            case SEPROXYHAL_TAG_MORE_TIME:
            case SEPROXYHAL_TAG_DEVICE_OFF:
            case SEPROXYHAL_TAG_SE_POWER_OFF:
            case SEPROXYHAL_TAG_SET_TICKER_INTERVAL:
            case SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS: 
            case SEPROXYHAL_TAG_NFC_READ_RESPONSE_STATUS:
            case SEPROXYHAL_TAG_USB_CONFIG:
            case SEPROXYHAL_TAG_USB_EP_PREPARE:
            case SEPROXYHAL_TAG_GO_BOOTLOADER:
              PRINTF("tlv: %.*H\n", MIN(l+3, 256), G_io_seproxyhal_buffer);
              PRINTF("  unexpected tag from SE: 0x%02X\n", G_io_seproxyhal_buffer[0]);
              break;

            // interpret printf within wait event, for easier debug of hardfault on the SE 
            // side (and of various invalid protocol handling)
            case SEPROXYHAL_TAG_PRINTF_STATUS: {
              unsigned char* pc = &G_io_seproxyhal_buffer[3];
              //poor's man printf pipe
              while(l--) {
                screen_printc(*pc++);
              }
              // printf ok
              G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
              G_io_seproxyhal_buffer[1] = 0;
              G_io_seproxyhal_buffer[2] = 0;

              io_seproxyhal_send(G_io_seproxyhal_buffer, 3);
              break;
            }
          }
        }

        // check for touch events (i2c touchscreen is somewhat interacting with usart)
        screen_update_touch_event();

        // ready to send the next incoming event (BLE/NFC/TOUCH/WATCHDOG/USB etc)
        if (G_io_seproxyhal_events) {
        
          volatile unsigned int consumed_events = 0;
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_SET_LINK_SPEED) {
            // change speed
            SE_set_link_speed(G_io_se_link_next_mhz, G_io_se_link_next_etu);

            // add some delay ?? for SE to setup the new link speed
            consumed_events = SEPROXYHAL_EVENT_SET_LINK_SPEED;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;  
          }
          // power off
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_WATCHDOG) {
            // VERY OVERPOWERED => chlik !chlak
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_UNSEC_CHUNK) {
            #define UNSEC_TOTAL_LENGTH (&_esignature-&_signed)
            #define UNSEC_SIGNED_LENGTH (&_esigned-&_signed)

            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_UNSEC_CHUNK_EVENT;
            G_io_seproxyhal_buffer[1] = G_io_unsec_chunk.size>>8;
            G_io_seproxyhal_buffer[2] = G_io_unsec_chunk.size;

            // send in 2 packets
            io_seproxyhal_send(G_io_seproxyhal_buffer, 3);

            // first packet encode the total length on a U4BE
            if (G_io_unsec_chunk.offset==0) {
              G_io_unsec_chunk.size-=4;
              G_io_seproxyhal_buffer[0] = UNSEC_SIGNED_LENGTH>>24;
              G_io_seproxyhal_buffer[1] = UNSEC_SIGNED_LENGTH>>16;
              G_io_seproxyhal_buffer[2] = UNSEC_SIGNED_LENGTH>>8;
              G_io_seproxyhal_buffer[3] = UNSEC_SIGNED_LENGTH;
              io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            }

            if (G_io_unsec_chunk.size) {
              io_seproxyhal_send((void*)(((unsigned int)&_signed)+G_io_unsec_chunk.offset), G_io_unsec_chunk.size);
            }

            // prepare next chunk if any
            G_io_unsec_chunk.offset+=G_io_unsec_chunk.size;

            consumed_events = SEPROXYHAL_EVENT_UNSEC_CHUNK;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;            
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_CONNECT) {
            PRINTF("BC. ");
            
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_CONNECTION_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = 1; // conn
            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);

            consumed_events = SEPROXYHAL_EVENT_BLE_CONNECT;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_DISCONNECT) {
            PRINTF("BD. ");
            
            #if 0
            // special for dev only, else trust the application to perform correctly
            NVIC_SystemReset();
            #endif 

            #if 1
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_CONNECTION_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = 0; // disconn
            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            #else
            BLE_power(0, NULL);
            BLE_power(1, NULL);
            #endif

            consumed_events = SEPROXYHAL_EVENT_BLE_DISCONNECT;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_WRITE) {
            //PRINTF("BW. ");
            // forge BLE packet to the SE
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_WRITE_REQUEST_EVENT;
            G_io_seproxyhal_buffer[1] = (G_io_ble.last_write_size+3)>>8;
            G_io_seproxyhal_buffer[2] = (G_io_ble.last_write_size+3);
            for (i = 0; i < IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT; i++) {
              if (G_io_seproxyhal_ble_handles[i] == G_io_ble.last_write_attr_handle) {
                G_io_seproxyhal_buffer[3] = i;
                break;
              }
            }
            G_io_seproxyhal_buffer[4] = G_io_ble.last_write_size>>8;
            G_io_seproxyhal_buffer[5] = G_io_ble.last_write_size;

            // send in 2 packets
            io_seproxyhal_send(G_io_seproxyhal_buffer, 3+3);
            io_seproxyhal_send(G_io_ble.last_write_buffer, G_io_ble.last_write_size);

            // switch to command state until the SE replies a general status
            G_io_seproxyhal_state = WAIT_COMMAND;
            // validate write on the blue tooth media, to avoid overwriting the previous value
            BLE_accept_previous_write();
            consumed_events = SEPROXYHAL_EVENT_BLE_WRITE;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_READ) {
            //PRINTF("BR. ");
            for (i = 0; i < IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT; i++) {
              if (G_io_seproxyhal_ble_handles[i] == G_io_ble.last_read_attr_handle) {
                G_io_seproxyhal_ble_last_read_request_handle = i;
                break;
              }
            }
            G_io_seproxyhal_state = WAIT_BLE_READ_DATA;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_READ_REQUEST_EVENT;
            // TODO send the read event to the SE
            consumed_events = SEPROXYHAL_EVENT_BLE_READ;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER) {
            PRINTF("NR. ");
            consumed_events = SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER) {
            PRINTF("NU. ");
            consumed_events = SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER;
            goto consume;
          }

          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {

            // when button is clicked, don't send to the SE when the backlight was off, just awake
            if (!backlight_is_enabled()) {
              backlight_enable(1);
            }
            else {
              G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BUTTON_PUSH_EVENT;
              G_io_seproxyhal_buffer[1] = 0;
              G_io_seproxyhal_buffer[2] = 1;
              G_io_seproxyhal_buffer[3] = (G_io_button.duration_ms >= SEPROXYHAL_LONG_BUTTON_PUSH_MS)?1:0;
              io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
              G_io_seproxyhal_state = WAIT_COMMAND;
            }
            consumed_events = SEPROXYHAL_EVENT_BUTTON;
            goto consume;
          }

          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_RELEASE) {
            consumed_events = SEPROXYHAL_EVENT_RELEASE;
           
            // avoid clicking anywhere, user must use the button to awake from sleep
            if (!backlight_is_enabled()) {
              goto consume_nowakeup;
            }

            //PRINTF("TR. ");
            x = G_io_touch.ts_last_x;
            y = G_io_touch.ts_last_y;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_FINGER_EVENT_RELEASE;
            // consume x&y to avoid multiple touch release (thanks SUPERB touchscreen impl)
            G_io_touch.ts_last_x = -1;
            G_io_touch.ts_last_y = -1;
            goto send_finger_event;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_TOUCH) {
            consumed_events = SEPROXYHAL_EVENT_TOUCH;
           
            // avoid clicking anywhere, user must use the button to awake from sleep
            if (!backlight_is_enabled()) {
              goto consume_nowakeup;
            }

            //PRINTF("TT. ");
            x = G_io_touch.ts_last_x;
            y = G_io_touch.ts_last_y;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_FINGER_EVENT_TOUCH;
          send_finger_event:
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_FINGER_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 5;
            // [3] already set
            G_io_seproxyhal_buffer[4] = x>>8;
            G_io_seproxyhal_buffer[5] = x;
            G_io_seproxyhal_buffer[6] = y>>8;
            G_io_seproxyhal_buffer[7] = y;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 8);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_RESET) {
            //PRINTF("UR. ");
            consumed_events = SEPROXYHAL_EVENT_USB_RESET;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_RESET;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_SUSPENDED) {
            //PRINTF("US. ");
            consumed_events = SEPROXYHAL_EVENT_USB_SUSPENDED;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_SUSPENDED;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_RESUMED) {
            //PRINTF("UP. ");
            consumed_events = SEPROXYHAL_EVENT_USB_RESUMED;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_RESUMED;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_SETUP) {
            //PRINTF("UT. ");
            consumed_events = SEPROXYHAL_EVENT_USB_SETUP;

            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EP_XFER_EVENT;
            G_io_seproxyhal_buffer[1] = (3+8)>>8;
            G_io_seproxyhal_buffer[2] = (3+8);
            G_io_seproxyhal_buffer[3] = 0;
            G_io_seproxyhal_buffer[4] = SEPROXYHAL_TAG_USB_EP_XFER_SETUP;
            G_io_seproxyhal_buffer[5] = 8;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 6);
            io_seproxyhal_send(hpcd_USB_OTG_FS.Setup, 8);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_XFER_IN) {
            unsigned char epnum;
            //PRINTF("UX. ");
            // process in before out to ensure device holds the DoS way
            if (G_io_usb.ep_in) {
              for (epnum=0; epnum<MAX_USB_ENDPOINTS; epnum++) {
                if (G_io_usb.ep_in & (1<<epnum)) {
                  G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EP_XFER_EVENT;
                  G_io_seproxyhal_buffer[1] = (3)>>8;
                  G_io_seproxyhal_buffer[2] = (3);
                  G_io_seproxyhal_buffer[3] = epnum|0x80;
                  G_io_seproxyhal_buffer[4] = SEPROXYHAL_TAG_USB_EP_XFER_IN;
                  G_io_seproxyhal_buffer[5] = G_io_usb.ep_in_len[epnum];

                  io_seproxyhal_send(G_io_seproxyhal_buffer, 6);
                  G_io_seproxyhal_state = WAIT_COMMAND;

                  __asm("cpsid i");
                  G_io_usb.ep_in &= ~(1<<epnum);
                  if (!G_io_usb.ep_in) {
                    G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_XFER_IN;
                  }
                  __asm("cpsie i");
                  goto consumed;
                }
              }
            }
            // nothing to be done, consume the event
            consumed_events = SEPROXYHAL_EVENT_USB_XFER_OUT;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_XFER_OUT) {
            unsigned char epnum;
            if (G_io_usb.ep_out) {
              for (epnum=0; epnum<MAX_USB_ENDPOINTS; epnum++) {
                if (G_io_usb.ep_out & (1<<epnum)) {
                  unsigned char len = G_io_usb.ep_out_len[epnum];
                  G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EP_XFER_EVENT;
                  G_io_seproxyhal_buffer[1] = (3+len)>>8;
                  G_io_seproxyhal_buffer[2] = (3+len);
                  G_io_seproxyhal_buffer[3] = epnum;
                  G_io_seproxyhal_buffer[4] = SEPROXYHAL_TAG_USB_EP_XFER_OUT;
                  G_io_seproxyhal_buffer[5] = len;

                  io_seproxyhal_send(G_io_seproxyhal_buffer, 6);
                  // xfer_buff is incremented
                  io_seproxyhal_send(G_io_usb.ep_out_buff[epnum], 
                                     len);
 
                  G_io_seproxyhal_state = WAIT_COMMAND;
                  __asm("cpsid i");
                  G_io_usb.ep_out &= ~(1<<epnum);
                  if (!G_io_usb.ep_out) {
                    G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_XFER_OUT;
                  }
                  __asm("cpsie i");
                  goto consumed;
                }
              }
            }
            // nothing to be done, consume the event
            consumed_events = SEPROXYHAL_EVENT_USB_XFER_OUT;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_SOF) {
            //PRINTF("UF. ");
            consumed_events = SEPROXYHAL_EVENT_USB_SOF;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_SOF;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }

          // last to avoid overflowing io events with a too short ticker
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_TICKER) {
            //PRINTF("TI. ");
            consumed_events = SEPROXYHAL_EVENT_TICKER;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_TICKER_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0; 

            io_seproxyhal_send(G_io_seproxyhal_buffer, 3);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }

#ifdef IO_EVENT_BEFORE_DISPLAY_PROCESSED_EVENT
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_DISPLAYED) {
            consumed_events = SEPROXYHAL_EVENT_DISPLAYED;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0; 

            io_seproxyhal_send(G_io_seproxyhal_buffer, 3);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }

#endif // IO_EVENT_BEFORE_DISPLAY_PROCESSED_EVENT

          // no event to be consumed
          break;

        consume_nowakeup:
          __asm("cpsid i");
          G_io_seproxyhal_events &= ~consumed_events;
          __asm("cpsie i");
          break;

        consume:
          __asm("cpsid i");
          G_io_seproxyhal_events &= ~consumed_events;
          __asm("cpsie i");
        consumed:
          // reset power off, an event command has been received
          backlight_enable(1);
        }
        break;

      case WAIT_BLE_READ_DATA:
      case WAIT_COMMAND:
        // avoid polling, stay low power
        rx = io_seproxyhal_rx_available();

        // discard invalid replies
        if (rx < 3) {
          // nothing to do yet
          break;
        }
        // receive tlv header
        rx = io_seproxyhal_recv(G_io_seproxyhal_buffer, sizeof(G_io_seproxyhal_buffer));
        l = (G_io_seproxyhal_buffer[1]<<8)|(G_io_seproxyhal_buffer[2]&0xFF);

        // reset power off, a new command has been received
        backlight_enable(1);

        //PRINTF("%.*H\n", MIN(l+3, 256), G_io_seproxyhal_buffer);

        switch(G_io_seproxyhal_buffer[0]) {
          case SEPROXYHAL_TAG_BLE_RADIO_POWER:
            if (((G_io_seproxyhal_buffer[3]?1:0) ^ G_io_se_powered) == 0) {
              // ignore
              continue;
            }
            PRINTF("BO. ");
            // turn BLE ON or OFF, use the last defined service. (make discoverable)
            BLE_power(G_io_seproxyhal_buffer[3]&0x2, NULL); // if not advertising, then don't turn on
            // set default handles
            memset(G_io_seproxyhal_ble_handles, 0, sizeof(G_io_seproxyhal_ble_handles));
            G_io_seproxyhal_ble_handles[1] = G_io_ble.tx_characteristic_handle ;
            G_io_seproxyhal_ble_handles[2] = G_io_ble.rx_characteristic_handle + 1;

            G_io_se_powered = G_io_seproxyhal_buffer[3]?1:0;
            break;
          case SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS:
            //PRINTF("BI. ");
            if (l > 3) {
              // will only return when data has been propagated. timeout => watchdog
              BLE_send(G_io_seproxyhal_ble_handles[G_io_seproxyhal_buffer[3]], &G_io_seproxyhal_buffer[6], l-3);
            }

            // say ok to the SE
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            // unchanged: G_io_seproxyhal_buffer[3] = G_io_seproxyhal_buffer[3];

            io_seproxyhal_send(G_io_seproxyhal_buffer, 4);

            // stay in command mode
            break;
          case SEPROXYHAL_TAG_SCREEN_POWER:
            // TODO PWN off
            break;
          case SEPROXYHAL_TAG_MORE_TIME:
            // TODO add watchdog time
            break;
          case SEPROXYHAL_TAG_DEVICE_OFF:
            harakiri();
            break;

          case SEPROXYHAL_TAG_SE_POWER_OFF:
            // reset the SE using a power sequence, and go back to interpretation of the ATR <=> system reset
            goto reboot;

          case SEPROXYHAL_TAG_SET_TICKER_INTERVAL: {
            PRINTF("TK ");
            if (l == 2) { 
              // ticker interval ms in U2BE
              G_io_seproxyhal_ticker_interval_ms = (G_io_seproxyhal_buffer[3]<<8)|(G_io_seproxyhal_buffer[4]&0xFF);
              // when interval is 0 then ticker is disabled
              G_io_seproxyhal_ticker_enabled = (G_io_seproxyhal_ticker_interval_ms != 0);
            }
            break;
          }

          case SEPROXYHAL_TAG_PRINTF_STATUS: {
            unsigned char* pc = &G_io_seproxyhal_buffer[3];
            //poor's man printf pipe
            while(l--) {
              screen_printc(*pc++);
            }
            // printf ok
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0;

            io_seproxyhal_send(G_io_seproxyhal_buffer, 3);
            break;
          }
          case SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS: {
            //PRINTF("CD. ");
            if (l < sizeof(bagl_component_t)) {
              PRINTF("INVALID");
              // too short packet
              break;
            }

            // check last display value to avoid blink without support in the application
            if (l != bagl_l 
              || memcmp(&bagl_e, &G_io_seproxyhal_buffer[3], MIN(sizeof(bagl_e), l)) != 0) {

              // little endian to little endian, cross finger and hope for same fields alignment
              memcpy(&bagl_e, &G_io_seproxyhal_buffer[3], MIN(sizeof(bagl_e), l));
              bagl_l = l;

              // magic replace if necessary
              if (((l - sizeof(bagl_component_t)) >= sizeof(ESC_DEVICE_NAME) - 1) && 
                  (memcmp(&G_io_seproxyhal_buffer[3+sizeof(bagl_component_t)], ESC_DEVICE_NAME, sizeof(ESC_DEVICE_NAME) - 1) == 0)) {
                if (G_io_se_powered) {
                  bagl_draw_with_context(&bagl_e.c, G_io_ble.last_discovered_name + 1, strlen(G_io_ble.last_discovered_name + 1), BAGL_ENCODING_LATIN1);
                } else {
                  bagl_draw_with_context(&bagl_e.c, DEVICE_NAME_NA, strlen(DEVICE_NAME_NA), BAGL_ENCODING_LATIN1);                
                }
              }
              else {
                bagl_draw_with_context(&bagl_e.c, &G_io_seproxyhal_buffer[3+sizeof(bagl_component_t)], l-sizeof(bagl_component_t), BAGL_ENCODING_LATIN1);
              }
            }
            #ifdef IO_EVENT_BEFORE_DISPLAY_PROCESSED_EVENT
            // ensure to 
            G_io_seproxyhal_state |= SEPROXYHAL_EVENT_DISPLAYED;
            G_io_seproxyhal_state = WAIT_EVENT;
            #else
            // reply display processed event
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0;
            io_seproxyhal_send(G_io_seproxyhal_buffer, 3);
            #endif 
            // stay in command state
            break;
          }
          case SEPROXYHAL_TAG_NFC_READ_RESPONSE_STATUS:
            if (G_io_seproxyhal_state != WAIT_BLE_READ_DATA) {
              continue;
            }            
            BLE_send(G_io_seproxyhal_ble_last_read_request_handle, &G_io_seproxyhal_buffer[4], G_io_seproxyhal_buffer[0] - 4);
            // this is the last command
            G_io_seproxyhal_state = WAIT_EVENT;
            break;
          case SEPROXYHAL_TAG_GENERAL_STATUS:
            //PRINTF("END ");
            if (l == 2) {
              switch((G_io_seproxyhal_buffer[3]<<8)|(G_io_seproxyhal_buffer[4]&0xFF)) {
                case SEPROXYHAL_TAG_GENERAL_STATUS_LAST_COMMAND:
                  G_io_seproxyhal_state = WAIT_EVENT;
                  break;
                case SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND:
                default:
                  // remain in wait command
                  break;
              }
            }
            break;

          // disable to remove USB support (on STM32L4 nucleo for example)
          #if 1
          case SEPROXYHAL_TAG_USB_CONFIG:
            switch (G_io_seproxyhal_buffer[3]) {

              case SEPROXYHAL_TAG_USB_CONFIG_CONNECT:
                // already done at boot // clock_config();

                // startup the low level usb stack
                USBD_Device.pClass = NULL;
                USBD_LL_Init(&USBD_Device);
                USBD_LL_Start(&USBD_Device);

                // enable usb interrupt
                //HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
                break;
              case SEPROXYHAL_TAG_USB_CONFIG_DISCONNECT:
                USBD_LL_Stop(&USBD_Device); 
                break;
              case SEPROXYHAL_TAG_USB_CONFIG_ADDR:
                USBD_LL_SetUSBAddress(&USBD_Device, G_io_seproxyhal_buffer[4]);
                break;
              case SEPROXYHAL_TAG_USB_CONFIG_ENDPOINTS:
                // configure all endpoints
                for(l=0;l<G_io_seproxyhal_buffer[4];l++) {
                  if (G_io_seproxyhal_buffer[5+l*3+1] == SEPROXYHAL_TAG_USB_CONFIG_TYPE_DISABLED) {
                    // still skip the other bytes
                    USBD_LL_CloseEP(&USBD_Device, G_io_seproxyhal_buffer[5+l*3]);
                  }
                  else {
                    uint8_t ep_type = -1;
                    switch(G_io_seproxyhal_buffer[5+l*3+1]) {
                      case SEPROXYHAL_TAG_USB_CONFIG_TYPE_CONTROL:
                        ep_type = USBD_EP_TYPE_CTRL;
                        break;
                      case SEPROXYHAL_TAG_USB_CONFIG_TYPE_ISOCHRONOUS:
                        ep_type = USBD_EP_TYPE_ISOC;
                        break;
                      case SEPROXYHAL_TAG_USB_CONFIG_TYPE_BULK:
                        ep_type = USBD_EP_TYPE_BULK;
                        break;
                      case SEPROXYHAL_TAG_USB_CONFIG_TYPE_INTERRUPT:
                        ep_type = USBD_EP_TYPE_INTR;
                        break;
                    }
                    USBD_LL_OpenEP(&USBD_Device, 
                                   G_io_seproxyhal_buffer[5+l*3], 
                                   ep_type, 
                                   G_io_seproxyhal_buffer[5+l*3+2]);
                  }
                }
                break;
            }
            break;

          case SEPROXYHAL_TAG_USB_EP_PREPARE:
            switch (G_io_seproxyhal_buffer[4]) { 
              case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_UNSTALL:
                USBD_LL_ClearStallEP(&USBD_Device, G_io_seproxyhal_buffer[3]);
                break;

              case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_STALL:
                USBD_LL_StallEP(&USBD_Device, G_io_seproxyhal_buffer[3]);
                break;

              case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_IN:
                USBD_LL_Transmit(&USBD_Device, 
                                 G_io_seproxyhal_buffer[3], 
                                 G_io_seproxyhal_buffer+6, 
                                 G_io_seproxyhal_buffer[5]);
                break;

              case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_SETUP:
              case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_OUT:
                USBD_LL_PrepareReceive(&USBD_Device, 
                                       G_io_seproxyhal_buffer[3], 
                                       (G_io_seproxyhal_buffer[5]>MAX_USB_ENDPOINT_SIZE?MAX_USB_ENDPOINT_SIZE:G_io_seproxyhal_buffer[5]));
                break;
            }
            break;
          #endif 

          case SEPROXYHAL_TAG_GO_BOOTLOADER:
            // return to bootloader
            return;

          case SEPROXYHAL_TAG_UNSEC_CHUNK_READ:
            // reset the unsec offset read if requested
            if (G_io_seproxyhal_buffer[5]) {
              G_io_unsec_chunk.offset = 0;
            }

            // | vector | bootloader | [gap] | seproxyhal | signature |
            // ^_signed                      ^ _text      ^_esigned   ^_esignature
            G_io_unsec_chunk.size = MIN((&_esignature-&_signed)-G_io_unsec_chunk.offset, 
                                        (G_io_seproxyhal_buffer[3]<<8) | G_io_seproxyhal_buffer[4]);
										
            // prepare async event after get status
            __asm("cpsid i");
            G_io_seproxyhal_events |= SEPROXYHAL_EVENT_UNSEC_CHUNK;
            __asm("cpsie i");
            break;

          case SEPROXYHAL_TAG_SET_LINK_SPEED: {
            unsigned int mhz = G_io_seproxyhal_buffer[3];
            unsigned int etu = G_io_seproxyhal_buffer[4];

            // ack speed change
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_ACK_LINK_SPEED;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 3;
            // unchanged since recv // G_io_seproxyhal_buffer[3] = mhz;
            //G_io_seproxyhal_buffer[3] = 10; // force mhz
            // unchanged since recv // G_io_seproxyhal_buffer[4] = etu;
            if (l == 2) {
              G_io_seproxyhal_buffer[5] = 1; //ACK

              G_io_se_link_next_mhz = mhz;
              G_io_se_link_next_etu = etu;

              __asm("cpsid i");
              G_io_seproxyhal_events |= SEPROXYHAL_EVENT_SET_LINK_SPEED;
              __asm("cpsie i");
            }
            else {
              G_io_seproxyhal_buffer[5] = 0; //NACK
            }
            io_seproxyhal_send(G_io_seproxyhal_buffer, 6);
            G_io_seproxyhal_state = WAIT_EVENT;
            break;
          }
        }
        break;
    }
  }

  #endif // HAVE_BL

  // shall never be reached
  for(;;);
}


/**
 * @}
 */

/**
 * @}
 */

