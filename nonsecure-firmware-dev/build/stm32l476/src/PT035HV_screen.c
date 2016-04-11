
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_tim.h"

// for varargs support in printf
#include <stdarg.h>
#include <string.h>

#ifdef HAVE_PT035HV
//#define HAVE_PT035HV_ROTATION_0
//#define HAVE_PT035HV_ROTATION_180

#include "bagl.h"

#include "seproxyhal.h"


// invert to display on a negative polarized screen
//#define HAVE_64128M_BLACK_ON_WHITE


/**
 * N lines of M chars on font 6x8
 * [{line0}{line1}...{line7}]
 */

/*
char screen_char_buffer[SCREEN_LINE_COUNT*SCREEN_LINE_CHAR_COUNT];
*/

extern SPI_HandleTypeDef hspi;

#define SCREEN_HEIGHT 480
#define SCREEN_WIDTH 320
#define PRINTF_ZONE_HEIGHT (56+1)
#define PRINTF_FONT_HEIGHT 8
#define PRINTF_FONT_WIDTH 5
#define PRINTF_FONT_ID BAGL_FONT_LUCIDA_CONSOLE_8
#define PRINTF_LINE_CHAR_LENGTH (SCREEN_WIDTH/PRINTF_FONT_WIDTH)
#define PRINTF_LINE_COUNT (PRINTF_ZONE_HEIGHT/PRINTF_FONT_HEIGHT)
unsigned char screen_charbuffer[PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT];


#define DISP_NRST(x) BB_OUT(GPIOC, 1,x)
#define DISP_NCS(x) BB_OUT(GPIOB, 14,x)
#define TS_CE(x) BB_OUT(GPIOC, 10, x)


#define MSG2133_I2C_ADDR 0x4C
I2C_HandleTypeDef hi2c;

#define FLAG_COMMAND 1
#define FLAG_PARAM 2
#define FLAG_READ 4
//#define FLAG_WRITE 8
#define FLAG_DUMMY 16
#define FLAG_FLUSH 32

unsigned char lcd_data(unsigned int flags, unsigned char d) {
  
  unsigned int tmp;
  if (flags&FLAG_PARAM) {
    tmp = 0x100 | d;
  }
  else {
    tmp = d;
  }

  SPI2->DR = tmp; // write data to be transmitted to the SPI data register
  while( !(SPI2->SR & SPI_SR_TXE) ); // wait until transmit complete

  return 0;
}

//--------------------------------------------------------------------------//
// HX8357 Driver Registers
//--------------------------------------------------------------------------//
#define HX8357_NOP                        0x00
#define HX8357_EXIT_SLEEP_MODE            0x11
#define HX8357_NORON                      0x13
#define HX8357_SET_DISPLAY_OFF            0x28
#define HX8357_SET_DISPLAY_ON             0x29
#define HX8357_SET_COLUMN_ADDRESS         0x2A
#define HX8357_SET_PAGE_ADDRESS           0x2B
#define HX8357_WRITE_MEMORY_START         0x2C
#define HX8357_READ_MEMORY_START          0x2E
#define HX8357_SET_TEAR_OFF               0x34
#define HX8357_SET_TEAR_ON                0x35
#define HX8357_SET_ADDRESS_MODE           0x36
#define HX8357_SET_PIXEL_FORMAT           0x3A
#define HX8357_WRITE_MEMORY_CONTINUE      0x3C
#define HX8357_READ_MEMORY_CONTINUE       0x3E
#define HX8357_SET_INTERNAL_OSCILLATOR    0xB0
#define HX8357_SET_POWER_CONTROL          0xB1
#define HX8357_SET_DISPLAY_MODE           0xB4
#define HX8357_SET_BGP                    0xB5
#define HX8357_SET_VCOM_VOLTAGE           0xB6
#define HX8357_ENABLE_EXTENSION_COMMAND   0xB9
#define HX8357_SET_PANEL_DRIVING          0xC0    // not documented!
#define HX8357_SET_PANEL_CHARACTERISTIC   0xCC
#define HX8357_SET_GAMMA_CURVE            0xE0

#define HX8357_ALLON            0x23
#define HX8357_ALLOFF           0x22
#define HX8357_INVON            0x21
#define HX8357_INVOFF           0x20
#define HX8357_DISPLAYOFF         0x28
#define HX8357_DISPLAYON          0x29

void screen_set_xy(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) 
{
 lcd_data(FLAG_COMMAND, HX8357_SET_COLUMN_ADDRESS); // Column addr set
 #define colstart 0
 lcd_data(FLAG_PARAM, (x0+colstart) >> 8);
 lcd_data(FLAG_PARAM, x0+colstart);     // XSTART 
 lcd_data(FLAG_PARAM, (x1+colstart) >> 8);
 lcd_data(FLAG_PARAM, x1+colstart);     // XEND

 lcd_data(FLAG_COMMAND, HX8357_SET_PAGE_ADDRESS); // Row addr set
 #define rowstart 0
 lcd_data(FLAG_PARAM, (y0+rowstart) >> 8);
 lcd_data(FLAG_PARAM, y0+rowstart);     // YSTART
 lcd_data(FLAG_PARAM, (y1+rowstart) >> 8);
 lcd_data(FLAG_PARAM, y1+rowstart);     // YEND

}


void screen_poweroff(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  // TODO low power the screen and touchscreen

}

/**
  *
  *
  */
unsigned int finger_index;
void EXTI15_10_IRQHandler(void) {
  // then it's the touch screen that interrupted
  // read the touch screen position data
  unsigned char buffer[32];
  HAL_I2C_Master_Receive(&hi2c, MSG2133_I2C_ADDR, buffer, 8, 1024);  // no timeout

  /*
  // light on upon touch
  UI_led_power(1);
  */

  if (buffer[0] == 0x52) {
    switch(buffer[1]) {
      case 0xFF:
        // releasing !!
        // only one release event to lower the cpu load
        if (G_io_touch.ts_last_x >= 0 && G_io_touch.ts_last_y >= 0) {
          __asm("cpsid i");
          G_io_seproxyhal_events |= SEPROXYHAL_EVENT_RELEASE;
          __asm("cpsie i");
        }
        break;
      default: {
        uint16_t x1, y1;
        x1 = ((buffer[1] & 0xF0) << 4) | buffer[2];
        y1 = ((buffer[1] & 0x0F) << 8) | buffer[3];
        
        //#ifdef YL035_2_FINGERS
        int16_t x2, y2;
        x2 = ((buffer[4] & 0xF0) << 4) | buffer[5];
        if (x2 & 0x800) {
          x2 |= 0xF000;
        }
        y2 = ((buffer[4] & 0x0F) << 8) | buffer[6];
        if (y2 & 0x800) {
          y2 |= 0xF000;
        }
        //x2 = x2 *15 / 64;//480/2048;
        //y2 = y2 * 5 / 32;//320/2048;
        //#endif // YL035_2_FINGERS

        // alternatively switch between fingers when a second coordinate is present
        if (finger_index && (x2 || y2)) {
          x2 += x1;
          y2 += y1;
          finger_index = 0;
          x1 = x2;
          y1 = y2;
        }
        else {
          finger_index = 1;
        }


        // adjust the second finger with the offset of the first
        x1 = x1 *15 / 64;//480/2048;
        y1 = y1 * 5 / 32;//320/2048;
        
        /*
        HAL_UART_Transmit(&huart1, (uint8_t*)&x1, 2, 1024);
        HAL_UART_Transmit(&huart1, (uint8_t*)&y1, 2, 1024);
        */
        // reverse the y coord
#ifdef HAVE_PT035HV_ROTATION_0
        G_io_touch.ts_last_x = SCREEN_WIDTH-y1;
        G_io_touch.ts_last_y = x1;
#endif // HAVE_PT035HV_ROTATION_0
#ifdef HAVE_PT035HV_ROTATION_180
        G_io_touch.ts_last_x = y1;
        G_io_touch.ts_last_y = SCREEN_HEIGHT-x1;
#endif // HAVE_PT035HV_ROTATION_180

        __asm("cpsid i");
        G_io_seproxyhal_events |= SEPROXYHAL_EVENT_TOUCH;
        __asm("cpsie i");
      }
    }
  }    
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
}

// manually pump the touch events to avoid problems of the USART/I2C conflicts
void screen_update_touch_event(void) {
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    EXTI15_10_IRQHandler();
  }
}

unsigned int current_x;
unsigned int newline_requested;
void screen_init(unsigned char reinit)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  G_io_touch.ts_last_x = -1;
  G_io_touch.ts_last_y = -1;

  current_x = 0;
  newline_requested = 0;
  memset(((char*)screen_charbuffer),' ',sizeof(screen_charbuffer));

  if (!reinit) {
    // reset the display and clear it
    // setup touchscreen
    /* GPIO Ports Clock Enable */
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    /* Peripheral clock enable */
    __SPI2_CLK_ENABLE();

    /**SPI2 GPIO Configuration    
    PB13     ------> SPI1_SCK
    PB15     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB14 DISP_CS */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    DISP_NCS(1);

    /* PC1 DISP_RESET (NRST) */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    DISP_NRST(0);

    hspi.Instance = SPI2;
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_9BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    // OK // hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
#ifdef LOW_SPEED_SCREEN
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
#endif // LOW_SPEED_SCREEN
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi.Init.CRCPolynomial = 10;
    hspi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
    HAL_SPI_Init(&hspi);
    /* Enable SPI peripheral */
    SPI2->CR1 |=  SPI_CR1_SPE;  

    // reset the screen
    DISP_NRST(1);  
    /* Enable the interrupt service thread/routine for INT after 50ms */
    HAL_Delay(50);


    // first, force a hardware reset

    // assert delay before starting requests
    // 1000 = 6ms @ msirange 5
    // 166 = 1ms @ msirange 5
    // 333 = 1ms @ msirange 6 @4mhz
    // 20000 = 120ms @ msirange 5
    //for (volatile int i = 0; i < 1000; i++);

    DISP_NCS(0);
    lcd_data(FLAG_COMMAND|FLAG_FLUSH, HX8357_EXIT_SLEEP_MODE); //Sleep Out
    HAL_Delay(150);

    lcd_data(FLAG_COMMAND, HX8357_ENABLE_EXTENSION_COMMAND);
    lcd_data(FLAG_PARAM, 0xFF);
    lcd_data(FLAG_PARAM, 0x83);
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x57);
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, HX8357_SET_POWER_CONTROL);
    lcd_data(FLAG_PARAM, 0x00);
    lcd_data(FLAG_PARAM, 0x12);
    lcd_data(FLAG_PARAM, 0x12);
    lcd_data(FLAG_PARAM, 0x12);
    lcd_data(FLAG_PARAM, 0xC3);
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x44);
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, HX8357_SET_DISPLAY_MODE);
    lcd_data(FLAG_PARAM, 0x02);
    lcd_data(FLAG_PARAM, 0x40);
    lcd_data(FLAG_PARAM, 0x00);
    lcd_data(FLAG_PARAM, 0x2A);
    lcd_data(FLAG_PARAM, 0x2A);
    lcd_data(FLAG_PARAM, 0x20);
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x91);
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, HX8357_SET_VCOM_VOLTAGE);
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x38);
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, HX8357_SET_INTERNAL_OSCILLATOR);
    lcd_data(FLAG_PARAM, 0x68);
    lcd_data(FLAG_COMMAND, 0xE3); //Unknown Command
    lcd_data(FLAG_PARAM, 0x2F);
    lcd_data(FLAG_PARAM, 0x1F);
    lcd_data(FLAG_COMMAND, HX8357_SET_BGP); //Set BGP
    lcd_data(FLAG_PARAM, 0x01);
    lcd_data(FLAG_PARAM, 0x01);
    lcd_data(FLAG_PARAM, 0x67);
    lcd_data(FLAG_COMMAND, HX8357_SET_PANEL_DRIVING);
    lcd_data(FLAG_PARAM, 0x70);
    lcd_data(FLAG_PARAM, 0x70);
    lcd_data(FLAG_PARAM, 0x01);
    lcd_data(FLAG_PARAM, 0x3C);
    lcd_data(FLAG_PARAM, 0xC8);
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x08);
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, 0xC2); // Set Gate EQ
    lcd_data(FLAG_PARAM, 0x00);
    lcd_data(FLAG_PARAM, 0x08);
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x04);
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, HX8357_SET_PANEL_CHARACTERISTIC);
  #ifdef HAVE_PT035HV_ROTATION_0  
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x09); // proto screen direction
  #endif
  #ifdef HAVE_PT035HV_ROTATION_180
    lcd_data(FLAG_PARAM|FLAG_FLUSH, 0x05); // final screen direction (usb reversed)
  #endif 
    HAL_Delay(1);
    lcd_data(FLAG_COMMAND, HX8357_SET_GAMMA_CURVE);
    lcd_data(FLAG_PARAM, 0x01);
    lcd_data(FLAG_PARAM, 0x02);
    lcd_data(FLAG_PARAM, 0x03);
    lcd_data(FLAG_PARAM, 0x05);
    lcd_data(FLAG_PARAM, 0x0E);
    lcd_data(FLAG_PARAM, 0x22);
    lcd_data(FLAG_PARAM, 0x32);
    lcd_data(FLAG_PARAM, 0x3B);
    lcd_data(FLAG_PARAM, 0x5C);
    lcd_data(FLAG_PARAM, 0x54);
    lcd_data(FLAG_PARAM, 0x4C);
    lcd_data(FLAG_PARAM, 0x41);
    lcd_data(FLAG_PARAM, 0x3D);
    lcd_data(FLAG_PARAM, 0x37);
    lcd_data(FLAG_PARAM, 0x31);
    lcd_data(FLAG_PARAM, 0x21);
    lcd_data(FLAG_PARAM, 0x01);
    lcd_data(FLAG_PARAM, 0x02);
    lcd_data(FLAG_PARAM, 0x03);
    lcd_data(FLAG_PARAM, 0x05);
    lcd_data(FLAG_PARAM, 0x0E);
    lcd_data(FLAG_PARAM, 0x22);
    lcd_data(FLAG_PARAM, 0x32);
    lcd_data(FLAG_PARAM, 0x3B);
    lcd_data(FLAG_PARAM, 0x5C);
    lcd_data(FLAG_PARAM, 0x54);
    lcd_data(FLAG_PARAM, 0x4C);
    lcd_data(FLAG_PARAM, 0x41);
    lcd_data(FLAG_PARAM, 0x3D);
    lcd_data(FLAG_PARAM, 0x37);
    lcd_data(FLAG_PARAM, 0x31);
    lcd_data(FLAG_PARAM, 0x21);
    lcd_data(FLAG_PARAM, 0x00);
    lcd_data(FLAG_PARAM, 0x01);
    HAL_Delay(1);
    
    lcd_data(FLAG_COMMAND, HX8357_SET_PIXEL_FORMAT); //COLMOD RGB565
    lcd_data(FLAG_PARAM, 0x55);
    
    lcd_data(FLAG_COMMAND, HX8357_SET_ADDRESS_MODE);
    lcd_data(FLAG_PARAM, 0x00);
    /*
    lcd_data(FLAG_COMMAND, HX8357_SET_TEAR_ON); //TE ON
    lcd_data(FLAG_PARAM, 0x00);
    */

    lcd_data(FLAG_COMMAND, HX8357_SET_INTERNAL_OSCILLATOR);
    //lcd_data(FLAG_PARAM, 0x66); // 60Hz
    lcd_data(FLAG_PARAM, 0x99); // 75Hz
    //lcd_data(FLAG_PARAM, 0xCC); // 90Hz
    lcd_data(FLAG_PARAM, 0x01); // OSC_EN

    lcd_data(FLAG_COMMAND, HX8357_ALLON);
    lcd_data(FLAG_COMMAND|FLAG_FLUSH, HX8357_SET_DISPLAY_ON); //Display On
    HAL_Delay(10);

    /*
    for(;;) {
      lcd_data(FLAG_COMMAND, HX8357_ALLON);
      HAL_Delay(1000);
      lcd_data(FLAG_COMMAND, HX8357_ALLOFF);
      HAL_Delay(1000);
    }
    */

    bagl_draw_bg(0xF9F9F9);
    // until screen_on is called
    lcd_data(FLAG_COMMAND|FLAG_FLUSH, HX8357_ALLOFF);
  }

  /* PC10 TS_CE */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PC11 TS_IRQ */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**I2C1 GPIO Configuration    
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  __I2C1_CLK_ENABLE();

  hi2c.Instance = I2C1;
  hi2c.Init.Timing = 0x10909CEC; // 100khz @ 80mhz
  hi2c.Init.Timing = 0x00702991; // 400khz @ 80mhz
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c);
  HAL_I2CEx_AnalogFilter_Config(&hi2c, I2C_ANALOGFILTER_ENABLED);

  // reset Touch screen
  TS_CE(0);
  HAL_Delay(200);
  TS_CE(1);
  HAL_Delay(10);
  TS_CE(0);
  HAL_Delay(10);
  TS_CE(1);

  // consume touch interrupt
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void screen_on(void) {
  lcd_data(FLAG_COMMAND|FLAG_FLUSH, HX8357_NORON);
}

unsigned int debug_last_percentage;
void screen_brightness(uint8_t percentage) {
  // set the light level
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_HandleTypeDef htim2;

  debug_last_percentage = percentage;

  memset(&htim2, 0, sizeof(htim2));
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  /*
  memset(&sMasterConfig, 0, sizeof(sMasterConfig));
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
  */

  memset(&sConfigOC, 0, sizeof(sConfigOC));
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250000/100*percentage;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  // start the timer
  HAL_TIM_Base_Start(&htim2);
  // start the pwm channel
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  PRINTF("freq:%d duty:%d\n", htim2.Init.Period, sConfigOC.Pulse);
}

// to draw font faster as the pixel loop is at the lower layer (if not directly within transport)
#define MAX_BITMAP_COLORS 16
unsigned char colors565_4bits[MAX_BITMAP_COLORS*2];
void bagl_hal_draw_bitmap_within_rect(unsigned short x, unsigned short y, unsigned short width, unsigned short height, unsigned int color_count, unsigned int *colors, unsigned int bit_per_pixel, unsigned char* bitmap, unsigned int bitmap_length_bits) {
  unsigned int i;  
  unsigned int pixel_mask = (1<<bit_per_pixel)-1;

  // TODO take care of bitmap with bit per pixel > 8 !!

  // compute  565 colors for the 4 first colors (subject to overflow here :p, but avoid too much parameters in this function)

  // RGB 888 to RGB 565
  for (i = 0; i< color_count && i < MAX_BITMAP_COLORS; i++) {
    colors565_4bits[i*2] = ((colors[i]>>16)&0xF8) | ((colors[i]>>(8+5))&0x07);
    colors565_4bits[i*2+1] = ((colors[i]>>5)&0xE0) | ((colors[i]>>3)&0x1F);  
  }
  
  // horizontal scan
  screen_set_xy(x, y, x+width-1, y+height-1);
  lcd_data(FLAG_COMMAND, HX8357_WRITE_MEMORY_START);

  // horizontal scan
  screen_set_xy(x, y, x+width-1, y+height-1);
  lcd_data(FLAG_COMMAND, HX8357_WRITE_MEMORY_START);


  while(bitmap_length_bits) {
    // horizontal scan
    unsigned int ch = *bitmap++;
    // draw each pixel (at most 256 index color bitmap support)
    for (i = 0; i < 8 && bitmap_length_bits; bitmap_length_bits -= bit_per_pixel, i += bit_per_pixel) {

      // grab LSB to MSB bits
      // compute the pixel color bit values to match the color in the color index
      unsigned int pixel_color_index = ((ch>>i) & pixel_mask);

      // write color index values corresponding to the given pixel color index value
      lcd_data(FLAG_PARAM, colors565_4bits[pixel_color_index*2]);
      lcd_data(FLAG_PARAM, colors565_4bits[pixel_color_index*2+1]);
    }
  }
}

void bagl_hal_draw_bitmap_continue(unsigned int bit_per_pixel, unsigned char* bitmap, unsigned int bitmap_length_bits) {
  unsigned int pixel_mask = (1<<bit_per_pixel)-1;

  while(bitmap_length_bits) {
    // horizontal scan
    unsigned int ch = *bitmap++;
    // draw each pixel (at most 256 index color bitmap support)
    unsigned int i;
    for (i = 0; i < 8 && bitmap_length_bits; bitmap_length_bits -= bit_per_pixel, i += bit_per_pixel) {

      // compute the pixel color bit values to match the color in the color index
      unsigned int pixel_color_index = ((ch>>i) & pixel_mask);

      // write color index values corresponding to the given pixel color index value
      lcd_data(FLAG_PARAM, colors565_4bits[pixel_color_index*2]);
      lcd_data(FLAG_PARAM, colors565_4bits[pixel_color_index*2+1]);
    }
  }  
}

// draw a simple rect
void bagl_hal_draw_rect(unsigned int color, unsigned short x, unsigned short y, unsigned short width, unsigned short height) {
  unsigned int i;

  // RGB 888 to RGB 565
  unsigned int color11 = ((color>>16)&0xF8) | ((color>>(8+5))&0x07);
  unsigned int color12 = ((color>>5)&0xE0) | ((color>>3)&0x1F);

  screen_set_xy(x, y, x+width-1, y+height-1);
  lcd_data(FLAG_COMMAND, HX8357_WRITE_MEMORY_START);

  screen_set_xy(x, y, x+width-1, y+height-1);
  lcd_data(FLAG_COMMAND, HX8357_WRITE_MEMORY_START);

  i = width*height;
  while(i--) {
    lcd_data(FLAG_PARAM, color11);
    lcd_data(FLAG_PARAM, color12);
  }
}

/*-------------
refresh the screen, need to be called by application every time the screen's content changes
this method is screen dependent, as it depends on the technology and transport.
--------------*/
void screen_update(void)
{
  /*
  // useless as all chars are redrawn // bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
  bagl_draw_string(PRINTF_FONT_ID, 
                 0x000000,
                 0xF9F9F9,
                 0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                 screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                 BAGL_ENCODING_LATIN1);
  */
}


void screen_printc(const char ch) {

  if (newline_requested) {
    newline_requested=0;
    current_x=0;
    // rotate n-1 line
    memmove(screen_charbuffer, ((char*)screen_charbuffer)+PRINTF_LINE_CHAR_LENGTH, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH);
    // erase last line
    memset(((char*)screen_charbuffer)+PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH,' ', PRINTF_LINE_CHAR_LENGTH);

    //bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
    // redraw the whole zone (all chars are printed, no problem with)
    /*
    bagl_draw_string(PRINTF_FONT_ID, 
                   0x000000,
                   0xF9F9F9,
                   0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                   screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                   BAGL_ENCODING_LATIN1);
    */
  }

  if(ch==0xA){
    // don't move screen yet, upon next print yes
    newline_requested = 1;

    // refresh surface
    // useless // bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
    bagl_draw_string(PRINTF_FONT_ID, 
                   0x000000,
                   0xF9F9F9,
                   0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                   screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                   BAGL_ENCODING_LATIN1);
  }
  else if(ch=='\r') {
    // erase last line (but don't move text around)
    memset(((char*)screen_charbuffer)+PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH,' ', PRINTF_LINE_CHAR_LENGTH);
  }
  else if (ch >= 0x20 && ch <= 0x7F) {
    // always write at the bottom of the screen
    screen_charbuffer[PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH+current_x] = ch;
    current_x++;
    if (current_x >= PRINTF_LINE_CHAR_LENGTH-1) {
      newline_requested = 1;
    }
  }

  /*
  // rotate n-1 line
  memmove(screen_charbuffer, ((char*)screen_charbuffer)+1, sizeof(screen_charbuffer)-1);
  // erase last line
  screen_charbuffer[sizeof(screen_charbuffer)-1] = ch;
  if (ch == '\n') {
    bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
    bagl_draw_string(PRINTF_FONT_ID, 
                   0x000000,
                   0xF9F9F9,
                   0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                   screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                   BAGL_ENCODING_LATIN1);
  }
  */
}

void screen_prints(const char* str, unsigned int charcount) {
  while(charcount--) {
    screen_printc(*str++);
  }
}


/**
 * Common printf code, relies on 2 FAL:
 * - screen_prints
 * - screen_printc
 */

static const char * const g_pcHex = "0123456789abcdef";
static const char * const g_pcHex_cap = "0123456789ABCDEF";

void screen_printf(const char* format, ...) {

    /* dummy version
    unsigned short i;
    unsigned short len = strlen(str);
    for(i=0;i<len;i++){

        screen_printc(str[i]);
    }
    */

    unsigned long ulIdx, ulValue, ulPos, ulCount, ulBase, ulNeg, ulStrlen, ulCap;
    char *pcStr, pcBuf[16], cFill;
    va_list vaArgP;
    char cStrlenSet;

    //
    // Check the arguments.
    //
    if(format == 0) {
      return;
    }

    //
    // Start the varargs processing.
    //
    va_start(vaArgP, format);

    //
    // Loop while there are more characters in the string.
    //
    while(*format)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ulIdx = 0; (format[ulIdx] != '%') && (format[ulIdx] != '\0');
            ulIdx++)
        {
        }

        //
        // Write this portion of the string.
        //
        screen_prints(format, ulIdx);

        //
        // Skip the portion of the string that was written.
        //
        format += ulIdx;

        //
        // See if the next character is a %.
        //
        if(*format == '%')
        {
            //
            // Skip the %.
            //
            format++;

            //
            // Set the digit count to zero, and the fill character to space
            // (i.e. to the defaults).
            //
            ulCount = 0;
            cFill = ' ';
            ulStrlen = 0;
            cStrlenSet = 0;
            ulCap = 0;
            ulBase = 10;

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*format++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((format[-1] == '0') && (ulCount == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ulCount *= 10;
                    ulCount += format[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Print out the character.
                    //
                    screen_prints((char *)&ulValue, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d command.
                //
                case 'd':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((long)ulValue < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ulValue = -(long)ulValue;

                        //
                        // Indicate that the value is negative.
                        //
                        ulNeg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ulNeg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ulBase = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle ths %.*s command
                // special %.*H or %.*h format to print a given length of hex digits (case: H UPPER, h lower)
                //
                case '.':
                {
                  // ensure next char is '*' and next one is 's'
                  if (format[0] == '*' && (format[1] == 's' || format[1] == 'H' || format[1] == 'h')) {
                    
                    // skip '*' char
                    format++;
                    
                    ulStrlen = va_arg(vaArgP, unsigned long);
                    cStrlenSet = 1;
                    
                    // interpret next char (H/h/s)
                    goto again;
                  }
                  
                  // does not support %.2x for example
                  goto error;
                }
                
                case '*':
                {
                  if (*format == 's' ) {                    
                    
                    ulStrlen = va_arg(vaArgP, unsigned long);
                    cStrlenSet = 2;
                    goto again;
                  }
                  
                  goto error;
                }
                
                case '-': // -XXs
                {
                  cStrlenSet = 0;
                  // read a number of space to post pad with ' ' the string to display
                  goto again;
                }

                //
                // Handle the %s command.
                // %H and %h also
                case 'H':
                  ulCap = 1; // uppercase base 16
                  ulBase = 16;
                  goto case_s;
                case 'h':
                  ulBase = 16; // lowercase base 16
                  goto case_s;
                case 's':
                case_s:
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string. (if not specified using .*)
                    //
                    switch(cStrlenSet) {
                      // compute length with strlen
                      case 0:
                        for(ulIdx = 0; pcStr[ulIdx] != '\0'; ulIdx++)
                        {
                        }
                        break;
                        
                      // use given length
                      case 1:
                        ulIdx = ulStrlen;
                        break;
                        
                      // printout prepad
                      case 2:
                        // if string is empty, then, ' ' padding
                        if (pcStr[0] == '\0') {
                        
                          // padd ulStrlen white space
                          do {
                            screen_prints(" ", 1);
                          } while(ulStrlen-- > 0);
                        
                          goto s_pad;
                        }
                        goto error; // unsupported if replicating the same string multiple times
                      case 3:
                        // skip '-' still buggy ...
                        goto again;
                    }

                    //
                    // Write the string.
                    //
                    switch(ulBase) {
                      default:
                        screen_prints(pcStr, ulIdx);
                        break;
                      case 16: {
                        unsigned char nibble1, nibble2;
                        for (ulCount = 0; ulCount < ulIdx; ulCount++) {
                          nibble1 = (pcStr[ulCount]>>4)&0xF;
                          nibble2 = pcStr[ulCount]&0xF;
                          switch(ulCap) {
                            case 0:
                              screen_printc(g_pcHex[nibble1]);
                              screen_printc(g_pcHex[nibble2]);
                              break;
                            case 1:
                              screen_printc(g_pcHex_cap[nibble1]);
                              screen_printc(g_pcHex_cap[nibble2]);
                              break;
                          }
                        }
                        break;
                      }
                    }

s_pad:
                    //
                    // Write any required padding spaces
                    //
                    if(ulCount > ulIdx)
                    {
                        ulCount -= ulIdx;
                        while(ulCount--)
                        {
                            screen_prints(" ", 1);
                        }
                    }
                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // Set the base to 10.
                    //
                    ulBase = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ulNeg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; i.e. %X will use lower case letters for a-f
                // instead of the upper case letters is should use.  We also
                // alias %p to %x.
                //
                case 'X':
                    ulCap = 1;
                case 'x':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // Set the base to 16.
                    //
                    ulBase = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ulNeg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ulIdx = 1;
                        (((ulIdx * ulBase) <= ulValue) &&
                         (((ulIdx * ulBase) / ulBase) == ulIdx));
                        ulIdx *= ulBase, ulCount--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ulNeg)
                    {
                        ulCount--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ulNeg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ulPos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ulNeg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ulCount > 1) && (ulCount < 16))
                    {
                        for(ulCount--; ulCount; ulCount--)
                        {
                            pcBuf[ulPos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ulNeg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ulPos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ulIdx; ulIdx /= ulBase)
                    {
                        if (!ulCap) {
                          pcBuf[ulPos++] = g_pcHex[(ulValue / ulIdx) % ulBase];
                        }
                        else {
                          pcBuf[ulPos++] = g_pcHex_cap[(ulValue / ulIdx) % ulBase];
                        }
                    }

                    //
                    // Write the string.
                    //
                    screen_prints(pcBuf, ulPos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                    screen_prints(format - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

error:
                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                    screen_prints("ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }

    //
    // End the varargs processing.
    //
    va_end(vaArgP);
}



void screen_test(void) {
  // initialize screen session and charge pump
  screen_init(0);
  
  /*
  screen_printf("Test\nString\n value ");
  screen_printf("azertyuiopqsdfghjklmwxcvbn,;:!?./§ù*%%µ12345678901234567890)=°+~#{[|`\\^$@]}ABCDEFGHIJKLMNOPQRSTUVWXYZ\n%04d\nBOOT SCREEN TEST\n",2763);
  
  screen_printf("%.*H\n", 8, g_pcHex);
  */
  // force refresh and close screen session
  screen_printf("Ledger BLE/NFC $itcoin Wallet\n");
  screen_update();
  
  /*
  screen_printf("lastline\n");
  
  screen_update();
  */
}

#endif // HAVE_PT035HV

