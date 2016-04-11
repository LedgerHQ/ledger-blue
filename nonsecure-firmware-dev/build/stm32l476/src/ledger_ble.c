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

#ifdef HAVE_BLE

 // not used for now
#define THROW(x) return;//for(;;);


#include "osal.h"
#include "sample_service.h"
#include "hci.h"
#include "hal.h"
#include "bluenrg_interface.h"

#include "seproxyhal.h"

/** @addtogroup BlueNRG_Shield
 *  @{
 */

#define BLE_TIMEOUT_CONNECTION_MS 5000UL
// defined by the characteristic length, use minimal to be wide usable
#define BLE_CHUNK_LENGTH_B 20


/** @defgroup BlueNRG_Shield_Sample_Application
 *  @brief Sample application for BlueNR // not used for now
#define THROW(x) for(;;);G Shield on STM32 Nucleo boards.
 *  @{
 */

/* Private defines ------------------------------------------------------------*/
/* Private macros -------------------------------------------------------------*/
#define BDADDR_SIZE 6

/* Private variables ---------------------------------------------------------*/


#if 0
unsigned short G_io_se_apdu_transport_offset;
unsigned short G_io_se_apdu_transport_remlen;
#endif


/**
 * The server-peripheral mac : leet(Wallet-000002)
 * Little endian encoding
 */
uint8_t SERVER_BDADDR[] = {0x02, 0x00, 0x00, 0xe7, 0x11, 0x3A};
uint8_t DEFAULT_NAME[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 'L','e','d','g', 'e', 'r', '(', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', ')', '\0'};



void UI_keyboard_press(unsigned char key);

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

uint8_t HCI_require_processing(void) {
    Disable_SPI_IRQ();
    uint8_t process_required = !list_is_empty(&hciReadPktRxQueue);
    Enable_SPI_IRQ();
    return process_required;
}

void BLE_diversify_name_address() {
  const char hexdigits[] = "0123456789ABCDEF";
  uint32_t uid, uid1, uid2, uid3;
  const char* UID = (char*)0x1FFF7590;
  memcpy(&uid1, UID, 4);
  memcpy(&uid2, UID + 4, 4);
  memcpy(&uid3, UID + 8, 4);
  uid = uid1 ^ uid2 ^ uid3;
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 10] = hexdigits[((uid >> 24) & 0xff) >> 4];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 9] = hexdigits[((uid >> 24) & 0xff) & 0x0f];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 8] = hexdigits[((uid >> 16) & 0xff) >> 4];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 7] = hexdigits[((uid >> 16) & 0xff) & 0x0f];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 6] = hexdigits[((uid >> 8) & 0xff) >> 4];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 5] = hexdigits[((uid >> 8) & 0xff) & 0x0f];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 4] = hexdigits[(uid & 0xff) >> 4];
  DEFAULT_NAME[sizeof(DEFAULT_NAME) - 3] = hexdigits[uid & 0x0f];
  memcpy(&SERVER_BDADDR[0], &uid, 4);
}

void BLE_make_discoverable(const char* discovered_name) {
  unsigned char ret ;
  
  // make discoverable
  // TODO limit the time it is done, to go back to sleep in case timeout
  
  if (discovered_name == NULL) {
    discovered_name = DEFAULT_NAME;
  }

  G_io_ble.last_discovered_name = discovered_name;
  
  /* disable scan response */ // good for low power // need to be reactivated for reconnection later but btle chip will get resetted
  ret = hci_le_set_scan_resp_data(0, NULL);
  if (ret) {
    THROW(EXCEPTION);
  }

  // set device name
  const char *name = "Ledger";  
  ret = aci_gatt_update_char_value(G_io_ble.gap_service_handle, G_io_ble.gap_dev_name_char_handle, 0, strlen(name), (uint8_t *)name);        
  if(ret){
    PRINTF("aci_gatt_update_char_value device name failed. %d\n", ret);
  }  

  // set appearance value (generic keyring)
  unsigned short appearance = 576;   
  ret = aci_gatt_update_char_value(G_io_ble.gap_service_handle, G_io_ble.gap_appearance_char_handle, 0, 2, (uint8_t *)&appearance);        
  if(ret){
    PRINTF("aci_gatt_update_char_value appearance failed. %d\n", ret);
  }  

  // remove manufacturer advertisement data
  ret = aci_gap_delete_ad_type(0xFF);
  if (ret) {
    PRINTF("aci_gap_delete_ad_type manufacturer failed. %d\n", ret);
  }

  /*
  #warning TODO
  // TODO use advert data to advertise the product ID instead of changing the Name
  ret = aci_gap_update_adv_data(0, NULL);
  if (ret) {
    THROW(EXCEPTION);
  }
  */

  //PRINTF("General Discoverable Mode ");
  /*
  Advertising_Event_Type, Adv_Interval_Min, Adv_Interval_Max, Address_Type, Adv_Filter_Policy,
  Local_Name_Length, Local_Name, Service_Uuid_Length, Service_Uuid_List, Slave_Conn_Interval_Min,
  Slave_Conn_Interval_Max
  */
  ret = aci_gap_set_discoverable(ADV_IND, 
                                  // low power leverage
                                  500 * 625/1000, 3000 * 625 / 1000, 
                                  PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                  strlen(discovered_name), discovered_name, 
#warning TODO, add service list as nicolas.bigot requested
                                  0, NULL, 
                                  // low power leverage
                                  6*1250/1000, 50*1250/1000); // (50) bigger than this it's far too long
  if (ret) {
    PRINTF("set discoverable failed.\n");
    THROW(EXCEPTION);
  }
  
  // TODO set a timeout to stop advertising (could be power off btw)
  
  //PRINTF("%d\n",ret);  
}


unsigned int last_state;
void BLE_power(unsigned char powered, const char* discovered_name) {
  GPIO_InitTypeDef GPIO_InitStruct;
  tBleStatus ret;

  if (last_state == powered) {
    return;
  }
  last_state = powered;

  BLE_diversify_name_address();
  
  // reset ble state
  memset(&G_io_ble, 0, sizeof(G_io_ble));

  /* Enable GPIO Ports Clock */  
  BNRG_SPI_RESET_CLK_ENABLE();
  BNRG_SPI_SCLK_CLK_ENABLE();
  BNRG_SPI_MISO_CLK_ENABLE();
  BNRG_SPI_MOSI_CLK_ENABLE();
  BNRG_SPI_CS_CLK_ENABLE();
  BNRG_SPI_IRQ_CLK_ENABLE();
  
  if (powered) {
    // wait a bit
    //HAL_Delay(1000);
    
    /* Initialize the BlueNRG HCI */
    HCI_Init();
    
    /* Enable SPI clock */
    BNRG_SPI_CLK_ENABLE();
    
    /* Reset */
    GPIO_InitStruct.Pin = BNRG_SPI_RESET_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_RESET_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_RESET_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_RESET_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_RESET_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);    
    BNRG_SPI_RESET_PORT->ODR &= ~BNRG_SPI_RESET_PIN;
    
    /* SCLK */
    GPIO_InitStruct.Pin = BNRG_SPI_SCLK_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_SCLK_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_SCLK_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_SCLK_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_SCLK_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_SCLK_PORT, &GPIO_InitStruct); 
    
    /* MISO */
    GPIO_InitStruct.Pin = BNRG_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_MISO_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_MISO_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_MISO_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MISO_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);
    
    /* MOSI */
    GPIO_InitStruct.Pin = BNRG_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_MOSI_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_MOSI_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_MOSI_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MOSI_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MOSI_PORT, &GPIO_InitStruct);
    
    /* NSS/CSN/CS */
    GPIO_InitStruct.Pin = BNRG_SPI_CS_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_CS_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_CS_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_CS_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_CS_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);

    /* IRQ -- OUTPUT => ! DFU MODE */
    GPIO_InitStruct.Pin = BNRG_SPI_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = BNRG_SPI_IRQ_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_IRQ_SPEED;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStruct);
    // make sure to let the bluenrg not in DFU mode
    HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_RESET);
    
    /* Initialize the BlueNRG SPI driver */
    BNRG_SPI_Init();

    /* Reset BlueNRG hardware */
    BlueNRG_RST();

    /* MISO */
    /*
    GPIO_InitStruct.Pin = BNRG_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_MISO_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_MISO_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_MISO_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MISO_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);
    */

    /* IRQ -- INPUT */
    GPIO_InitStruct.Pin = BNRG_SPI_IRQ_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_IRQ_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_IRQ_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_IRQ_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_IRQ_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStruct);

    /* Configure the NVIC for SPI */  
    // clear up interrupts
    NVIC_ClearPendingIRQ(BNRG_SPI_EXTI_IRQn);
    HAL_NVIC_SetPriority(BNRG_SPI_EXTI_IRQn, 4, 0);    
    HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);

    // at least 3ms, and the irq must show up before sending the first command
    HAL_Delay(5);

    // force a read
    //while(HCI_Isr_read_packet() == 0);
    
    // probably not a good idea to do this each at each startup
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                    CONFIG_DATA_PUBADDR_LEN,
                                    SERVER_BDADDR
                                  );
    if(ret){
      PRINTF("Setting BD_ADDR failed.\n");
      THROW(EXCEPTION);
    }
    
    ret = aci_gatt_init();    
    if(ret){
      PRINTF("GATT_Init failed.\n");
      THROW(EXCEPTION);
    }
    
    #ifdef BLUENRG_MS
    ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 7, &G_io_ble.gap_service_handle, &G_io_ble.gap_dev_name_char_handle, &G_io_ble.gap_appearance_char_handle);
    #else
    ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &G_io_ble.gap_service_handle, &G_io_ble.gap_dev_name_char_handle, &G_io_ble.gap_appearance_char_handle);
    #endif 
    
    if(ret != BLE_STATUS_SUCCESS){
      PRINTF("GAP_Init failed.\n");
      THROW(EXCEPTION);
    }
      
    // limit exchanges and data security to limit power, moreover apdus are public (f)
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_NOT_REQUIRED,
                                      OOB_AUTH_DATA_ABSENT,
                                      NULL,
                                      7,
                                      16,
                                      USE_FIXED_PIN_FOR_PAIRING,
                                      1337,
                                      NO_BONDING);
    if (ret != BLE_STATUS_SUCCESS) {
      PRINTF("BLE Stack Initialized.\n");
      THROW(EXCEPTION);
    }
    
    //PRINTF("SERVER: BLE Stack Initialized\n");
    ret = Add_Sample_Service();
    
    if(ret != BLE_STATUS_SUCCESS) {
      PRINTF("Error while adding service.\n");
      THROW(EXCEPTION);
    }
    
    /* Set output power level */
    ret = aci_hal_set_tx_power_level(1,7); // max power
    //ret = aci_hal_set_tx_power_level(0, 0); // lowest power
    if (ret) {
      THROW(EXCEPTION);
    }

    G_io_ble.apdu_available = 0;
  
    BLE_make_discoverable(discovered_name);
  }
  else {
    // disable the interrupt when performing 
    HAL_NVIC_DisableIRQ(BNRG_SPI_EXTI_IRQn);
    
    BNRG_SPI_CLK_DISABLE();
    
    /* Reset */
    GPIO_InitStruct.Pin = BNRG_SPI_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; // consume 4.5mA more, 0.95mA => 5.3mA,  NOT ACCEPTABLE
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(BNRG_SPI_RESET_PORT, &GPIO_InitStruct);       
    BNRG_SPI_RESET_PORT->ODR &= ~BNRG_SPI_RESET_PIN;
    
    /* SCLK */
    GPIO_InitStruct.Pin = BNRG_SPI_SCLK_PIN;
    HAL_GPIO_Init(BNRG_SPI_SCLK_PORT, &GPIO_InitStruct); 
    HAL_GPIO_WritePin(BNRG_SPI_SCLK_PORT, BNRG_SPI_SCLK_PIN, GPIO_PIN_RESET);
    
    /* MOSI */
    GPIO_InitStruct.Pin = BNRG_SPI_MOSI_PIN;
    HAL_GPIO_Init(BNRG_SPI_MOSI_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BNRG_SPI_MOSI_PORT, BNRG_SPI_MOSI_PIN, GPIO_PIN_RESET);
    
    /* NSS/CSN/CS */
    GPIO_InitStruct.Pin = BNRG_SPI_CS_PIN;
    HAL_GPIO_Init(BNRG_SPI_CS_PORT, &GPIO_InitStruct);
    //HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET); // Consume 35mA more, driving current through the bluenrg chip for nothing.
    HAL_GPIO_WritePin(BNRG_SPI_SCLK_PORT, BNRG_SPI_SCLK_PIN, GPIO_PIN_RESET);
    
    /* IRQ -- INPUT */
    GPIO_InitStruct.Pin = BNRG_SPI_IRQ_PIN;
    //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_RESET);
    
    /* MISO */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = BNRG_SPI_MISO_PIN;
    HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BNRG_SPI_MISO_PORT, BNRG_SPI_MISO_PIN, GPIO_PIN_RESET);
  }
}

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
        uuid_struct.uuid128[0] = uuid_0; uuid_struct.uuid128[1] = uuid_1; uuid_struct.uuid128[2] = uuid_2; uuid_struct.uuid128[3] = uuid_3; \
        uuid_struct.uuid128[4] = uuid_4; uuid_struct.uuid128[5] = uuid_5; uuid_struct.uuid128[6] = uuid_6; uuid_struct.uuid128[7] = uuid_7; \
        uuid_struct.uuid128[8] = uuid_8; uuid_struct.uuid128[9] = uuid_9; uuid_struct.uuid128[10] = uuid_10; uuid_struct.uuid128[11] = uuid_11; \
        uuid_struct.uuid128[12] = uuid_12; uuid_struct.uuid128[13] = uuid_13; uuid_struct.uuid128[14] = uuid_14; uuid_struct.uuid128[15] = uuid_15; \
        }while(0)

/**
 * @brief  Add a sample service using a vendor specific profile
 * @param  None
 * @retval Status
 */
const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};

tBleStatus Add_Sample_Service(void)
{
  tBleStatus ret;
  
  /*
  UUIDs:
  D973F2E0-B19E-11E2-9E96-0800200C9A66
  D973F2E1-B19E-11E2-9E96-0800200C9A66
  D973F2E2-B19E-11E2-9E96-0800200C9A66
  */
  
#if 1

  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &G_io_ble.service_handle); /* original is 9?? */
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  ret =  aci_gatt_add_char(G_io_ble.service_handle, UUID_TYPE_128, charUuidTX, BLE_CHUNK_LENGTH_B, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &G_io_ble.tx_characteristic_handle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret =  aci_gatt_add_char(G_io_ble.service_handle, UUID_TYPE_128, charUuidRX, BLE_CHUNK_LENGTH_B, CHAR_PROP_WRITE/*|CHAR_PROP_WRITE_WITHOUT_RESP*/, ATTR_PERMISSION_NONE, GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &G_io_ble.rx_characteristic_handle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  //PRINTF("Sample Service added.\nTX Char Handle %04X, RX Char Handle %04X\n", G_io_ble.tx_characteristic_handle, G_io_ble.rx_characteristic_handle);
  return BLE_STATUS_SUCCESS;
#elif 0
  // ORIGINAL
  
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &G_io_ble.service_handle); /* original is 9?? */
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  ret =  aci_gatt_add_char(G_io_ble.service_handle, UUID_TYPE_128, charUuidTX, BLE_CHUNK_LENGTH_B, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &G_io_ble.tx_characteristic_handle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret =  aci_gatt_add_char(G_io_ble.service_handle, UUID_TYPE_128, charUuidRX, BLE_CHUNK_LENGTH_B, CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, ATTR_ACCESS_WRITE_REQ_ONLY,
                           16, 1, &G_io_ble.rx_characteristic_handle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  //PRINTF("Sample Service added.\nTX Char Handle %04X, RX Char Handle %04X\n", G_io_ble.tx_characteristic_handle, G_io_ble.rx_characteristic_handle);
  return BLE_STATUS_SUCCESS; 
#else 
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &G_io_ble.service_handle); /* original is 9?? */
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  ret =  aci_gatt_add_char(G_io_ble.service_handle, UUID_TYPE_128, charUuidTX, BLE_CHUNK_LENGTH_B, CHAR_PROP_INDICATE, ATTR_PERMISSION_NONE, GATT_INTIMATE_APPL_WHEN_READ_N_WAIT,
                           16, 1, &G_io_ble.tx_characteristic_handle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret =  aci_gatt_add_char(G_io_ble.service_handle, UUID_TYPE_128, charUuidRX, BLE_CHUNK_LENGTH_B, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE, GATT_SERVER_ATTR_WRITE,
                           16, 1, &G_io_ble.rx_characteristic_handle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  //PRINTF("Sample Service added.\nTX Char Handle %04X, RX Char Handle %04X\n", G_io_ble.tx_characteristic_handle, G_io_ble.rx_characteristic_handle);
  return BLE_STATUS_SUCCESS; 
#endif 
  
fail:
  PRINTF("Error while adding Sample Service.\n");
  return BLE_STATUS_ERROR ;
}

/**
 * @brief  This function is used to send data related to the sample service
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
// OTO handle = G_io_ble.tx_characteristic_handle for default apdu tranport
void BLE_send(unsigned short handle, uint8_t* data_buffer, uint8_t Nb_bytes)
{
  unsigned char ret;
retry:
  ret = aci_gatt_update_char_value(G_io_ble.service_handle, handle, 0, Nb_bytes, data_buffer);
  // likely the link could be busy
  if(ret!=BLE_STATUS_SUCCESS) {
    // TODO go low power, before retrying
    goto retry;
  }
}


void BLE_accept_previous_write(void) {
  // Accept it before processing its data to avoid deadlock in spi access within attribute modified
  aci_gatt_write_response( G_io_ble.last_write_conn_handle,
                           G_io_ble.last_write_attr_handle,
                           0, // accept the write
                           0, // no error
                           G_io_ble.last_write_size, // TODO must check it's the size of the chunk
                           G_io_ble.last_write_buffer);
}

/**
 * @brief  This function is called when an attribute gets modified
 * @param  handle : handle of the attribute
 * @param  data_length : size of the modified attribute data
 * @param  att_data : pointer to the modified attribute data
 * @retval None
 */

#define IS_BLE_APDU_AVAILABLE() (G_io_apdu_length>0 && G_io_ble.apdu_transport_remlen==0)

#define BLE_OFFSET_TAG 0
#define BLE_OFFSET_SEQH 1
#define BLE_OFFSET_SEQL 2
#define BLE_SEQ0_OFFSET_LENH 3
#define BLE_SEQ0_OFFSET_LENL 4
#define BLE_SEQ0_OFFSET_DATA 5 // TAG, SEQ2, LEN2, DATA
#define BLE_SEQN_OFFSET_DATA 3 // TAG, SEQ2, DATA

// instruction supported
#define BLE_TAG_VERSION 0
#define BLE_TAG_INIT 1
#define BLE_TAG_ECHO 2
#define BLE_TAG_APDU 5
#ifdef BLE_PACKET_ACK
#define BLE_TAG_ACK  6 // ack each packet of transmitted by the server to avoid overwritting properties before the client read it.
#endif // BLE_PACKET_ACK
#define BLE_TAG_FLASHBACK 7
#define BLE_TAG_ERROR 0xE

#define os_memmove memmove
#define os_memset memset

/**
 * Function called when a complete command apdu has been received and is ready for processing
 */
void BLE_protocol_command_apdu_CB() {
  /*
  screen_printf("apdu received\n");
  screen_update();
  */
  
  // wake the main thread to process the command apdu through the SE+screen+keyboard.
  // avoid sleep-on-exit after interrupt processing (could also be done in main instead of sleep-on-exit, by checking the event bit, and else to go sleep)
  G_io_ble.apdu_available = 1;
  // avoid BLE timeout
  G_io_ble.connection_timeout_enabled = 0;
}

/**
 * Receive and concat Command APDU chunks
 * Calls
 */
void BLE_protocol_recv(unsigned char data_length, unsigned char* att_data) {
  unsigned char buf[5];
  unsigned short l = data_length;
  
  // not processable, must be resent by the client (a chunk is being output btw)
  if (G_io_ble.apdu_transport_lock) {
    return;
  }

  // process the chunk content
  switch(att_data[BLE_OFFSET_TAG]) {
  case BLE_TAG_APDU:
    // won't process if busy sending (out of it)
    if (G_io_ble.apdu_transport_busy_sending) {
      return;
    }
    
    if (att_data[BLE_OFFSET_SEQH] != (G_io_ble.apdu_transport_seq>>8) 
      || att_data[BLE_OFFSET_SEQL] != (G_io_ble.apdu_transport_seq&0xFF)) {
      // ignore packet
      att_data[0] = 0;
      goto error;
    }
    
    // tag, seq
    l -= 1+2;
    
    // append the received chunk to the current command apdu
    if (G_io_ble.apdu_transport_seq == 0) {
      /// This is the apdu first chunk
      // total apdu size to receive
      G_io_ble.apdu_transport_remlen = (att_data[BLE_SEQ0_OFFSET_LENH]<<8)+(att_data[BLE_SEQ0_OFFSET_LENL]&0xFF);
      G_io_ble.apdu_transport_ptr = G_io_apdu_buffer;
      // check for invalid length encoding (more data in chunk that announced in the total apdu)
      // total length
      l -= 2;
      // compute remaining size to receive
      G_io_ble.apdu_length = G_io_ble.apdu_transport_remlen;
      
      if (G_io_apdu_length > sizeof(G_io_apdu_buffer)) {
        att_data[0] = 1;
        goto error;
      }
      // in case of invalid formatting
      if (l > G_io_ble.apdu_transport_remlen) {
        l = G_io_ble.apdu_transport_remlen;
      }
      // copy data
      //os_memmove(G_io_ble.apdu_transport_ptr, att_data+BLE_TAG_APDU_OFFSET_DATA_SEQ0, l);
      att_data+=BLE_SEQ0_OFFSET_DATA;
    }
    else {
      // check for invalid length encoding (more data in chunk that announced in the total apdu)
      if (l > G_io_ble.apdu_transport_remlen) {
        l = G_io_ble.apdu_transport_remlen;
      }

      /// This is a following chunk
      // append content
      //os_memmove(G_io_ble.apdu_transport_ptr, att_data+BLE_TAG_APDU_OFFSET_DATA, l);
      att_data+=BLE_SEQN_OFFSET_DATA;
    }
    // factorize (f)
    os_memmove(G_io_ble.apdu_transport_ptr, att_data, l);
    
#ifdef BLE_PACKET_ACK
    // ack the received packet
    os_memset(buf, 0, 5);
    buf[BLE_OFFSET_TAG] = BLE_TAG_ACK;
    buf[BLE_OFFSET_SEQH] = G_io_ble.apdu_transport_seq>>8;
    buf[BLE_OFFSET_SEQL] = G_io_ble.apdu_transport_seq;
    
    // ack just before procesing apdu if any (event driven)
    BLE_send(G_io_ble.tx_characteristic_handle, buf, 5);
#endif // BLE_PACKET_ACK
    
    // advance 
    G_io_ble.apdu_transport_ptr += l;
    G_io_ble.apdu_transport_remlen -= l;
    G_io_ble.apdu_transport_seq ++;
    
    // the APDU has been completely received
    if(G_io_ble.apdu_transport_remlen==0) {
      BLE_protocol_command_apdu_CB();
    }
    break;
    
#ifdef BLE_PACKET_ACK
  case BLE_TAG_ACK: 
  {
    /*
    if (att_data[BLE_OFFSET_SEQH] != (G_io_ble.apdu_transport_seq>>8) 
      || att_data[BLE_OFFSET_SEQL] != (G_io_ble.apdu_transport_seq&0xFF)) {
      // ignore packet
      att_data[0] = 2;
      goto error;
    }
    */
    if (l != 5) {
      // ignore packet
      att_data[0] = 6;
      goto error;
    }
    
    G_io_ble.apdu_transport_ack = 1;
    break;
  }
#endif // BLE_PACKET_ACK
    
  case BLE_TAG_VERSION: // get version ID 
  {
    unsigned char buf[9];
    
    if (att_data[BLE_OFFSET_SEQH] != 0 
      || att_data[BLE_OFFSET_SEQL] != 0) {
      // ignore packet
      att_data[0] = 3;
      goto error;
    }
    
    // TAG,SEQ2,LEN2
    // do not reset the current apdu reception if any
    os_memset(buf, 0, 9); // PROTOCOL VERSION is 0
    buf[BLE_SEQ0_OFFSET_LENL]=4;
    // send the response
    BLE_send(G_io_ble.tx_characteristic_handle, buf, 9);
    break;
  }
  case BLE_TAG_ECHO: // ECHO|PING
  {
    if (att_data[BLE_OFFSET_SEQH] != 0 
      || att_data[BLE_OFFSET_SEQL] != 0) {
      // ignore packet
      att_data[0] = 4;
      goto error;
    }
    
    BLE_send(G_io_ble.tx_characteristic_handle, att_data, data_length);
    break;
  }
  case BLE_TAG_INIT: // (RE)INIT communication machine state
  {
    if (att_data[BLE_OFFSET_SEQH] != 0 
      || att_data[BLE_OFFSET_SEQL] != 0) {
      // ignore packet
      att_data[0] = 5;
      goto error;
    }
    
    // reset tokenization of the capdu reception
    G_io_ble.apdu_transport_seq = 0;
    G_io_ble.apdu_transport_remlen = 0;
    G_io_ble.apdu_transport_ptr = G_io_apdu_buffer;
    G_io_ble.apdu_transport_busy_sending = 0; // make sure the BLE is not sending, if sending then exits the send loop
    G_io_ble.apdu_transport_lock = 0;
    G_io_ble.apdu_available = 0;
#ifdef BLE_PACKET_ACK
    G_io_ble.apdu_transport_ack = 0;
#endif // BLE_PACKET_ACK
    G_io_apdu_length = 0;
    BLE_send(G_io_ble.tx_characteristic_handle, att_data, data_length); // echoed
    break;
  }
  default: 
  error:
    // reset state upon error
    G_io_ble.apdu_transport_seq = 0;
    G_io_ble.apdu_transport_remlen = 0;
    G_io_ble.apdu_transport_ptr = G_io_apdu_buffer;
    G_io_ble.apdu_transport_busy_sending = 0; // make sure the BLE is not sending, if sending then exits the send loop
    G_io_ble.apdu_transport_lock = 0;
    G_io_ble.apdu_available = 0;
#ifdef BLE_PACKET_ACK
    G_io_ble.apdu_transport_ack = 0;
#endif // BLE_PACKET_ACK
    G_io_apdu_length = 0;
    
    // send error
    os_memset(buf, 0, 5);
    buf[0]=BLE_TAG_ERROR;
    buf[1] = att_data[0];
    BLE_send(G_io_ble.tx_characteristic_handle, buf, 5);
    break;
  }
}

/**
 * Split and send the Response APDU chunks
 */
#define BLE_protocol_send_RET_OK 0
#define BLE_protocol_send_RET_BUSY 1
#define BLE_protocol_send_RET_ABORTED 2
// TODO ensure not being stuck in case disconnect or timeout during reply.
// add a timeout for reply operation
char BLE_protocol_send(unsigned char* response_apdu, unsigned short response_apdu_length) {
  
  unsigned char chunk[BLE_CHUNK_LENGTH_B];
  
  // error if already sending (inception)
  if (G_io_ble.apdu_transport_busy_sending) {
    return 1;
  }
  
  G_io_ble.apdu_transport_busy_sending = 1;
  
  G_io_ble.apdu_transport_ptr = response_apdu;
  G_io_ble.apdu_transport_remlen = response_apdu_length;
  G_io_ble.apdu_transport_seq = 0;
  // mark capdu consumed
  G_io_apdu_length = 0;

  // perform send
  while(G_io_ble.apdu_transport_remlen) {
    unsigned short l;

    // keep the channel identifier
    chunk[BLE_OFFSET_TAG] = BLE_TAG_APDU;
    chunk[BLE_OFFSET_SEQH] = G_io_ble.apdu_transport_seq>>8;
    chunk[BLE_OFFSET_SEQL] = G_io_ble.apdu_transport_seq;

    if (G_io_ble.apdu_transport_seq == 0) {
      l = ((G_io_ble.apdu_transport_remlen>BLE_CHUNK_LENGTH_B-BLE_SEQ0_OFFSET_DATA) ? BLE_CHUNK_LENGTH_B-BLE_SEQ0_OFFSET_DATA : G_io_ble.apdu_transport_remlen);
      chunk[BLE_SEQ0_OFFSET_LENH] = G_io_ble.apdu_transport_remlen>>8;
      chunk[BLE_SEQ0_OFFSET_LENL] = G_io_ble.apdu_transport_remlen;
      os_memmove(chunk+BLE_SEQ0_OFFSET_DATA, G_io_ble.apdu_transport_ptr, l);
      G_io_ble.apdu_transport_ptr += l;
      G_io_ble.apdu_transport_remlen -= l;
      l += BLE_SEQ0_OFFSET_DATA;
    }
    else {
      l = ((G_io_ble.apdu_transport_remlen>BLE_CHUNK_LENGTH_B-BLE_SEQN_OFFSET_DATA) ? BLE_CHUNK_LENGTH_B-BLE_SEQN_OFFSET_DATA : G_io_ble.apdu_transport_remlen);
      os_memmove(chunk+BLE_SEQN_OFFSET_DATA, G_io_ble.apdu_transport_ptr, l);
      G_io_ble.apdu_transport_ptr += l;
      G_io_ble.apdu_transport_remlen -= l;
      l += BLE_SEQN_OFFSET_DATA;
    }
    // prepare next chunk numbering
    G_io_ble.apdu_transport_seq++;
    
    // check if isr is performing before colliding
    G_io_ble.apdu_transport_lock = 1;
    // TAG_INIT called under isr
    if (!G_io_ble.apdu_transport_busy_sending) {
      G_io_ble.apdu_transport_lock = 0;
      // aborted by previous interrupt
      return 2;
    }
    // in mutual exclusion with isr
    BLE_send(G_io_ble.tx_characteristic_handle, chunk, l);
    // release the seal (f)
    G_io_ble.apdu_transport_lock = 0;
#ifdef BLE_PACKET_ACK
    // wait until the data chunk is acked by the client
    while(!G_io_ble.apdu_transport_ack) {
      __asm volatile ("cpsid i");
      __asm volatile ("wfi");
      __asm volatile ("cpsie i");
      __asm volatile ("nop"); 
      __asm volatile ("nop"); 
    }
    // TODO low power busy wait to solve ...
    // consume the lock
    G_io_ble.apdu_transport_ack = 0;
#endif // BLE_PACKET_ACK
  }
  
  // prepare for a new command reception
  G_io_ble.apdu_transport_seq = 0;
  G_io_ble.apdu_transport_busy_sending = 0;
  
  //OK
  return 0;
}

#if 0
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if(handle == G_io_ble.rx_characteristic_handle + 1){

    // have data ready for seproxy hal transport
    G_io_ble.write_size = evt->data_length;
    memmove(G_io_ble.write_buffer, evt->data, G_io_ble.write_size);
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_WRITE;

    /*
    screen_printf("BLE packet: %.*H\n",data_length, att_data);
    screen_update();
    */
    
    #ifdef BLE_DEFAULT_APDU_TRANSPORT_PROTOCOL
    if (!G_io_ble.client_link_established) {
      // either disconnected or not yet connected, cannot proceed
      return;
    }
    // decapsulate BLE framing to reconstruct the command.
    // command set : 
    //  - reset
    //  - DFU
    //  - apdu transport (to deliver the SE, whom will afterwards drive the screen and request a pin code
    BLE_protocol_recv(data_length, att_data);
    #endif
  } else if (handle == G_io_ble.tx_characteristic_handle + 2) {        
    // tx char handle attribute is triggered, the client registered to notification reception
    if(att_data[0] == 0x01 && att_data[1] == 0) {

      G_io_ble.notification_registered = handle;
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER;

      #ifdef BLE_DEFAULT_APDU_TRANSPORT_PROTOCOL
      screen_printf("logic link ok\n");
      screen_update();
      
      G_io_ble.client_link_established = TRUE;
      // from here we can send response to the host, make sure this is done upon receiving of the first capdu block, else it's an error
      #endif 
    }
    else if(att_data[0] == 0x01 && att_data[1] == 0) {
      G_io_ble.notification_unregistered = handle;
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER;      
    }
  }
}
#endif 

void BLE_reset_connection(void) {

  G_io_ble.client_link_established = FALSE;
  G_io_ble.connection_timeout_enabled = 0;

  // TODO remove me
  //BEGIN
  BLE_power(0, NULL);
//  HAL_Delay(500);
  
  // it's faster to reset the device to avoid problems (f)
  NVIC_SystemReset();
  //END

    // seems ok since ENABLE_SPI_FIX and pendsv
  G_io_ble.client_link_established = FALSE;
  G_io_ble.connection_timeout_enabled = 0;
  BLE_make_discoverable(G_io_ble.last_discovered_name);
}


/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  addr : Address of peer device
 * @param  handle : Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{ 
  /*
  screen_printf("client connected\n");
  screen_update();
  */
  
  G_io_ble.client_link_established = FALSE;
  G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_CONNECT;
  /*
  // wait until notification registration is performed
  G_io_ble.connection_timeout_ms = BLE_TIMEOUT_CONNECTION_MS;
  G_io_ble.connection_timeout_enabled = 1;
  */
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  /*
  // TODO power off ? after a timeout !
  screen_printf("client disconnected\n");
  screen_update();
  */

  G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_DISCONNECT;

  /*
  // TODO remove me
  //BEGIN
//  HAL_Delay(500);
  
  // it's faster to reset the device to avoid problems (f)
  NVIC_SystemReset();
  //END
  
  */
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  pckt  Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;


  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
    
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
        
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data;
          if (evt->attr_handle == G_io_ble.tx_characteristic_handle + 2) {        
            // tx char handle attribute is triggered, the client registered to notification reception
            if(evt->att_data[0] == 0x01 && evt->att_data[1] == 0) {
              G_io_ble.notification_reg_handle = evt->attr_handle;
              G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER;

              if (G_io_ble_apdu_protocol_enabled) {
                /*
                screen_printf("logic link ok\n");
                screen_update();
                */
                
                G_io_ble.client_link_established = TRUE;
                // from here we can send response to the host, make sure this is done upon receiving of the first capdu block, else it's an error
              }
            }
            else if(evt->att_data[0] == 0x01 && evt->att_data[1] == 0) {
              G_io_ble.notification_unreg_handle = evt->attr_handle;
              G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER;      
            }
          }
        }
        break;

      case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
        {
          evt_gatt_write_permit_req *evt = (evt_gatt_write_permit_req*)blue_evt->data;

          if(evt->attr_handle == G_io_ble.rx_characteristic_handle + 1){

            if (G_io_ble_apdu_protocol_enabled) {
              if (!G_io_ble.client_link_established) {
                // either disconnected or not yet connected, cannot proceed
                return;
              }
              // decapsulate BLE framing to reconstruct the command.
              // command set : 
              //  - reset
              //  - DFU
              //  - apdu transport (to deliver the SE, whom will afterwards drive the screen and request a pin code
              BLE_protocol_recv(evt->data_length, evt->data);

              // accept the write request
              // Accept it before processing its data to avoid deadlock in spi access within attribute modified
              aci_gatt_write_response( evt->conn_handle,
                                       evt->attr_handle,
                                       0, // accept the write
                                       0, // no error
                                       evt->data_length, // TODO must check it's the size of the chunk
                                       evt->data);
            }
            else {
              // have data ready for seproxy hal transport
              G_io_ble.last_write_conn_handle = evt->conn_handle;
              G_io_ble.last_write_attr_handle = evt->attr_handle;
              G_io_ble.last_write_size = evt->data_length;
              memmove(G_io_ble.last_write_buffer, evt->data, G_io_ble.last_write_size);


              #if 0
#warning FOR DEV ONLY
              // perform flashback loader
              if (G_io_ble.last_write_size > 0 && G_io_ble.last_write_buffer[BLE_OFFSET_TAG] == BLE_TAG_FLASHBACK) {
                PRINTF("performing ST flashback:");
                SE_iso_flashback();
                PRINTF(" ok\n");
                // no more processing
                return;
              }
              #endif

              G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BLE_WRITE;
              // the write response ack is perforemd async after SE replied
            }
          }
        }
        break;
      }
    }
    break;
  }    
}

#endif // HAVE_BLE
