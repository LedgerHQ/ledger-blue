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

#ifndef SEPROXYHAL_H
#define SEPROXYHAL_H

#define BLE_CMD_APDU 0x05
#define BLE_CHUNK_LENGTH 20
#define M24SR_CHUNK_LENGTH 0xF6

// EVENTS
#define SEPROXYHAL_TAG_SESSION_START_EVENT         0x01
#define SEPROXYHAL_TAG_SESSION_START_EVENT_REQBLE   0x01
#define SEPROXYHAL_TAG_SESSION_START_EVENT_RECOVERY 0x02
#define SEPROXYHAL_TAG_BLE_PAIRING_ATTEMPT_EVENT   0x02
#define SEPROXYHAL_TAG_BLE_WRITE_REQUEST_EVENT     0x03
#define SEPROXYHAL_TAG_BLE_READ_REQUEST_EVENT      0x04
#define SEPROXYHAL_TAG_BUTTON_PUSH_EVENT           0x05
#define SEPROXYHAL_LONG_BUTTON_PUSH_MS               2000
#define SEPROXYHAL_TAG_NFC_FIELD_DETECTION_EVENT   0x06
#define SEPROXYHAL_TAG_NFC_APDU_RECEIVED_EVENT     0x07
#define SEPROXYHAL_TAG_BATTERY_NOTIFICATION_EVENT  0x08
#define SEPROXYHAL_TAG_M24SR_GPO_CHANGE_EVENT      0x09
#define SEPROXYHAL_TAG_M24SR_RESPONSE_APDU_EVENT   0x0A
#define SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_EVENT   0x0B
#define SEPROXYHAL_TAG_FINGER_EVENT                0x0C
#define SEPROXYHAL_TAG_FINGER_EVENT_TOUCH           0x01
#define SEPROXYHAL_TAG_FINGER_EVENT_RELEASE         0x02
#define SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT     0x0D
#define SEPROXYHAL_TAG_TICKER_EVENT                0x0E
#define SEPROXYHAL_TAG_USB_EVENT                   0x0F // <connect/disconnect/suspend/resume>
#define SEPROXYHAL_TAG_USB_EVENT_RESET              0x01
#define SEPROXYHAL_TAG_USB_EVENT_SOF                0x02
#define SEPROXYHAL_TAG_USB_EVENT_SUSPENDED          0x04
#define SEPROXYHAL_TAG_USB_EVENT_RESUMED            0x08
#define SEPROXYHAL_TAG_USB_EP_XFER_EVENT           0x10 // <epnum> <xferin/xferout/xfersetup> <datalength> <data>
#define SEPROXYHAL_TAG_USB_EP_XFER_SETUP            0x01
#define SEPROXYHAL_TAG_USB_EP_XFER_IN               0x02
#define SEPROXYHAL_TAG_USB_EP_XFER_OUT              0x04
#define SEPROXYHAL_TAG_BLE_CONNECTION_EVENT        0x11 // <connected(1)|disconnected(0)>
#define SEPROXYHAL_TAG_UNSEC_CHUNK_EVENT           0x12
#define SEPROXYHAL_TAG_ACK_LINK_SPEED              0x13 // <ack=1|nack=0 (1byte)>

// COMMANDS
#define SEPROXYHAL_TAG_GO_BOOTLOADER               0x31
#define SEPROXYHAL_TAG_UNSEC_CHUNK_READ            0x32
#define SEPROXYHAL_TAG_BLE_DEFINE_GENERIC_SETTING  0x41
#define SEPROXYHAL_TAG_BLE_DEFINE_SERVICE_SETTING  0x42
#define SEPROXYHAL_TAG_NFC_DEFINE_SERVICE_SETTING  0x43
#define SEPROXYHAL_TAG_BLE_RADIO_POWER             0x44
#define SEPROXYHAL_TAG_NFC_RADIO_POWER             0x45
#define SEPROXYHAL_TAG_SE_POWER_OFF                0x46
#define SEPROXYHAL_TAG_SCREEN_POWER                0x47
#define SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE         0x48 
#define SEPROXYHAL_TAG_BATTERY_LEVEL               0x49
#define SEPROXYHAL_TAG_SCREEN_DISPLAY              0x4A // wait for display_event after sent
#define SEPROXYHAL_TAG_DEVICE_OFF                  0x4B
#define SEPROXYHAL_TAG_MORE_TIME                   0x4C
#define SEPROXYHAL_TAG_M24SR_C_APDU                0x4D
#define SEPROXYHAL_TAG_SET_TICKER_INTERVAL         0x4E
#define SEPROXYHAL_TAG_USB_CONFIG                  0x4F // <connect/disconnect> <nbendpoints> [<epaddr> <eptype:control/interrupt/bulk/isochrone/disabled> <epmps>]
#define SEPROXYHAL_TAG_USB_CONFIG_CONNECT           0x01
#define SEPROXYHAL_TAG_USB_CONFIG_DISCONNECT        0x02
#define SEPROXYHAL_TAG_USB_CONFIG_ADDR              0x03
#define SEPROXYHAL_TAG_USB_CONFIG_ENDPOINTS         0x04
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_DISABLED      0x00
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_CONTROL       0x01
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_INTERRUPT     0x02
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_BULK          0x03
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_ISOCHRONOUS   0x04
#define SEPROXYHAL_TAG_USB_EP_PREPARE              0x50 // <epnum> <direction:setup/in/out/stall/unstall> <expected_length> <data>
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_SETUP      0x10
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_IN         0x20
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_OUT        0x30
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_STALL      0x40
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_UNSTALL    0x80

// STATUS
#define SEPROXYHAL_TAG_GENERAL_STATUS              0x60
#define SEPROXYHAL_TAG_GENERAL_STATUS_LAST_COMMAND 0x0000
#define SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND 0x0001
#define SEPROXYHAL_TAG_GENERAL_STATUS_ERROR        0x0002
#define SEPROXYHAL_TAG_PAIRING_STATUS              0x61
#define SEPROXYHAL_TAG_BLE_READ_RESPONSE_STATUS    0x62
#define SEPROXYHAL_TAG_NFC_READ_RESPONSE_STATUS    0x63
#define SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS  0x64
#define SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS       0x65
#define SEPROXYHAL_TAG_PRINTF_STATUS               0x66
#define SEPROXYHAL_TAG_SET_LINK_SPEED              0x67 // <mhz(1byte)> <etu(1byte)>

#define IO_SEPROXYHAL_BUFFER_SIZE_B 300
extern volatile unsigned char G_io_seproxyhal_buffer[IO_SEPROXYHAL_BUFFER_SIZE_B];
extern volatile unsigned int G_io_seproxyhal_events;
#define IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT 32
extern volatile unsigned char G_io_ble_handles[IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT];
extern volatile unsigned char G_io_ble_apdu_protocol_enabled;

#define SEPROXYHAL_EVENT_WATCHDOG                       0x001UL
#define SEPROXYHAL_EVENT_TOUCH                          0x002UL
#define SEPROXYHAL_EVENT_BLE_WRITE                      0x004UL
#define SEPROXYHAL_EVENT_BLE_READ                       0x008UL
#define SEPROXYHAL_EVENT_USB_XFER_IN                    0x010UL
#define SEPROXYHAL_EVENT_USB_SETUP                      0x020UL
#define SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER    0x040UL
#define SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER  0x080UL
#define SEPROXYHAL_EVENT_BLE_CONNECT                    0x100UL
#define SEPROXYHAL_EVENT_BLE_DISCONNECT                 0x200UL
#define SEPROXYHAL_EVENT_RELEASE                        0x400UL
#define SEPROXYHAL_EVENT_TICKER                         0x800UL
//#define SEPROXYHAL_EVENT_USB_OUT                       0x1000UL
#define SEPROXYHAL_EVENT_USB_RESET                     0x2000UL
#define SEPROXYHAL_EVENT_USB_SOF                       0x4000UL
#define SEPROXYHAL_EVENT_USB_SUSPENDED                 0x8000UL
#define SEPROXYHAL_EVENT_USB_RESUMED                  0x10000UL
#define SEPROXYHAL_EVENT_USB_XFER_OUT                 0x20000UL
#define SEPROXYHAL_EVENT_BUTTON                       0x40000UL
#define SEPROXYHAL_EVENT_DISPLAYED                    0x80000UL
#define SEPROXYHAL_EVENT_UNSEC_CHUNK                 0x100000UL
#define SEPROXYHAL_EVENT_SET_LINK_SPEED              0x200000UL


extern volatile struct touch_state_s {
  short ts_last_x; 
  short ts_last_y;
} G_io_touch;

extern volatile unsigned char G_io_apdu_buffer[260];
extern volatile unsigned short G_io_apdu_length;

extern volatile struct ble_state_s {
  unsigned short gap_service_handle;
  unsigned short gap_dev_name_char_handle;
  unsigned short gap_appearance_char_handle;

  unsigned short service_handle, tx_characteristic_handle, rx_characteristic_handle;
  
  unsigned char client_link_established;
  unsigned short apdu_transport_remlen;
  unsigned short apdu_transport_seq;
  unsigned char* apdu_transport_ptr;
  unsigned char apdu_transport_busy_sending;
  unsigned char apdu_transport_lock;
  
  unsigned char connection_reset;
  unsigned int connection_timeout_ms;
  unsigned char connection_timeout_enabled;
  
#ifdef BLE_PACKET_ACK
  unsigned char apdu_transport_ack;
#endif // BLE_PACKET_ACK
  // public
  unsigned char apdu_available;
  unsigned short apdu_length;


  unsigned short notification_reg_handle;
  unsigned short notification_unreg_handle;
  unsigned short last_read_conn_handle;
  unsigned short last_read_attr_handle;
  unsigned short last_write_conn_handle;
  unsigned short last_write_attr_handle;
  unsigned char last_write_size;
  unsigned char last_write_buffer[512];

  unsigned char* last_discovered_name;
} G_io_ble;

void BLE_accept_previous_write(void);
void BLE_power(unsigned char powered, const char* discovered_name);
void BLE_send(unsigned short handle, uint8_t* data_buffer, uint8_t Nb_bytes);


extern unsigned int G_io_se_link_next_etu;
extern unsigned int G_io_se_link_next_mhz;
// return 1 if ok
unsigned char SE_iso_power_up(void);
// return 1 if ok
unsigned char SE_iso_power(unsigned char powered);
void SE_set_link_speed(unsigned int mhz, unsigned int etu);
unsigned short io_seproxyhal_rx_available(void);
unsigned short io_seproxyhal_recv(unsigned char* buffer, unsigned short length);
void io_seproxyhal_send(unsigned char* buffer, unsigned short length);
unsigned short SE_iso_exchange_apdu(unsigned char* apdu, unsigned short length);

void screen_init(unsigned char reinit);
void screen_clear(void);
void screen_poweroff(void);
void screen_update(void);
void screen_printf(const char* format,...);
void screen_xy(unsigned short x, unsigned short y, unsigned short rotation);
void screen_on(void); 
void screen_update_touch_event(void);
void screen_brightness(uint8_t percentage);
unsigned int battery_get_level_mv(void);

#define MAX_USB_ENDPOINTS 7 
#define MAX_USB_ENDPOINT_SIZE 64
extern volatile struct usb_state_s {
  unsigned char bootloader;
  // up to 32 ep manageable this way
  unsigned int  ep_in;
  unsigned int  ep_out;

  unsigned int  ep_in_len[MAX_USB_ENDPOINTS];
  unsigned int  ep_out_len[MAX_USB_ENDPOINTS];
  unsigned char ep_out_buff[MAX_USB_ENDPOINTS][MAX_USB_ENDPOINT_SIZE];
} G_io_usb;

#define CHANNEL_APDU 0
#define CHANNEL_KEYBOARD 1
#define CHANNEL_SPI 2
#define IO_RESET_AFTER_REPLIED 0x80
#define IO_RECEIVE_DATA 0x40
#define IO_RETURN_AFTER_TX 0x20
#define IO_FLAGS 0xF0

extern unsigned short io_exchange(unsigned char flags, unsigned short tx_length);
extern void io_usb_hid_init(void);



#define WAIT_COMMAND 1
#define WAIT_EVENT 2
#define WAIT_BLE_READ_DATA 3

#endif

