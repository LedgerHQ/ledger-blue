/*******************************************************************************
*   Ledger Blue - Secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
********************************************************************************/


#ifndef OS_IO_SEPROXYHAL_H
#define OS_IO_SEPROXYHAL_H

#include "os.h"

#ifdef OS_IO_SEPROXYHAL

#define BLE_CMD_APDU 0x05
#define BLE_CHUNK_LENGTH 20
#define M24SR_CHUNK_LENGTH 0xF6

// EVENTS
#define SEPROXYHAL_TAG_SESSION_START_EVENT 0x01
#define SEPROXYHAL_TAG_SESSION_START_EVENT_REQBLE 0x01
#define SEPROXYHAL_TAG_SESSION_START_EVENT_RECOVERY 0x02
#define SEPROXYHAL_TAG_BLE_PAIRING_ATTEMPT_EVENT 0x02
#define SEPROXYHAL_TAG_BLE_WRITE_REQUEST_EVENT 0x03
#define SEPROXYHAL_TAG_BLE_READ_REQUEST_EVENT 0x04
#define SEPROXYHAL_TAG_BUTTON_PUSH_EVENT 0x05
#define SEPROXYHAL_TAG_NFC_FIELD_DETECTION_EVENT 0x06
#define SEPROXYHAL_TAG_NFC_APDU_RECEIVED_EVENT 0x07
#define SEPROXYHAL_TAG_BATTERY_NOTIFICATION_EVENT 0x08
#define SEPROXYHAL_TAG_M24SR_GPO_CHANGE_EVENT 0x09
#define SEPROXYHAL_TAG_M24SR_RESPONSE_APDU_EVENT 0x0A
#define SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_EVENT 0x0B
#define SEPROXYHAL_TAG_FINGER_EVENT 0x0C
#define SEPROXYHAL_TAG_FINGER_EVENT_TOUCH 0x01
#define SEPROXYHAL_TAG_FINGER_EVENT_RELEASE 0x02
#define SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT 0x0D
#define SEPROXYHAL_TAG_TICKER_EVENT 0x0E
#define SEPROXYHAL_TAG_USB_EVENT 0x0F // <connect/disconnect/suspend/resume>
#define SEPROXYHAL_TAG_USB_EVENT_RESET 0x01
#define SEPROXYHAL_TAG_USB_EVENT_SOF 0x02
#define SEPROXYHAL_TAG_USB_EVENT_SUSPENDED 0x04
#define SEPROXYHAL_TAG_USB_EVENT_RESUMED 0x08
#define SEPROXYHAL_TAG_USB_EP_XFER_EVENT                                       \
    0x10 // <epnum> <xferin/xferout/xfersetup> <datalength> <data>
#define SEPROXYHAL_TAG_USB_EP_XFER_SETUP 0x01
#define SEPROXYHAL_TAG_USB_EP_XFER_IN 0x02
#define SEPROXYHAL_TAG_USB_EP_XFER_OUT 0x04
#define SEPROXYHAL_TAG_BLE_CONNECTION_EVENT                                    \
    0x11 // <connected(1)|disconnected(0)>
#define SEPROXYHAL_TAG_UNSEC_CHUNK_EVENT 0x12
#define SEPROXYHAL_TAG_ACK_LINK_SPEED                                          \
    0x13 // <mhz(1byte)> <etu(1byte)> <ack=1|nack=0 (1byte)>

// COMMANDS
#define SEPROXYHAL_TAG_GO_BOOTLOADER 0x31
#define SEPROXYHAL_TAG_UNSEC_CHUNK_READ 0x32 // <chunklen2BE> <reset(1|0)>
#define SEPROXYHAL_TAG_BLE_DEFINE_GENERIC_SETTING 0x41
#define SEPROXYHAL_TAG_BLE_DEFINE_SERVICE_SETTING 0x42
#define SEPROXYHAL_TAG_NFC_DEFINE_SERVICE_SETTING 0x43
#define SEPROXYHAL_TAG_BLE_RADIO_POWER 0x44
#define SEPROXYHAL_TAG_NFC_RADIO_POWER 0x45
#define SEPROXYHAL_TAG_SE_POWER_OFF 0x46
#define SEPROXYHAL_TAG_SCREEN_POWER 0x47
#define SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE 0x48
#define SEPROXYHAL_TAG_BATTERY_LEVEL 0x49
#define SEPROXYHAL_TAG_SCREEN_DISPLAY 0x4A // wait for display_event after sent
#define SEPROXYHAL_TAG_DEVICE_OFF 0x4B
#define SEPROXYHAL_TAG_MORE_TIME 0x4C
#define SEPROXYHAL_TAG_M24SR_C_APDU 0x4D
#define SEPROXYHAL_TAG_SET_TICKER_INTERVAL 0x4E
#define SEPROXYHAL_TAG_USB_CONFIG                                              \
    0x4F // <connect/disconnect> <nbendpoints> [<epaddr>
         // <eptype:control/interrupt/bulk/isochrone/disabled> <epmps>]
#define SEPROXYHAL_TAG_USB_CONFIG_CONNECT 0x01
#define SEPROXYHAL_TAG_USB_CONFIG_DISCONNECT 0x02
#define SEPROXYHAL_TAG_USB_CONFIG_ADDR 0x03
#define SEPROXYHAL_TAG_USB_CONFIG_ENDPOINTS 0x04
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_DISABLED 0x00
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_CONTROL 0x01
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_INTERRUPT 0x02
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_BULK 0x03
#define SEPROXYHAL_TAG_USB_CONFIG_TYPE_ISOCHRONOUS 0x04
#define SEPROXYHAL_TAG_USB_EP_PREPARE                                          \
    0x50 // <epnum> <direction:setup/in/out/stall/unstall> <expected_length>
         // <data>
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_SETUP 0x10
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_IN 0x20
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_OUT 0x30
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_STALL 0x40
#define SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_UNSTALL 0x80

// STATUS
#define SEPROXYHAL_TAG_GENERAL_STATUS 0x60
#define SEPROXYHAL_TAG_GENERAL_STATUS_LAST_COMMAND 0x0000
#define SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND 0x0001
#define SEPROXYHAL_TAG_GENERAL_STATUS_ERROR 0x0002
#define SEPROXYHAL_TAG_PAIRING_STATUS 0x61
#define SEPROXYHAL_TAG_BLE_READ_RESPONSE_STATUS 0x62
#define SEPROXYHAL_TAG_NFC_READ_RESPONSE_STATUS 0x63
#define SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS 0x64
#define SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS 0x65
#define SEPROXYHAL_TAG_PRINTF_STATUS 0x66
#define SEPROXYHAL_TAG_SET_LINK_SPEED 0x67 // <mhz(1byte)> <etu(1byte)>

#ifdef HAVE_BAGL
#include "bagl.h"

typedef struct bagl_element_e bagl_element_t;

// callback returns 0 when element must not be redrawn (with a changing color or
// what so ever)
typedef unsigned int (*bagl_element_callback_t)(const bagl_element_t *element);

// a graphic element is an element with defined text and actions depending on
// user touches
struct bagl_element_e {
    bagl_component_t component;

    const char *text;
    unsigned char touch_area_brim;
    int overfgcolor;
    int overbgcolor;
    bagl_element_callback_t tap;
    bagl_element_callback_t out;
    bagl_element_callback_t over;
};

// touch management helper function (callback the call with the element for the
// given position, taking into account touch release)
void io_seproxyhal_touch(const bagl_element_t *elements,
                         unsigned short element_count, unsigned short x,
                         unsigned short y, unsigned char event_kind);
// callback to be implemented by the se
void io_seproxyhal_touch_callback(const bagl_element_t *element,
                                  unsigned char event);

// hal point (if application has to reprocess elements)
void io_seproxyhal_display(const bagl_element_t *element);

// default version to be called by ::io_seproxyhal_display if nothing to be done
// by the application
void io_seproxyhal_display_default(bagl_element_t *element);
#endif // HAVE_BAGL

extern unsigned char G_io_seproxyhal_spi_buffer[IO_SEPROXYHAL_BUFFER_SIZE_B];

// can be called by application
SYSCALL void io_seproxyhal_spi_send(unsigned char *buffer PLENGTH(length),
                                    unsigned short length);

// not to be called by application (application is triggered using io_event
// instead), resered for seproxyhal
#define IO_CACHE 1
SYSCALL unsigned short
io_seproxyhal_spi_recv(unsigned char *buffer PLENGTH(maxlength),
                       unsigned short maxlength, unsigned int flags);

// HAL init
SYSCALL void io_seproxyhal_spi_init(void);

void io_seproxyhal_init(void);

// delegate function for generic io_exchange
unsigned short io_exchange_al(unsigned char channel_and_flags,
                              unsigned short tx_len);

// for delegation of Native NFC / USB
unsigned char io_event(unsigned char channel);

void io_seproxyhal_handle_usb_event(void);
void io_seproxyhal_handle_usb_ep_xfer_event(void);

// process event for io protocols when waiting for a ux operation to end
// return 1 when event replied, 0 else
unsigned int io_seproxyhal_handle_event(void);

// reply a general status last command
void io_seproxyhal_general_status(void);

void io_usb_send_apdu_data(unsigned char *buffer, unsigned short length);

typedef enum {
    APDU_IDLE,
    APDU_BLE,
    APDU_BLE_WAIT_NOTIFY,
    APDU_NFC_M24SR,
    APDU_NFC_M24SR_SELECT,
    APDU_NFC_M24SR_FIRST,
    APDU_NFC_M24SR_RAPDU,
    APDU_USB_HID,
} io_apdu_state_e;

extern volatile io_apdu_state_e io_apdu_state; // by default

extern volatile unsigned short io_apdu_offset; // total length already received
extern volatile unsigned short io_apdu_length; // total length to be received
extern volatile unsigned short io_apdu_seq;

#ifdef HAVE_USB
extern unsigned int usb_ep_xfer_len[7];
#endif // HAVE_USB

/**
 *  Ledger Bluetooth Low Energy APDU Protocol
 *  Characteristic content:
 *  [______________________________]
 *   TT SSSS VVVV................VV
 *
 *  All fields are big endian encoded.
 *  TT: 1 byte content tag
 *  SSSS: 2 bytes sequence number, big endian encoded (start @ 0).
 *  VVVV..VV: variable length content. When SSSS is 0, the first two bytes
 * encodes in big endian the total length of the APDU to transport.
 *
 *  Command/Response APDU are split in chunks to fill up the bluetooth's
 * characteristic
 *
 *  APDU are using either standard or extended header. up to the application to
 * check the total received length and the lc field
 *
 *  Tags:
 *  Direction:*  T:0x05 S=<sequence-idx-U2BE>
 * V=<seq==0?totallength(U2BE):NONE><apducontent> APDU (command/response)
 * packet.
 *
 * Example:
 * --------
 *   Wrapping of Command APDU:
 *     E0 FF 12 13 14
 *     15 16 17 18 19 1A 1B 1C
 *     1D 1E 1F 20 21 22 23 24
 *     25 26 27 28 29 2A 2B 2C
 *     2D 2E 2F 30 31 32 33 34
 *     35
 *   Result in 3 chunks (20 bytes at most):
 *     0500000026E0FF12131415161718191A1B1C1D1E
 *     0500011F202122232425262728292A2B2C2D2E2F
 *     050002303132333435
 *
 *
 *   Wrapping of Response APDU:
 *     15 16 17 18 19 1a 1b 1c
 *     1d 1e 1f 20 21 22 23 24
 *     25 26 27 28 29 2a 2b 2c
 *     2d 2e 2f 30 31 32 33 34
 *     35 90 00
 *   Result in 3 chunks (20 bytes at most):
 *     050000002315161718191a1b1c1d1e1f20212223
 *     0500012425262728292a2b2c2d2e2f3031323334
 *     050002359000
 */

#endif // OS_IO_SEPROXYHAL

#endif // OS_IO_SEPROXYHAL_H
