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

#include "os.h"

#ifdef OS_IO_SEPROXYHAL

#include "os_io_seproxyhal.h"

#ifdef DEBUG
#define LOG printf
#else
#define LOG(...)
#endif

volatile io_apdu_state_e G_io_apdu_state; // by default
volatile unsigned short G_io_apdu_offset; // total length already received
volatile unsigned short G_io_apdu_length; // total length to be received
volatile unsigned short G_io_apdu_seq;
volatile io_apdu_media_t G_io_apdu_media;

void io_seproxyhal_general_status(void) {
    // send the general status
    G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_GENERAL_STATUS;
    G_io_seproxyhal_spi_buffer[1] = 0;
    G_io_seproxyhal_spi_buffer[2] = 2;
    G_io_seproxyhal_spi_buffer[3] =
        SEPROXYHAL_TAG_GENERAL_STATUS_LAST_COMMAND >> 8;
    G_io_seproxyhal_spi_buffer[4] = SEPROXYHAL_TAG_GENERAL_STATUS_LAST_COMMAND;
    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 5);
}

#ifdef HAVE_IO_USB
#ifdef HAVE_L4_USBLIB
extern volatile unsigned char G_io_usb_ep_xfer_len[IO_USB_MAX_ENDPOINTS];
#include "usbd_def.h"
#include "usbd_core.h"
extern USBD_HandleTypeDef USBD_Device;

void io_seproxyhal_handle_usb_event(void) {
    switch (G_io_seproxyhal_spi_buffer[3]) {
    case SEPROXYHAL_TAG_USB_EVENT_RESET:
        USBD_LL_SetSpeed(&USBD_Device, USBD_SPEED_FULL);
        USBD_LL_Reset(&USBD_Device);
        break;
    case SEPROXYHAL_TAG_USB_EVENT_SOF:
        USBD_LL_SOF(&USBD_Device);
        break;
    case SEPROXYHAL_TAG_USB_EVENT_SUSPENDED:
        USBD_LL_Suspend(&USBD_Device);
        break;
    case SEPROXYHAL_TAG_USB_EVENT_RESUMED:
        USBD_LL_Resume(&USBD_Device);
        break;
    }
}

void io_seproxyhal_handle_usb_ep_xfer_event(void) {
    switch (G_io_seproxyhal_spi_buffer[4]) {
    case SEPROXYHAL_TAG_USB_EP_XFER_SETUP:
        // assume length of setup packet, and that it is on endpoint 0
        USBD_LL_SetupStage(&USBD_Device, &G_io_seproxyhal_spi_buffer[6]);
        break;

    case SEPROXYHAL_TAG_USB_EP_XFER_IN:
        USBD_LL_DataInStage(&USBD_Device, G_io_seproxyhal_spi_buffer[3] & 0x7F,
                            &G_io_seproxyhal_spi_buffer[6]);
        break;

    case SEPROXYHAL_TAG_USB_EP_XFER_OUT:
        // saved just in case it is needed ...
        G_io_usb_ep_xfer_len[G_io_seproxyhal_spi_buffer[3] & 0x7F] =
            G_io_seproxyhal_spi_buffer[5];
        USBD_LL_DataOutStage(&USBD_Device, G_io_seproxyhal_spi_buffer[3] & 0x7F,
                             &G_io_seproxyhal_spi_buffer[6]);
        break;
    }
}

#else
// no usb lib: X86 for example

void io_seproxyhal_handle_usb_event(void) {
}
void io_seproxyhal_handle_usb_ep_xfer_event(void) {
}

#endif // HAVE_L4_USBLIB

void io_usb_send_apdu_data(unsigned char *buffer, unsigned short length) {
    unsigned int rx_len;

    // won't send if overflowing seproxyhal buffer
    if (length + 6 > sizeof(G_io_seproxyhal_spi_buffer)) {
        return;
    }
    G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_USB_EP_PREPARE;
    G_io_seproxyhal_spi_buffer[1] = (3 + length) >> 8;
    G_io_seproxyhal_spi_buffer[2] = (3 + length);
    G_io_seproxyhal_spi_buffer[3] = 0x82;
    G_io_seproxyhal_spi_buffer[4] = SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_IN;
    G_io_seproxyhal_spi_buffer[5] = length;
    os_memmove(G_io_seproxyhal_spi_buffer + 6, buffer, length);
    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, length + 6);

    // wait for event
    io_seproxyhal_general_status();

    for (;;) {
        rx_len = io_seproxyhal_spi_recv(G_io_seproxyhal_spi_buffer,
                                        sizeof(G_io_seproxyhal_spi_buffer), 0);

        // wait for ack of the seproxyhal
        // discard if not an acknowledgment
        if (G_io_seproxyhal_spi_buffer[0] != SEPROXYHAL_TAG_USB_EP_XFER_EVENT ||
            rx_len != 6 || G_io_seproxyhal_spi_buffer[3] != 0x82 ||
            G_io_seproxyhal_spi_buffer[4] != SEPROXYHAL_TAG_USB_EP_XFER_IN ||
            G_io_seproxyhal_spi_buffer[5] != length) {
            if (!io_event(CHANNEL_SPI)) {
                LOG("missing NOTIFY_INDICATE event, %02X received\n",
                    G_io_seproxyhal_spi_buffer[0]);
            }

            // ack and wiat next event
            io_seproxyhal_general_status();
            continue;
        }

        // chunk sending succeeded
        break;
    }
}

#endif // HAVE_IO_USB

unsigned int io_seproxyhal_handle_event(void) {
    unsigned int rx_len = U2BE(G_io_seproxyhal_spi_buffer, 1);

    switch (G_io_seproxyhal_spi_buffer[0]) {
#ifdef HAVE_IO_USB
    case SEPROXYHAL_TAG_USB_EVENT:
        if (rx_len != 3 + 1) {
            return 0;
        }
        io_seproxyhal_handle_usb_event();

        // no state change, we're not dealing with an apdu yet
        io_seproxyhal_general_status();
        return 1;

    case SEPROXYHAL_TAG_USB_EP_XFER_EVENT:
        if (rx_len < 3 + 3) {
            // error !
            return 0;
        }
        io_seproxyhal_handle_usb_ep_xfer_event();

        io_seproxyhal_general_status();
        return 1;
#endif // HAVE_IO_USB
    }
    // defaulty return as not processed
    return 0;
}

volatile bagl_element_t *G_bagl_last_touched_not_released_component;

//#define DEBUG_APDU
#ifdef DEBUG_APDU
volatile unsigned int debug_apdus_offset;
const char debug_apdus[] = {
    9, 0xe0, 0x22, 0x00, 0x00, 0x04, 0x31, 0x32, 0x33, 0x34,
};
#endif // DEBUG_APDU

void io_seproxyhal_init(void) {
    G_io_apdu_state = APDU_IDLE;
    G_io_apdu_offset = 0;
    G_io_apdu_length = 0;
    G_io_apdu_seq = 0;
    G_io_apdu_media = IO_APDU_MEDIA_NONE;

#ifdef DEBUG_APDU
    debug_apdus_offset = 0;
#endif // DEBUG_APDU

    // initialize the touch part
    G_bagl_last_touched_not_released_component = NULL;

#ifdef HAVE_IO_USB
    io_usb_hid_init();
#endif // HAVE_IO_USB

    io_seproxyhal_spi_init();
}

#ifdef HAVE_BAGL

void io_seproxyhal_touch_out(const bagl_element_t *element) {
    if (element->out != NULL) {
        if (!((bagl_element_callback_t)PIC(element->out))(element)) {
            return;
        }
    }
    io_seproxyhal_display(element);
}

void io_seproxyhal_touch_over(const bagl_element_t *element) {
    bagl_element_t e;
    if (element->over != NULL) {
        if (!((bagl_element_callback_t)PIC(element->over))(element)) {
            return;
        }
    }
    os_memmove(&e, element, sizeof(bagl_element_t));
    e.component.fgcolor = element->overfgcolor;
    e.component.bgcolor = element->overbgcolor;
    io_seproxyhal_display(&e);
}

void io_seproxyhal_touch_tap(const bagl_element_t *element) {
    if (element->tap != NULL) {
        if (!((bagl_element_callback_t)PIC(element->tap))(element)) {
            return;
        }
    }
    io_seproxyhal_display(element);
}

void io_seproxyhal_touch(const bagl_element_t *elements,
                         unsigned short element_count, unsigned short x,
                         unsigned short y, unsigned char event_kind) {
    unsigned char comp_idx;

    // find the first empty entry
    for (comp_idx = 0; comp_idx < element_count; comp_idx++) {
        // the first component drawn with a
        if ((elements[comp_idx].component.type & BAGL_FLAG_TOUCHABLE) &&
            elements[comp_idx].component.x -
                    elements[comp_idx].touch_area_brim <=
                x &&
            x < elements[comp_idx].component.x +
                    elements[comp_idx].component.width +
                    elements[comp_idx].touch_area_brim &&
            elements[comp_idx].component.y -
                    elements[comp_idx].touch_area_brim <=
                y &&
            y < elements[comp_idx].component.y +
                    elements[comp_idx].component.height +
                    elements[comp_idx].touch_area_brim) {
            // outing the previous overed component
            if (&elements[comp_idx] !=
                    G_bagl_last_touched_not_released_component &&
                G_bagl_last_touched_not_released_component != NULL) {
                io_seproxyhal_touch_out(
                    G_bagl_last_touched_not_released_component);
                // ok component out has been emitted
                G_bagl_last_touched_not_released_component = NULL;
            }

            // unmark the last component, we've been notified TOUCH
            if (&elements[comp_idx] ==
                    G_bagl_last_touched_not_released_component &&
                event_kind == SEPROXYHAL_TAG_FINGER_EVENT_RELEASE) {
                G_bagl_last_touched_not_released_component = NULL;
            }
            // remember the last touched component
            else if (event_kind == SEPROXYHAL_TAG_FINGER_EVENT_TOUCH) {
                G_bagl_last_touched_not_released_component =
                    &elements[comp_idx];
            }

            // callback the hal to notify the component impacted by the user
            // input
            if (event_kind == SEPROXYHAL_TAG_FINGER_EVENT_RELEASE) {
                io_seproxyhal_touch_tap(&elements[comp_idx]);
            } else {
                io_seproxyhal_touch_over(&elements[comp_idx]);
            }

            // process all components matching the x/y/w/h (no break) => fishy
            // for the released out of zone
            return;
        }
    }

    // if overing out of component or over another component, the out event is
    // sent after the over event of the previous component
    if (G_bagl_last_touched_not_released_component != NULL) {
        io_seproxyhal_touch_out(G_bagl_last_touched_not_released_component);
        // ok component out has been emitted
        G_bagl_last_touched_not_released_component = NULL;
    }
}

extern unsigned int _text;
void io_seproxyhal_display_default(bagl_element_t *element) {
    // process automagically address from rom and from ram

    if (element->text != NULL) {
        unsigned int text_adr = (unsigned int)element->text;
        if (text_adr >= (unsigned int)&_text) {
            text_adr = PIC(text_adr);
        }
        unsigned short length =
            sizeof(bagl_component_t) + strlen((const char *)text_adr);
        G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS;
        G_io_seproxyhal_spi_buffer[1] = length >> 8;
        G_io_seproxyhal_spi_buffer[2] = length;
        os_memcpy(&G_io_seproxyhal_spi_buffer[3], &element->component,
                  sizeof(bagl_component_t));
        os_memcpy(&G_io_seproxyhal_spi_buffer[3 + sizeof(bagl_component_t)],
                  (const void *)text_adr, strlen((const char *)text_adr));
        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, length + 3);
    } else {
        unsigned short length = sizeof(bagl_component_t);
        G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS;
        G_io_seproxyhal_spi_buffer[1] = length >> 8;
        G_io_seproxyhal_spi_buffer[2] = length;
        os_memcpy(&G_io_seproxyhal_spi_buffer[3], &element->component,
                  sizeof(bagl_component_t));
        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, length + 3);
    }
}

#endif // HAVE_BAGL

unsigned short io_exchange(unsigned char channel, unsigned short tx_len) {
    unsigned short rx_len;

#ifdef DEBUG_APDU
    if ((channel & ~(IO_FLAGS)) == CHANNEL_APDU) {
        // ignore tx len

        // already received the data of the apdu when received the whole apdu
        if ((channel & (CHANNEL_APDU | IO_RECEIVE_DATA)) ==
            (CHANNEL_APDU | IO_RECEIVE_DATA)) {
            // return apdu data - header
            return G_io_apdu_length - 5;
        }

        // fetch next apdu
        if (debug_apdus_offset < sizeof(debug_apdus)) {
            G_io_apdu_length = debug_apdus[debug_apdus_offset] & 0xFF;
            os_memmove(G_io_apdu_buffer, &debug_apdus[debug_apdus_offset + 1],
                       G_io_apdu_length);
            debug_apdus_offset += G_io_apdu_length + 1;
            return G_io_apdu_length;
        }
    }
after_debug:
#endif // DEBUG_APDU

    switch (channel & ~(IO_FLAGS)) {
    case CHANNEL_APDU:
        // TODO work up the spi state machine over the HAL proxy until an APDU
        // is available

        if (tx_len) {
            // until the whole RAPDU is transmitted, send chunks using the
            // current mode for communication
            for (;;) {
                switch (G_io_apdu_state) {
                default:
                    // delegate to the hal in case of not generic transport mode
                    // (or asynch)
                    if (io_exchange_al(channel, tx_len) == 0) {
                        goto break_send;
                    }
                case APDU_IDLE:
                    LOG("invalid state for APDU reply\n");
                    THROW(INVALID_STATE);
                    break;

#ifdef HAVE_IO_USB
                case APDU_USB_HID:
                    // only send, don't perform synchronous reception of the
                    // next command (will be done later by the seproxyhal packet
                    // processing)
                    io_usb_hid_exchange(io_usb_send_apdu_data, tx_len, NULL,
                                        IO_RETURN_AFTER_TX);
                    goto break_send;
#endif // HAVE_IO_USB

                case APDU_BLE:
                    // split in notify blocks and wait for each notify block to
                    // be replied
                    // any other event is discarded

                    G_io_apdu_length = tx_len;
                    G_io_apdu_offset = 0;
                    G_io_apdu_seq = 0;

                    G_io_apdu_state = APDU_BLE_WAIT_NOTIFY;

                    tx_len = MIN(G_io_apdu_length, BLE_CHUNK_LENGTH - 5);

#ifdef ST31
                    G_io_seproxyhal_spi_buffer[0] =
                        SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS;
#else
                    G_io_seproxyhal_spi_buffer[0] =
                        SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE;
#endif
                    G_io_seproxyhal_spi_buffer[1] = (tx_len + 5 + 3) >> 8;
                    G_io_seproxyhal_spi_buffer[2] = (tx_len + 5 + 3);
                    G_io_seproxyhal_spi_buffer[3] = 1; // default notify handle
                    G_io_seproxyhal_spi_buffer[4] = (tx_len + 5) >> 8;
                    G_io_seproxyhal_spi_buffer[5] = (tx_len + 5);
                    G_io_seproxyhal_spi_buffer[6] = BLE_CMD_APDU;
                    G_io_seproxyhal_spi_buffer[7] = G_io_apdu_seq >> 8;
                    G_io_seproxyhal_spi_buffer[8] = G_io_apdu_seq;
                    G_io_seproxyhal_spi_buffer[9] = G_io_apdu_length >> 8;
                    G_io_seproxyhal_spi_buffer[10] = G_io_apdu_length;
                    os_memmove(G_io_seproxyhal_spi_buffer + 11,
                               G_io_apdu_buffer + G_io_apdu_offset, tx_len);
                    G_io_apdu_seq++;
                    G_io_apdu_offset += tx_len;

                    // send next chunk, wait for reply
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer,
                                           tx_len + 11);
                    break;

                case APDU_BLE_WAIT_NOTIFY:
                    // await notify acknowledgment
                    rx_len = io_seproxyhal_spi_recv(
                        G_io_seproxyhal_spi_buffer,
                        sizeof(G_io_seproxyhal_spi_buffer), 0);

                    // discard if not an acknowledgment
                    if (G_io_seproxyhal_spi_buffer[0] !=
                        SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_EVENT) {
                        if (!io_event(CHANNEL_SPI)) {
                            LOG("missing NOTIFY_INDICATE event, %02X "
                                "received\n",
                                G_io_seproxyhal_spi_buffer[0]);
                        }
                        continue;
                    }

                    if (G_io_apdu_offset < G_io_apdu_length) {
                        // send next chunk
                        tx_len = MIN(G_io_apdu_length - G_io_apdu_offset,
                                     BLE_CHUNK_LENGTH - 3);

#ifdef ST31
                        G_io_seproxyhal_spi_buffer[0] =
                            SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS;
#else
                        G_io_seproxyhal_spi_buffer[0] =
                            SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE;
#endif
                        G_io_seproxyhal_spi_buffer[1] = (tx_len + 3 + 3) >> 8;
                        G_io_seproxyhal_spi_buffer[2] = (tx_len + 3 + 3);
                        G_io_seproxyhal_spi_buffer[3] =
                            1; // default notify handle
                        G_io_seproxyhal_spi_buffer[4] = (tx_len + 3) >> 8;
                        G_io_seproxyhal_spi_buffer[5] = (tx_len + 3);
                        G_io_seproxyhal_spi_buffer[6] = BLE_CMD_APDU;
                        G_io_seproxyhal_spi_buffer[7] = G_io_apdu_seq >> 8;
                        G_io_seproxyhal_spi_buffer[8] = G_io_apdu_seq;
                        os_memmove(G_io_seproxyhal_spi_buffer + 9,
                                   G_io_apdu_buffer + G_io_apdu_offset, tx_len);
                        G_io_apdu_seq++;
                        G_io_apdu_offset += tx_len;

                        // send next chunk, wait for reply
                        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer,
                                               tx_len + 9);
                    } else {
                        // finished sending data
                        goto break_send;
                    }
                    break;

                case APDU_NFC_M24SR:
                    // split in apdu to write into the M24SR and wait M24SR
                    // rapdu response.
                    // select the NDEF file
                    // issue an APDU to read the first block content (containing
                    // the total apdu length) (read as much as possible)
                    G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_M24SR_C_APDU;
                    G_io_seproxyhal_spi_buffer[1] = 0;
                    G_io_seproxyhal_spi_buffer[2] = 7;
                    // Forge NDEF Select command
                    G_io_seproxyhal_spi_buffer[3] = 0x00;
                    G_io_seproxyhal_spi_buffer[4] = 0xA4;
                    G_io_seproxyhal_spi_buffer[5] = 0x00;
                    G_io_seproxyhal_spi_buffer[6] = 0x0C;
                    G_io_seproxyhal_spi_buffer[7] = 0x02;
                    G_io_seproxyhal_spi_buffer[8] = 0x00;
                    G_io_seproxyhal_spi_buffer[9] = 0x01;

                    // wait for a NFC_RESPONSE
                    G_io_apdu_state = APDU_NFC_M24SR_SELECT;
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 10);
                    break;

                case APDU_NFC_M24SR_SELECT:
                    // wait for select reply
                    rx_len = io_seproxyhal_spi_recv(
                        G_io_seproxyhal_spi_buffer,
                        sizeof(G_io_seproxyhal_spi_buffer), 0);

                    if (rx_len - 3 != U2(G_io_seproxyhal_spi_buffer[1],
                                         G_io_seproxyhal_spi_buffer[2])) {
                        LOG("invalid TLV format\n");
                        goto invalid_apdu_packet;
                    }
                    // expect M24SR RAPDU with 0x90 0x00
                    if (G_io_seproxyhal_spi_buffer[0] !=
                        SEPROXYHAL_TAG_M24SR_RESPONSE_APDU_EVENT) {
                        if (!io_event(CHANNEL_SPI)) {
                            LOG("invalid SELECT reply\n");
                            // goto invalid_apdu_packet;
                        }
                        continue;
                    }
                    if (G_io_seproxyhal_spi_buffer[1] != 0 ||
                        G_io_seproxyhal_spi_buffer[2] != 2 ||
                        G_io_seproxyhal_spi_buffer[3] != 0x90 ||
                        G_io_seproxyhal_spi_buffer[4] != 0x00) {
                        LOG("invalid SELECT reply\n");
                        goto invalid_apdu_packet;
                    }

                    G_io_apdu_length = tx_len;
                    G_io_apdu_offset = 0;

                    tx_len = MIN(0xF6 - 2, G_io_apdu_length);

                    // forge the first update binary, putting 0x00 0x00 into the
                    // RAPDU NDEF file length
                    G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_M24SR_C_APDU;
                    G_io_seproxyhal_spi_buffer[1] = 0;
                    G_io_seproxyhal_spi_buffer[2] = tx_len +
                                                    5 /*update binary header*/ +
                                                    2 /* NDEF file length*/;
                    // Forge NDEF Select command
                    G_io_seproxyhal_spi_buffer[3] = 0x00;
                    G_io_seproxyhal_spi_buffer[4] = 0xD6;
                    G_io_seproxyhal_spi_buffer[5] = 0x00;
                    G_io_seproxyhal_spi_buffer[6] = 0x00;
                    G_io_seproxyhal_spi_buffer[7] =
                        tx_len + 2 /*NDEF file length*/;
                    G_io_seproxyhal_spi_buffer[8] = 0x00;
                    G_io_seproxyhal_spi_buffer[9] = 0x00;
                    os_memmove(G_io_seproxyhal_spi_buffer + 10,
                               G_io_apdu_buffer + G_io_apdu_offset, tx_len);
                    G_io_apdu_offset = tx_len;

                    // wait for a NFC_RESPONSE
                    G_io_apdu_state = APDU_NFC_M24SR_RAPDU;
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer,
                                           G_io_seproxyhal_spi_buffer[2] + 3);
                    break;

                case APDU_NFC_M24SR_RAPDU:
                    // wait for update binary reply
                    rx_len = io_seproxyhal_spi_recv(
                        G_io_seproxyhal_spi_buffer,
                        sizeof(G_io_seproxyhal_spi_buffer), 0);

                    if (rx_len - 3 != U2(G_io_seproxyhal_spi_buffer[1],
                                         G_io_seproxyhal_spi_buffer[2])) {
                        LOG("invalid TLV format\n");
                        goto invalid_apdu_packet;
                    }
                    // expect M24SR RAPDU with 0x90 0x00
                    if (G_io_seproxyhal_spi_buffer[0] !=
                        SEPROXYHAL_TAG_M24SR_RESPONSE_APDU_EVENT) {
                        if (!io_event(CHANNEL_SPI)) {
                            LOG("invalid UPDATE BINARY reply\n");
                            // goto invalid_apdu_packet;
                        }
                        continue;
                    }
                    if (G_io_seproxyhal_spi_buffer[1] != 0 ||
                        G_io_seproxyhal_spi_buffer[2] != 2 ||
                        G_io_seproxyhal_spi_buffer[3] != 0x90 ||
                        G_io_seproxyhal_spi_buffer[4] != 0x00) {
                        LOG("invalid UPDATE BINARY reply\n");
                        goto invalid_apdu_packet;
                    }

                    if (G_io_apdu_length == G_io_apdu_offset) {
                        // update ndef length
                        G_io_seproxyhal_spi_buffer[0] =
                            SEPROXYHAL_TAG_M24SR_C_APDU;
                        G_io_seproxyhal_spi_buffer[1] = 0;
                        G_io_seproxyhal_spi_buffer[2] =
                            7 /*update binary header*/;
                        // Forge NDEF Select command
                        G_io_seproxyhal_spi_buffer[3] = 0x00;
                        G_io_seproxyhal_spi_buffer[4] = 0xD6;
                        G_io_seproxyhal_spi_buffer[5] = 0x00;
                        G_io_seproxyhal_spi_buffer[6] = 0x00;
                        G_io_seproxyhal_spi_buffer[7] = 2;
                        G_io_seproxyhal_spi_buffer[8] = G_io_apdu_length >> 8;
                        G_io_seproxyhal_spi_buffer[9] = G_io_apdu_length;

                        G_io_apdu_state = APDU_NFC_M24SR_FIRST;
                        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer,
                                               G_io_seproxyhal_spi_buffer[2] +
                                                   3);
                    } else {
                        // update a part of the apdu
                        tx_len = MIN(0xF6, G_io_apdu_length - G_io_apdu_offset);

                        // forge the first update binary, putting 0x00 0x00 into
                        // the RAPDU NDEF file length
                        G_io_seproxyhal_spi_buffer[0] =
                            SEPROXYHAL_TAG_M24SR_C_APDU;
                        G_io_seproxyhal_spi_buffer[1] = 0;
                        G_io_seproxyhal_spi_buffer[2] =
                            tx_len + 5 /*update binary header*/;
                        // Forge NDEF Select command
                        G_io_seproxyhal_spi_buffer[3] = 0x00;
                        G_io_seproxyhal_spi_buffer[4] = 0xD6;
                        G_io_seproxyhal_spi_buffer[5] =
                            (G_io_apdu_offset + 2 /*NDEF file length*/) >> 8;
                        G_io_seproxyhal_spi_buffer[6] =
                            (G_io_apdu_offset + 2 /*NDEF file length*/);
                        G_io_seproxyhal_spi_buffer[7] = tx_len;
                        os_memmove(G_io_seproxyhal_spi_buffer + 8,
                                   G_io_apdu_buffer + G_io_apdu_offset, tx_len);
                        G_io_apdu_offset += tx_len;

                        // wait for a NFC_RESPONSE
                        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer,
                                               G_io_seproxyhal_spi_buffer[2] +
                                                   3);
                    }
                    break;

                // update the NDEF length after all the content of the RAPDU has
                // been written
                case APDU_NFC_M24SR_FIRST:
                    // wait for last update binary reply
                    rx_len = io_seproxyhal_spi_recv(
                        G_io_seproxyhal_spi_buffer,
                        sizeof(G_io_seproxyhal_spi_buffer), 0);

                    if (rx_len - 3 != U2(G_io_seproxyhal_spi_buffer[1],
                                         G_io_seproxyhal_spi_buffer[2])) {
                        LOG("invalid TLV format\n");
                        goto invalid_apdu_packet;
                    }
                    // expect M24SR RAPDU with 0x90 0x00
                    if (G_io_seproxyhal_spi_buffer[0] !=
                        SEPROXYHAL_TAG_M24SR_RESPONSE_APDU_EVENT) {
                        if (!io_event(CHANNEL_SPI)) {
                            LOG("invalid NDEF file length UPDATE BINARY "
                                "reply\n");
                            // goto invalid_apdu_packet;
                        }
                        continue;
                    }
                    if (G_io_seproxyhal_spi_buffer[1] != 0 ||
                        G_io_seproxyhal_spi_buffer[2] != 2 ||
                        G_io_seproxyhal_spi_buffer[3] != 0x90 ||
                        G_io_seproxyhal_spi_buffer[4] != 0x00) {
                        LOG("invalid NDEF file length UPDATE BINARY reply\n");
                        goto invalid_apdu_packet;
                    }
                    goto break_send;
                }
                continue;

            break_send:
                // continue sending commands, don't issue status yet
                if (channel & IO_RETURN_AFTER_TX) {
                    return 0;
                }
                // acknowledge the write request (general status OK) and no more
                // command to follow (wait until another APDU container is
                // received to continue unwrapping)
                io_seproxyhal_general_status();
                break;
            }

            // perform reset after io exchange
            if (channel & IO_RESET_AFTER_REPLIED) {
                reset();
            }
        }

        // already received the data of the apdu when received the whole apdu
        if ((channel & (CHANNEL_APDU | IO_RECEIVE_DATA)) ==
            (CHANNEL_APDU | IO_RECEIVE_DATA)) {
            // return apdu data - header
            return G_io_apdu_length - 5;
        }

        // reply has ended, proceed to next apdu reception
        G_io_apdu_state = APDU_IDLE;
        G_io_apdu_offset = 0;
        G_io_apdu_length = 0;
        G_io_apdu_seq = 0;
        G_io_apdu_media = IO_APDU_MEDIA_NONE;

        // until a new whole CAPDU is received
        for (;;) {
            // wait until a SPI packet is available
            // NOTE: on ST31, dual wait ISO & RF (ISO instead of SPI)
            rx_len =
                io_seproxyhal_spi_recv(G_io_seproxyhal_spi_buffer,
                                       sizeof(G_io_seproxyhal_spi_buffer), 0);

            // can't process split TLV, continue
            if (rx_len - 3 != U2(G_io_seproxyhal_spi_buffer[1],
                                 G_io_seproxyhal_spi_buffer[2])) {
                LOG("invalid TLV format\n");
                goto invalid_apdu_packet;
            }

            // depending on received TAG
            switch (G_io_seproxyhal_spi_buffer[0]) {
#ifdef HAVE_IO_USB
            case SEPROXYHAL_TAG_USB_EVENT:
                if (rx_len != 3 + 1) {
                    // invalid length, not processable
                    goto invalid_apdu_packet;
                }
                io_seproxyhal_handle_usb_event();

                // no state change, we're not dealing with an apdu yet
                goto send_last_command;

            case SEPROXYHAL_TAG_USB_EP_XFER_EVENT:
                if (rx_len < 3 + 3) {
                    // error !
                    return 0;
                }
                io_seproxyhal_handle_usb_ep_xfer_event();

                // an apdu has been received, ack with mode commands (the reply
                // at least)
                if (G_io_apdu_length > 0) {
                    // acknowledge the write request (general status OK) and
                    // more command to follow (processing of the apdu)
                    G_io_seproxyhal_spi_buffer[0] =
                        SEPROXYHAL_TAG_GENERAL_STATUS;
                    G_io_seproxyhal_spi_buffer[1] = 0;
                    G_io_seproxyhal_spi_buffer[2] = 2;
                    G_io_seproxyhal_spi_buffer[3] =
                        SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND >> 8;
                    G_io_seproxyhal_spi_buffer[4] =
                        SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND;
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 5);

                    G_io_apdu_media =
                        IO_APDU_MEDIA_USB_HID; // for application code
                    G_io_apdu_state =
                        APDU_USB_HID; // for next call to io_exchange
                    return G_io_apdu_length;
                } else {
                    goto send_last_command;
                }
                break;
#endif // HAVE_IO_USB

            case SEPROXYHAL_TAG_BLE_WRITE_REQUEST_EVENT:
                // join block of BLE apdu chunks into the CAPDU

                // ensure ready to process a BLE APDU
                if (G_io_apdu_state != APDU_BLE &&
                    G_io_apdu_state != APDU_IDLE) {
                    LOG("invalid state for BLE_WRITE_REQ\n");
                    // THROW(INVALID_STATE);
                    goto invalid_apdu_packet;
                }

                if (G_io_apdu_state == APDU_IDLE) {
                    // TAG LENH LENL HNDL WLENH WLENL CMD SEQH=0 SEQL=0 LENH
                    // LENL APDUDATA
                    // ensure the total length of the APDU is to be received,
                    // else ignore the packet
                    if (U2(G_io_seproxyhal_spi_buffer[1],
                           G_io_seproxyhal_spi_buffer[2]) < 1 + 2 + 1 + 2 + 2) {
                        LOG("invalid minimal BLE_WITE_REQ length\n");
                    invalid_apdu_packet:
                        G_io_apdu_state = APDU_IDLE;
                        G_io_apdu_offset = 0;
                        G_io_apdu_length = 0;
                        G_io_apdu_seq = 0;

                    send_last_command:
                        io_seproxyhal_general_status();

                        break;
                    }

                    // ensure this is an APDU command
                    if (G_io_seproxyhal_spi_buffer[6] != BLE_CMD_APDU) {
                        LOG("not an APDU BLE WRITE REQ\n");
                        goto invalid_apdu_packet;
                    }

                    // check sequence number is valid
                    if (U2(G_io_seproxyhal_spi_buffer[7],
                           G_io_seproxyhal_spi_buffer[8]) != 0) {
                        LOG("invalid first BLE WRITE REQ sequence number, "
                            "expected %d, received %d\n",
                            0, U2(G_io_seproxyhal_spi_buffer[7],
                                  G_io_seproxyhal_spi_buffer[8]));
                        goto invalid_apdu_packet;
                    }
                    G_io_apdu_seq = 0;

                    // retrieve the total apdu length to be received
                    G_io_apdu_length = U2(G_io_seproxyhal_spi_buffer[9],
                                          G_io_seproxyhal_spi_buffer[10]);

                    // copy the data to the apdu buffer
                    G_io_apdu_offset = U2(G_io_seproxyhal_spi_buffer[1],
                                          G_io_seproxyhal_spi_buffer[2]) -
                                       1 - 2 - 1 - 2 - 2;

                    // ensure data not overflowed
                    if (G_io_apdu_length < G_io_apdu_offset) {
                        LOG("received too much data for described APDU\n");
                        goto invalid_apdu_packet;
                    }

                    os_memmove(G_io_apdu_buffer,
                               G_io_seproxyhal_spi_buffer + 11,
                               G_io_apdu_offset);
                } else {
                    // TAG LENH LENL HNDL DLENH DLENL CMD SEQH SEQL APDUDATA

                    // ensure this is an APDU command
                    if (G_io_seproxyhal_spi_buffer[6] != BLE_CMD_APDU) {
                        LOG("not an APDU BLE WRITE REQ\n");
                        goto invalid_apdu_packet;
                    }

                    // check sequence number is valid
                    if (U2(G_io_seproxyhal_spi_buffer[7],
                           G_io_seproxyhal_spi_buffer[8]) != G_io_apdu_seq) {
                        LOG("invalid BLE WRITE REQ sequence number, expected "
                            "%d, received %d\n",
                            G_io_apdu_seq, U2(G_io_seproxyhal_spi_buffer[7],
                                              G_io_seproxyhal_spi_buffer[8]));
                        goto invalid_apdu_packet;
                    }

                    // copy data to the apdu buffer
                    os_memmove(G_io_apdu_buffer + G_io_apdu_offset,
                               G_io_seproxyhal_spi_buffer + 9,
                               U2(G_io_seproxyhal_spi_buffer[1],
                                  G_io_seproxyhal_spi_buffer[2]) -
                                   1 - 2 - 1 - 2);
                    G_io_apdu_offset += U2(G_io_seproxyhal_spi_buffer[1],
                                           G_io_seproxyhal_spi_buffer[2]) -
                                        1 - 2 - 1 - 2;

                    if (G_io_apdu_length < G_io_apdu_offset) {
                        LOG("received too much data for APDU\n");
                        goto invalid_apdu_packet;
                    }
                }
                G_io_apdu_seq++;
                G_io_apdu_state = APDU_BLE;

                // check if apdu has been completely received
                if (G_io_apdu_offset == G_io_apdu_length) {
                    // acknowledge the write request (general status OK) and
                    // more command to follow (processing of the apdu)
                    G_io_seproxyhal_spi_buffer[0] =
                        SEPROXYHAL_TAG_GENERAL_STATUS;
                    G_io_seproxyhal_spi_buffer[1] = 0;
                    G_io_seproxyhal_spi_buffer[2] = 2;
                    G_io_seproxyhal_spi_buffer[3] =
                        SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND >> 8;
                    G_io_seproxyhal_spi_buffer[4] =
                        SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND;
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 5);

                    G_io_apdu_media = IO_APDU_MEDIA_BLE;
                    return G_io_apdu_length;
                } else {
                    io_seproxyhal_general_status();

                    // wait next command (or timeout)
                }
                break;

            // event triggered upon write of the
            case SEPROXYHAL_TAG_M24SR_GPO_CHANGE_EVENT:
                // ensure ready to process a NFC APDU (or accept GPO change
                // during a NFC session)
                if (G_io_apdu_state < APDU_NFC_M24SR &&
                    G_io_apdu_state != APDU_IDLE) {
                    LOG("invalid state for NFC over M24SR\n");
                    // THROW(INVALID_STATE);
                    goto invalid_apdu_packet;
                }

                // only process when the GPO notifies of a valid NDEF length
                // being available
                if (G_io_apdu_state == APDU_IDLE &&
                    G_io_seproxyhal_spi_buffer[3] == 1) {
                    // issue an APDU to read the first block content (containing
                    // the total apdu length) (read as much as possible)
                    G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_M24SR_C_APDU;
                    G_io_seproxyhal_spi_buffer[1] = 0;
                    G_io_seproxyhal_spi_buffer[2] = 7;
                    // Forge NDEF Select command
                    G_io_seproxyhal_spi_buffer[3] = 0x00;
                    G_io_seproxyhal_spi_buffer[4] = 0xA4;
                    G_io_seproxyhal_spi_buffer[5] = 0x00;
                    G_io_seproxyhal_spi_buffer[6] = 0x0C;
                    G_io_seproxyhal_spi_buffer[7] = 0x02;
                    G_io_seproxyhal_spi_buffer[8] = 0x00;
                    G_io_seproxyhal_spi_buffer[9] = 0x01;

                    // wait for a NFC_RESPONSE
                    G_io_apdu_state = APDU_NFC_M24SR_SELECT;
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 10);
                } else {
                    // NDEF message rewritten while being updated ?? => error ?
                    LOG("unexpected GPO change\n");
                    if (!io_event(CHANNEL_SPI)) {
                        goto invalid_apdu_packet;
                    }
                }

                break;
            case SEPROXYHAL_TAG_M24SR_RESPONSE_APDU_EVENT:
                // ensure ready to process a NFC APDU (shall have been started
                // with GPO_CHANGE)
                switch (G_io_apdu_state) {
                case APDU_NFC_M24SR_SELECT:
                    // expect 0x90 0x00
                    if (G_io_seproxyhal_spi_buffer[1] != 0 ||
                        G_io_seproxyhal_spi_buffer[2] != 2 ||
                        G_io_seproxyhal_spi_buffer[3] != 0x90 ||
                        G_io_seproxyhal_spi_buffer[4] != 0x00) {
                        LOG("invalid NFC SELECT reply\n");
                        goto invalid_apdu_packet;
                    }

                    // read first part of the NDEF file, maximize throughput but
                    // to read only the total length

                    // issue an APDU to read the first block content (containing
                    // the total apdu length) (read as much as possible)
                    G_io_seproxyhal_spi_buffer[0] = SEPROXYHAL_TAG_M24SR_C_APDU;
                    G_io_seproxyhal_spi_buffer[1] = 0;
                    G_io_seproxyhal_spi_buffer[2] = 5;
                    // Forge NDEF Select command
                    G_io_seproxyhal_spi_buffer[3] = 0x00;
                    G_io_seproxyhal_spi_buffer[4] = 0xB0;
                    G_io_seproxyhal_spi_buffer[5] = 0x00;
                    G_io_seproxyhal_spi_buffer[6] = 0x00;
                    G_io_seproxyhal_spi_buffer[7] = M24SR_CHUNK_LENGTH;

                    // wait for a NFC_RESPONSE
                    G_io_apdu_state = APDU_NFC_M24SR_FIRST;
                    io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 8);
                    break;
                case APDU_NFC_M24SR_FIRST:
                    // ensure read has ended with a positive status
                    if (G_io_seproxyhal_spi_buffer[rx_len - 2] != 0x90 ||
                        G_io_seproxyhal_spi_buffer[rx_len - 1] != 0x00) {
                        LOG("invalid READBINARY reply\n");
                        goto invalid_apdu_packet;
                    }

                    G_io_apdu_length = U2(G_io_seproxyhal_spi_buffer[3],
                                          G_io_seproxyhal_spi_buffer[4]);
                    rx_len = MIN(G_io_apdu_length,
                                 rx_len - 3 /*TagLen*/ - 2 /*SW*/ -
                                     2 /*total apdu length (NDEF length)*/);

                    // copy the first part of the apdu
                    os_memmove(G_io_apdu_buffer,
                               G_io_seproxyhal_spi_buffer + 3 /*TagLen*/ +
                                   2 /*NDEF length*/,
                               rx_len);
                    G_io_apdu_offset = rx_len;

                    G_io_apdu_state = APDU_NFC_M24SR;
                    goto m24sr_continue_read_apdu;

                case APDU_NFC_M24SR:
                    // ensure read has ended with a positive status
                    if (G_io_seproxyhal_spi_buffer[rx_len - 2] != 0x90 ||
                        G_io_seproxyhal_spi_buffer[rx_len - 1] != 0x00) {
                        LOG("invalid READBINARY reply (2)\n");
                        goto invalid_apdu_packet;
                    }

                    rx_len = MIN(G_io_apdu_length - G_io_apdu_offset,
                                 rx_len - 3 /*TagLen*/ - 2 /*SW*/);

                    // copy the first part of the apdu
                    os_memmove(G_io_apdu_buffer + G_io_apdu_offset,
                               G_io_seproxyhal_spi_buffer + 3 /*TagLen*/,
                               rx_len);
                    G_io_apdu_offset += rx_len;

                m24sr_continue_read_apdu:
                    // check if need more data to complete the CAPDU content
                    if (G_io_apdu_length == G_io_apdu_offset) {
                        // acknowledge the M24SR RAPDU EVENT (general status OK)
                        // and more command to follow (processing of the apdu)
                        G_io_seproxyhal_spi_buffer[0] =
                            SEPROXYHAL_TAG_GENERAL_STATUS;
                        G_io_seproxyhal_spi_buffer[1] = 0;
                        G_io_seproxyhal_spi_buffer[2] = 2;
                        G_io_seproxyhal_spi_buffer[3] =
                            SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND >> 8;
                        G_io_seproxyhal_spi_buffer[4] =
                            SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND;
                        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 5);

                        G_io_apdu_media = IO_APDU_MEDIA_NFC;
                        return G_io_apdu_length;
                    } else {
                        // fetch more data from the M24SR memory
                        G_io_seproxyhal_spi_buffer[0] =
                            SEPROXYHAL_TAG_M24SR_C_APDU;
                        G_io_seproxyhal_spi_buffer[1] = 0;
                        G_io_seproxyhal_spi_buffer[2] = 5;
                        // Forge NDEF Select command
                        G_io_seproxyhal_spi_buffer[3] = 0x00;
                        G_io_seproxyhal_spi_buffer[4] = 0xB0;
                        G_io_seproxyhal_spi_buffer[5] =
                            (G_io_apdu_offset + 2 /*NDEF length*/) >> 8;
                        G_io_seproxyhal_spi_buffer[6] =
                            (G_io_apdu_offset + 2 /*NDEF length*/);
                        G_io_seproxyhal_spi_buffer[7] =
                            MIN(M24SR_CHUNK_LENGTH,
                                G_io_apdu_length - G_io_apdu_offset);
                        io_seproxyhal_spi_send(G_io_seproxyhal_spi_buffer, 8);
                    }
                    break;
                default:
                    LOG("invalid state for M24SR RAPDU EVENT\n");
                    goto invalid_apdu_packet;
                }
                break;
            default:
                // tell the application that a non-apdu packet has been received
                io_event(CHANNEL_SPI);
                break;
            }
        }
        break;

    default:
        return io_exchange_al(channel, tx_len);
    }
}

unsigned int os_ux_blocking(bolos_ux_params_t *params) {
    unsigned int ret;
    // until a real status is returned
    while (!(ret = os_ux(params))) {
        // process events
        for (;;) {
            unsigned int rx_len =
                io_seproxyhal_spi_recv(G_io_seproxyhal_spi_buffer,
                                       sizeof(G_io_seproxyhal_spi_buffer), 0);
            // if malformed, then a stall is likely to occur
            if (io_seproxyhal_handle_event()) {
                continue;
            }

            // TODO check here for

            // pass the packet to the ux
            break;
        }
        // prepare processing of the packet by the ux
        params->ux_id = BOLOS_UX_EVENT;
        params->len = 0;
    }

    return ret;
}

// so unoptimized
void screen_printc(unsigned char c) {
    unsigned char buf[4];
    buf[0] = SEPROXYHAL_TAG_PRINTF_STATUS;
    buf[1] = 0;
    buf[2] = 1;
    buf[3] = c;
    io_seproxyhal_spi_send(buf, 4);
#ifndef IO_SEPROXYHAL_DEBUG
    // wait printf ack (no race kthx)
    io_seproxyhal_spi_recv(buf, 3, 0);
    buf[0] = 0; // consume tag to avoid misinterpretation (due to IO_CACHE)
#endif          // IO_SEPROXYHAL_DEBUG
}

#endif // OS_IO_SEPROXYHAL
