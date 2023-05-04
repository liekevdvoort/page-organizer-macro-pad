#pragma once

// RHPort number used for device can be defined by board.mk, default to port 0
#define BOARD_TUD_RHPORT 0
// RHPort max operational speed can defined by board.mk
#define BOARD_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED

#define CFG_TUSB_MCU OPT_MCU_STM32F0
#define CFG_TUSB_OS OPT_OS_NONE

// Enable usb device stack (instead of the host stack)
#define CFG_TUD_ENABLED 1
#define CFG_TID_MAX_SPEED BOARD_TUD_MAX_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
// for now this is not the case for the STM32F07
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

//--------------------------------------------------------------------
// USB DEVICE CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENDPOINT0_SIZE 64
//------------- CLASS -------------//
#define CFG_TUD_HID 1
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// HID buffer size Should be sufficient to hold ID (if any) + Data
#define CFG_TUD_HID_EP_BUFSIZE 64
