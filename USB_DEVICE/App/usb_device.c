/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usb_device.c
 * @version        : v2.0_Cube
 * @brief          : This file implements the USB Device
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
#define CHARACTER_MODIFIER 96
typedef struct keyboard_HID_t
{
  uint8_t id;
  uint8_t modifiers;
  uint8_t key1;
  uint8_t key2;
  uint8_t key3;
} keyboard_HID_t;

void usb_keyboard_send_character(char character)
{
  keyboard_HID_t keyboard_report = {
      .id = 0,
      .modifiers = 0,
      .key1 = character - 96,
      .key2 = 0,
      .key3 = 0,
  };
  USBD_HID_SendReport(&hUsbDeviceFS, &keyboard_report, sizeof(keyboard_report));

  keyboard_HID_t keyboard_report_off = {
      .id = 0,
      .modifiers = 0,
      .key1 = 0,
      .key2 = 0,
      .key3 = 0,
  };
  USBD_HID_SendReport(&hUsbDeviceFS, &keyboard_report_off, sizeof(keyboard_report_off));
}

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * Init USB device Library, add supported class and start the library
 * @retval None
 */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  uint16_t length = 0;
  USBD_HID.GetFSConfigDescriptor(&length)[16] = 1;
  USBD_HID.GetHSConfigDescriptor(&length)[16] = 1;
  USBD_HID.GetOtherSpeedConfigDescriptor(&length)[16] = 1;
  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
 * @}
 */

/**
 * @}
 */
