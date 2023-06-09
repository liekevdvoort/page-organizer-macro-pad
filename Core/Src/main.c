/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "tusb.h"
#include "usb_descriptors.h"
#include "debounce.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_BUTTONS 2
#define AXIS_MAX_VALUE 4028 / 2
#define AXIS_MIN_VALUE 0
#define AXIS_THRESHOLD AXIS_MAX_VALUE - 400
#define AXIS_RETURN_THRESHOLD AXIS_THRESHOLD - 200
#define QUEUE_LENGTH 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t keyboard_report[6] = {0, 0, 0, 0, 0, 0};
// For a more in depth example visit:
// https://github.com/hathach/tinyusb/blob/master/examples/device/hid_composite/src/main.c

// OLD HID SEND KEYCODE (part of the code)
// static void hid_send_keycode(int keycode, bool active)
//   // second parameter is keyboard modifier. Can be something like:
//   // KEYBOARD_MODIFIER_LEFTCTRL
//   // KEYBOARD_MODIFIER_LEFTSHIFT
//   // KEYBOARD_MODIFIER_LEFTALT
//   // KEYBOARD_MODIFIER_LEFTGUI
//   // KEYBOARD_MODIFIER_RIGHTCTRL
//   // KEYBOARD_MODIFIER_RIGHTSHIFT
//   // KEYBOARD_MODIFIER_RIGHTALT
//   // KEYBOARD_MODIFIER_RIGHTGUI

//   // EXAMPLE FROM JORT about keyboard shortcuts __________________________
//   // uint8_t macro_a_report[8] = {
//   //     HID_KEY_2,
//   //     HID_KEY_5,
//   //     HID_KEY_A,
//   //     0, 0, 0};
//   // uint8_t macro_a_modifiers = KEYBOARD_MODIFIER_LEFTGUI | KEYBOARD_MODIFIER_LEFTSHIFT;
//   // tud_hid_keyboard_report(REPORT_ID_KEYBOARD, macro_a_modifiers, macro_a_report);
//   // END EXAMPLE FROM JORT __________________________
// }

static bool report_complete = true;
uint32_t report_send_time = 0;
// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
  (void)instance;
  (void)len;

  // uint8_t next_report_id = report[0] + 1u;

  // if (next_report_id < REPORT_ID_COUNT) {
  //   // send_hid_report(next_report_id, board_button_read());
  // }
  report_complete = true;
}
static void hid_send_keycode(uint8_t keycode, uint8_t modifiers, bool active)
{
  // skip if HID is not ready
  if (!tud_hid_ready())
    return;

  const int keyboard_report_size = (sizeof(keyboard_report) / sizeof(uint8_t));
  int keycode_index = -1;
  int first_empty_keycode_index = -1;

  for (int report_index = 0; report_index < keyboard_report_size; report_index++)
  {
    if (keyboard_report[report_index] == 0 && first_empty_keycode_index == -1)
    {
      first_empty_keycode_index = report_index;
    }
    if (keyboard_report[report_index] == keycode)
    {
      keycode_index = report_index;
    }
  }
  // button is on and should be sent
  if (active)
  {
    if (keycode_index != -1)
    {
      // already pressed
      return;
    }
    else
    {
      // add it to the first empty location
      keyboard_report[first_empty_keycode_index] = keycode;
    }
  }
  else
  {
    // inactive
    if (keycode_index != -1)
    {
      keyboard_report[keycode_index] = 0;
    }
  }

  if (!active)
  {
    modifiers = 0;
  }
  report_send_time = HAL_GetTick();
  report_complete = false;
  tud_hid_keyboard_report(REPORT_ID_KEYBOARD, modifiers, keyboard_report);
}

void wait_for_report_to_complete(uint32_t timeout)
{
  while (!report_complete && HAL_GetTick() - report_send_time <= timeout)
  {
    tud_task();
  }
}

void button_pressed(bool isPressed, uint8_t keycode, uint8_t modifiers)
{
  printf("keycode %d modifiers %d", keycode, modifiers);

  hid_send_keycode(keycode, modifiers, isPressed);
  // queue_keycode(keycode,  modifiers, isPressed);
}

// // CODE JORT __________________________________________
typedef enum joystick_direction_t
{
  JOYSTICK_TOP = 0,
  JOYSTICK_RIGHT_TOP,
  JOYSTICK_RIGHT,
  JOYSTICK_RIGHT_BOTTOM,
  JOYSTICK_BOTTOM,
  JOYSTICK_LEFT_BOTTOM,
  JOYSTICK_LEFT,
  JOYSTICK_LEFT_TOP,
} joystick_direction_t;

typedef enum joystick_mode_t
{
  JOYSTICK_MODE_MAGNET = 0,
  JOYSTICK_MODE_DESKTOPS,
  JOYSTICK_MODE_ARROW_KEYS,
  JOYSTICK_MODE_SCROLL,
} joystick_mode_t;

typedef struct keyboard_report_t
{
  uint8_t report[8];
  uint8_t modifier;
} keyboard_report_t;

keyboard_report_t mode_specific_commands[4][8] = {
    // modus 1 arrow keys
    {(keyboard_report_t){
         .report = {HID_KEY_ARROW_UP},
         .modifier = 0,
     },
     (keyboard_report_t){
         .report = {HID_KEY_ARROW_RIGHT},
         .modifier = 0,
     }},
    // modus 2 scroll
    {},
    // modus 3 magnet controls
    {},
    // modus 4 switch desktop
    {},

};

void send_joystick(joystick_mode_t mode, joystick_direction_t direction)
{
  printf("sendJoystick mode direction: %lu, %lu", mode, direction);
  // send this
  // mode_specific_commands[mode][direction]

  // copied from above
  // tud_hid_keyboard_report(REPORT_ID_KEYBOARD, macro_a_modifiers, macro_a_report);
}
// END CODE JORT __________________________________________

typedef struct button_definition_t
{
  GPIO_TypeDef *port;
  uint16_t pin;
  bool previous_button_state;
  debounce_t debounce;
  uint8_t keycode;
  uint8_t modifiers;
} button_definition_t;

button_definition_t buttons[NUMBER_OF_BUTTONS] = {
    {
        // Maximize
        .port = GPIO_INPUT_SWITCH_GPIO_Port,
        .pin = GPIO_INPUT_SWITCH_Pin,
        .previous_button_state = false,
        .debounce = {0},
        // .keycode = HID_KEY_BACKSPACE,
        // .modifiers = KEYBOARD_MODIFIER_LEFTALT | KEYBOARD_MODIFIER_LEFTCTRL,
    },
    {
        // Minimize
        .port = GPIO_IN_JOYSTICK_SEL_GPIO_Port,
        .pin = GPIO_IN_JOYSTICK_SEL_Pin,
        .previous_button_state = false,
        .debounce = {0},
        .keycode = HID_KEY_ENTER,
        .modifiers = KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT,
    }};

void initialize_buttons()
{
  for (uint32_t button_index = 0; button_index < NUMBER_OF_BUTTONS; button_index++)
  {
    buttons[button_index].debounce = debounce_init(HAL_GetTick, false, 5, NULL);
  }
}

int modus = 0;

void update_modus()
{
  if (modus < 2)
  {
    modus = modus + 1;
  }
  else
  {
    modus = 0;
  }
}

void buttons_check()
{
  for (uint32_t button_index = 0; button_index < NUMBER_OF_BUTTONS; button_index++)
  {
    button_definition_t current_button = buttons[button_index];

    // True if button is pressed, False if button is released
    bool current_button_state = debounce_update(&buttons[button_index].debounce, !HAL_GPIO_ReadPin(current_button.port, current_button.pin));

    if (current_button_state != current_button.previous_button_state)
    {
      if (current_button.pin == GPIO_INPUT_SWITCH_Pin && current_button_state)
      {
        update_modus();
      }
      // We got a change on the key
      button_pressed(current_button_state, current_button.keycode, current_button.modifiers);
      buttons[button_index].previous_button_state = current_button_state;
    }
  }
}

uint32_t joystick_axis[2] = {0};
void read_joystick()
{
  // configure channel one and start conversion
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_PollForConversion(&hadc, 1000) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Stop(&hadc);
  joystick_axis[0] = HAL_ADC_GetValue(&hadc);

  // read out seconds axis
  // set channel 1 and disable channel 0
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_PollForConversion(&hadc, 1000) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Stop(&hadc);
  joystick_axis[1] = HAL_ADC_GetValue(&hadc);

  // disable channel 1
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  printf("axis values: %lu, %lu", joystick_axis[0], joystick_axis[1]);
  axes_check(joystick_axis[0], joystick_axis[1]);
}

typedef struct axes_position_definition_t
{
  bool left;
  bool right;
  bool top;
  bool bottom;
} axes_position_definition_t;

axes_position_definition_t axes_position = {
    .left = false,
    .right = false,
    .top = false,
    .bottom = false,
};

bool check_axis_position(int16_t value, bool *previous_state)
{
  // Based on the previous state, use a different threshold to avoid overtriggering.
  if (*previous_state)
  {
    if (value < AXIS_RETURN_THRESHOLD)
    {
      *previous_state = false;
      return true;
    }
  }
  else
  {
    if (value > AXIS_THRESHOLD)
    {
      *previous_state = true;
      return true;
    }
  }
  return false;
}

void axes_check(uint32_t axis_horizontal, uint32_t axis_vertical)
{
  printf("axis values: %lu, %lu", joystick_axis[0], joystick_axis[1]);
  int16_t calculated_vertical_axis = axis_vertical - AXIS_MAX_VALUE;
  int16_t calculated_horizontal_axis = axis_horizontal - AXIS_MAX_VALUE;

  bool has_update = false;

  // Horizontal
  if (check_axis_position(calculated_horizontal_axis * -1, &axes_position.left))
  {
    has_update = true;
  }
  if (check_axis_position(calculated_horizontal_axis, &axes_position.right))
  {
    has_update = true;
  }

  // Vertical
  if (check_axis_position(calculated_vertical_axis, &axes_position.top))
  {
    has_update = true;
  }
  if (check_axis_position(calculated_vertical_axis * -1, &axes_position.bottom))
  {
    has_update = true;
  }

  // If one of the computed values updated (so one of the thresholds is passed), do an update.
  if (has_update)
  {
    axes_changed(axes_position);
  }
}

void axes_changed(axes_position_definition_t position)
{
  // Define what keycombination needs to be sent based on the direction of the joystick.
  uint8_t keycode = 0;
  uint8_t modifiers = 0;
  if (!position.left && !position.right && !position.top && !position.bottom)
  {
    return;
  }
  if (position.left)
  {
    if (modus == 0)
    {
      keycode = HID_KEY_ARROW_LEFT;
      modifiers = KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT;
    }
    if (modus == 1)
    {
      keycode = HID_KEY_ARROW_LEFT;
      modifiers = KEYBOARD_MODIFIER_LEFTCTRL;
    }
    if (modus == 2)
    {
      keycode = HID_KEY_ARROW_LEFT;
    }
  }
  if (position.right)
  {
    if (modus == 0)
    {
      keycode = HID_KEY_ARROW_RIGHT;
      modifiers = KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT;
    }
    if (modus == 1)
    {
      keycode = HID_KEY_ARROW_RIGHT;
      modifiers = KEYBOARD_MODIFIER_LEFTCTRL;
    }
    if (modus == 2)
    {
      keycode = HID_KEY_ARROW_RIGHT;
    }
  }
  if (position.top)
  {
    if (modus == 0)
    {
      keycode = HID_KEY_ARROW_UP;
      modifiers = KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT;
    }
    if (modus == 2)
    {
      keycode = HID_KEY_ARROW_UP;
    }
  }
  if (position.bottom)
  {
    if (modus == 0)
    {
      keycode = HID_KEY_ARROW_DOWN;
      modifiers = KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT;
    }
    if (modus == 2)
    {
      keycode = HID_KEY_ARROW_DOWN;
    }
  }

  // Send the keycombination, and directly turn it off to avoid retriggering.
  hid_send_keycode(keycode, modifiers, true);
  wait_for_report_to_complete(100);
  hid_send_keycode(keycode, modifiers, false);

  // queue_keycode(keycode,  modifiers, true);
  // queue_keycode(keycode,  modifiers, false);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  tud_init(BOARD_TUD_RHPORT);
  initialize_buttons();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    buttons_check();
    read_joystick();
    tud_task();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
