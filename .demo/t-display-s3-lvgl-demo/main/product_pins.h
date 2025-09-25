/**
 * @file      product_pins.h
 * @author    Baciu Aurel Florin
 * @brief     Product specific pin definitions for the Lilygo T HMI development board.
 * @license   MIT
 * @copyright Copyright (c) 2025 Baciu Aurel Florin
 * @date      2025-07-01
 *
 */

#pragma once

#include <sdkconfig.h>

#define BOARD_NONE_PIN (-1)

#define BOARD_BUTTON_BOOT (0)
#define BOARD_BUTTON (14)

#define BOARD_POWERON (gpio_num_t)(15) // LCD and battery Power Enable

#define BOARD_TFT_BL (38) // LCD and battery Power Enable
#define BOARD_TFT_DATA0 (39)
#define BOARD_TFT_DATA1 (40)
#define BOARD_TFT_DATA2 (41)
#define BOARD_TFT_DATA3 (42)
#define BOARD_TFT_DATA4 (45)
#define BOARD_TFT_DATA5 (46)
#define BOARD_TFT_DATA6 (47)
#define BOARD_TFT_DATA7 (48)
#define BOARD_TFT_CS (6)
#define BOARD_TFT_DC (7)
#define BOARD_TFT_WR (8)
#define BOARD_TFT_RD (9)
#define BOARD_TFT_RST (5)

#define AMOLED_WIDTH (170)
#define AMOLED_HEIGHT (320)

