/**
 * @file stm32ota.h
 * @author Bastian de Byl (bastian@debyl.io)
 * @brief
 * @version 0.1
 * @date 2025-03-25
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __stm32_ota_H
#define __stm32_ota_H

#define STM32_TAG           "STM32_OTA"

#define STM32_HIGH          1
#define STM32_LOW           0

#define STM32_UART_TIMEOUT  0x00FFFFFF  // ~16 seconds (for normal operations)
#define STM32_UART_TIMEOUT_EXTENDED  0x0FFFFFFF  // ~70 seconds (for mass erase)
#define STM32_UART_ACK      0x79
#define STM32_UART_NACK     0x1F  // Bootloader rejection (invalid cmd, checksum error, protection active, etc.)

#define STM32_MAX_PAGE_SIZE 0xFF

// AN3155 Rev 19 states the max baudrate of the UART bootloader is 115200 baud
#define STM32_MAX_BAUD_RATE 115200

// Read verification retry configuration
#define STM32_READ_VERIFY_RETRIES       3       // Number of retry attempts for read verification
#define STM32_RETRY_DELAY_1_MS          500     // First retry delay: 500ms
#define STM32_RETRY_DELAY_2_MS          2000    // Second retry delay: 2s
#define STM32_RETRY_DELAY_3_MS          4000    // Third retry delay: 4s

#define STM32_ERROR_CHECK(func)                                                                                        \
  {                                                                                                                    \
    esp_err_t err = func;                                                                                              \
    if (err != ESP_OK) {                                                                                               \
      return err;                                                                                                      \
    }                                                                                                                  \
  }

#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

typedef struct _stm32_ota_t {
  QueueHandle_t *uart_queue;          // Uart event queue handle, if set to NULL driver
                                      // will not use a queue (ex. NULL)
  gpio_num_t  stm_boot0_pin;          // ESP pin connected to BOOT0 of STM32
  gpio_num_t  stm_boot1_pin;          // ESP pin connected to BOOT1 of STM32
  gpio_num_t  stm_rx_pin;             // ESP pin connected to RX of STM32
  gpio_num_t  stm_tx_pin;             // ESP pin connected to TX of STM32
  gpio_num_t  stm_nrst_pin;           // ESP pin connected to ~NRST (Reset) of STM32
  int         uart_intr_alloc_flags;  // UART alloc flags (ex. 0)*/
  int         uart_queue_size;        // UART event queue size/depth (ex. 0)*/
  int         uart_rx_buffer_size;    // RX UART buffer size in bytes (ex. 0)
  int         uart_tx_buffer_size;    // TX UART buffer size in bytes (ex. 1024)
  int         uart_baudrate;          // UART config (115200 for bootloader)
  uart_port_t uart_port;              // UART port of the ESP to use
  uint8_t     disable_boot1_pin;      // Set to 1 if not using boot1_pin
  uint8_t     uart_externally_managed; // Set to 1 if UART is managed externally (shared with VCU protocol)
  uint8_t     skip_read_unprotect;    // Set to 1 to skip read unprotect, 0 to auto-detect/attempt (default: 0)
  uint8_t     skip_write_unprotect;   // Set to 1 to skip write unprotect, 0 to auto-detect/attempt (default: 0)
} stm32_ota_t;

typedef struct _stm32_loadaddress_t {
  char high_byte;
  char mid_high_byte;
  char mid_low_byte;
  char low_byte;
} stm32_loadaddress_t;

/**
 * @brief Adds the addition to the load address and handles the overflow
 *
 * @param stm32_ota
 * @param addition
 * @return esp_err_t
 */
esp_err_t stm32_increment_loadaddress(stm32_loadaddress_t *stm32_loadaddress, size_t addition);

/**
 * @brief Initialize the instance of the STM32 OTA for the given parameters set
 * in the struct members
 *
 * @param stm32_ota_t Pointer to the stm32_ota_t struct
 * @return esp_err_t
 */
esp_err_t stm32_init(stm32_ota_t *stm32_ota);
/**
 * @brief Reset the STM32 by flipping the BOOTn and ~NRST pins to the 'running'
 * values
 *
 * @param stm32_ota
 * @return esp_err_t
 */
esp_err_t stm32_reset(stm32_ota_t *stm32_ota);
/**
 * @brief Set the STM32 up to begin a full flash.
 *
 * Note that this erases all flash memory on the STM32! Only call this function
 * if beginning to flash a new program.
 *
 * @param stm32_ota
 * @return esp_err_t
 */
esp_err_t stm32_ota_begin(stm32_ota_t *stm32_ota);
/**
 * @brief Ends the STM32 OTA flash session, and reset the STM32 to the running
 * values
 *
 * @param stm32_ota
 * @return esp_err_t
 */
esp_err_t stm32_ota_end(stm32_ota_t *stm32_ota);
/**
 * @brief Writes a page of byte data to the STM32 without read verification.
 *
 * Use this when read protection prevents verification or when speed is critical.
 * stm32_ota_begin must be called before writing a page!
 *
 * @param stm32_ota
 * @param load_address
 * @param ota_data
 * @param ota_data_size
 * @return esp_err_t
 */
esp_err_t stm32_ota_write_page(stm32_ota_t *stm32_ota, stm32_loadaddress_t *load_address, const char *ota_data,
                               size_t ota_data_size);

/**
 * @brief Writes a page of byte data to the STM32 with read-back verification.
 *
 * Falls back to unverified write if read protection prevents verification.
 * stm32_ota_begin must be called before writing a page!
 *
 * @param stm32_ota
 * @param load_address
 * @param ota_data
 * @param ota_data_size
 * @return esp_err_t
 */
esp_err_t stm32_ota_write_page_verified(stm32_ota_t *stm32_ota, stm32_loadaddress_t *load_address, const char *ota_data,
                                        size_t ota_data_size);

#endif  //  __stm32_ota_H
