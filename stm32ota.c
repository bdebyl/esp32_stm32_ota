/**
 * @file stm32ota.c
 * @author Bastian de Byl (bastian@debyl.io)
 * @brief
 * @version 0.1
 * @date 2025-03-25
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "stm32ota.h"

/**
 * @brief Internal function to XOR all bytes in an STM32_LoadAddress_t
 *
 * @param load_address
 * @return char XOR of all bytes in load_address
 */
static char _stm32_load_address_xor(STM32_LoadAddress_t *load_address) {
  return load_address->high_byte ^ load_address->mid_high_byte ^
         load_address->mid_low_byte ^ load_address->low_byte;
}

/**
 * @brief Internal function to await (blocking) response of expected size on
 * UART buffer
 *
 * @param stm32_ota
 * @param expected_size
 * @return esp_err_t
 */
static esp_err_t _stm32_await_rx(STM32_OTA_t *stm32_ota, size_t expected_size) {
  // Setup awaiting expected response size in the UART buffer prior to
  // continuing, this is a blocking call and could be improved later
  uint16_t timer = 0;
  size_t read_size = 0;

  // Loop to check against timeout or if we got the expected size
  esp_err_t uart_err = ESP_OK;
  while (timer < STM32_UART_TIMEOUT && read_size < expected_size) {
    uart_err = uart_get_buffered_data_len(stm32_ota->uart_port, &read_size);

    if (uart_err != ESP_OK) {
      return uart_err;
    }

    // TODO: Don't use vTaskDelay
    vTaskDelay(1 / portTICK_PERIOD_MS);
    timer++;
  }

  if (timer >= STM32_UART_TIMEOUT) {
    return ESP_ERR_TIMEOUT;
  }

  if (read_size < expected_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  return ESP_OK;
}

/**
 * @brief Internal function to write data over UART
 *
 * @param stm32_ota
 * @param write_bytes
 * @param write_size
 * @param response_size
 * @return esp_err_t
 */
static esp_err_t _stm32_write_bytes(STM32_OTA_t *stm32_ota,
                                    const char *write_bytes, size_t write_size,
                                    size_t response_size) {
  // Send initial UART data and ensure response size is what we expect
  const int bytes_written =
      uart_write_bytes(stm32_ota->uart_port, write_bytes, write_size);
  if (bytes_written != write_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  esp_err_t err = _stm32_await_rx(stm32_ota, response_size);
  if (err != ESP_OK) {
    return err;
  }

  // Prepare for reading back UART data for validity
  uint8_t *bytes_read = (uint8_t *)calloc(response_size, sizeof(uint8_t));

  // Read back data
  const int read_size = uart_read_bytes(stm32_ota->uart_port, bytes_read,
                                        read_size, 1000 / portTICK_PERIOD_MS);

  // Ensure we got an acknowledgement or there is more than 0 response data
  if (bytes_read[0] != STM32_UART_ACK || read_size == 0) {
    err = ESP_ERR_INVALID_RESPONSE;
  }

  free(bytes_read);

  return err;
}

esp_err_t stm32_init(STM32_OTA_t *stm32_ota) {
  // Check for null pointer
  if (stm32_ota == NULL)
    return ESP_ERR_INVALID_ARG;

  // Setup the UART driver
  STM32_ERROR_CHECK(uart_driver_install(
      stm32_ota->uart_port, stm32_ota->uart_rx_buffer_size,
      stm32_ota->uart_tx_buffer_size, stm32_ota->uart_queue_size,
      stm32_ota->uart_queue, stm32_ota->uart_intr_alloc_flags));

  STM32_ERROR_CHECK(
      uart_param_config(stm32_ota->uart_port, stm32_ota->uart_config));

  STM32_ERROR_CHECK(uart_set_pin(stm32_ota->uart_port, stm32_ota->stm_tx_pin,
                                 stm32_ota->stm_rx_pin, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

  // Setup GPIO pin for reset pin ~NRST
  STM32_ERROR_CHECK(
      gpio_set_direction(stm32_ota->stm_nrst_pin, GPIO_MODE_OUTPUT));

  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_nrst_pin, 1));

  // Setup GPIO pin for boot0 pin
  STM32_ERROR_CHECK(
      gpio_set_direction(stm32_ota->stm_boot0_pin, GPIO_MODE_OUTPUT));

  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_LOW));

  if (stm32_ota->disable_boot1_pin) {
    return ESP_OK;
  }

  // Setup GPIO pin for boot1 pin if used
  STM32_ERROR_CHECK(
      gpio_set_direction(stm32_ota->stm_boot1_pin, GPIO_MODE_OUTPUT));
  return gpio_set_level(stm32_ota->stm_boot1_pin, STM32_LOW);
}

esp_err_t stm32_reset(STM32_OTA_t *STM32_OTA_t) {
  // TODO: don't use vTaskDelay which is a blocking call
  STM32_ERROR_CHECK(gpio_set_level(STM32_OTA_t->stm_nrst_pin, STM32_LOW));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  STM32_ERROR_CHECK(gpio_set_level(STM32_OTA_t->stm_nrst_pin, STM32_HIGH));
  vTaskDelay(500 / portTICK_PERIOD_MS);

  return ESP_OK;
}

esp_err_t stm32_ota_begin(STM32_OTA_t *stm32_ota) {
  // Set GPIO levels to being flashing STM32
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_LOW));
  if (!stm32_ota->disable_boot1_pin)
    STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot1_pin, STM32_LOW));

  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_nrst_pin, STM32_LOW));

  // TODO: use non-blocking delay
  vTaskDelay(500 / portTICK_PERIOD_MS);

  /**
   * @brief For more information on commands below, see the following PDF:
   *
   * https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
   */
  // TODO: split out the sizes and response sizes to the header file as defines

  // 1. Initial bootloader command
  char cmd_bootloader[] = {0x7F};
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota->uart_port, cmd_sync, 1, 1));

  // 2. Run the Get command and validate we got expected response size
  // TODO: do something with response information
  char cmd_get[] = {0x00, 0xFF};
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota->uart_port, cmd_get, 2, 15));

  // 3. Run the Get Version command and validate we get expected response size
  // TODO: do something with response information
  char cmd_get_version[] = {0x01, 0xFE};
  STM32_ERROR_CHECK(
      _stm32_write_bytes(stm32_ota->uart_port, cmd_get_version, 2, 5));

  // 4. Run the Get ID command and validate we get expected response size
  // TODO: do something with response information
  char cmd_get_id[] = {0x02, 0xFD};
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota->uart_port, cmd_get_id, 2, 5));

  // 5.1. Erase all data on the STM32
  // TODO: do something with response information
  char cmd_erase[] = {0x43, 0xBC};
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota->uart_port, cmd_erase, 2, 1));

  // 5.2. Send a global erase once the erase has been acknowledged
  char cmd_erase_global[] = {0xFF, 0x00};
  STM32_ERROR_CHECK(
      _stm32_write_bytes(stm32_ota->uart_port, cmd_erase_global, 2, 1));

  // 6.1. Send extended erase (is this necessary?)
  char cmd_extended_erase[] = {0x44, 0xBB};
  STM32_ERROR_CHECK(
      _stm32_write_bytes(stm32_ota->uart_port, cmd_extended_erase, 2, 1));

  // 6.2. Send mass erase special instruction to extended erase with checksum
  char cmd_extended_erase_mass[] = {0xFF, 0xFF, 0x00};
  STM32_ERROR_CHECK(
      _stm32_write_bytes(stm32_ota->uart_port, cmd_extended_erase_mass, 3, 1));

  // Begin procedure is finished, we are ready to start writing to the STM32
  return ESP_OK;
}

esp_err_t stm32_ota_end(STM32_OTA_t *stm32_ota) {
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_HIGH));
  if (!stm32_ota->disable_boot1_pin)
    STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot1_pin, STM32_HIGH));

  return gpio_set_level(stm32_ota->stm_nrst_pin, STM32_HIGH);
}

esp_err_t stm32_ota_write_page(STM32_OTA_t *stm32_ota,
                               STM32_LoadAddress_t *load_address,
                               const char *ota_data, size_t ota_data_size) {
  if (ota_data_size > STM32_MAX_PAGE_SIZE) {
    return ESP_ERR_INVALID_SIZE;
  }

  char cmd_write[] = {0x31, 0xCE};
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota->uart_port, cmd_write, 2, 1));

  // Send the load address with the last byte being an XOR of all bytes combined
  char cmd_load_address[] = {
      load_address->high_byte,
      load_address->mid_high_byte,
      load_address->mid_low_byte,
      load_address->low_byte,
      _stm32_load_address_xor(load_address),
  };
  STM32_ERROR_CHECK(
      _stm32_write_bytes(stm32_ota->uart_port, cmd_load_address, 5, 1));

  // Send the size of the data to be written
  if (uart_write_bytes(stm32_ota->uart_port, &ota_data_size, 1) != 1) {
    return ESP_FAIL;
  }

  // Write the page to STM32 memory
  if (uart_write_bytes(stm32_ota->uart_port, ota_data, ota_data_size) !=
      ota_data_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  // Calculate the XOR of all bytes for the checksum and send it to the STM32
  char page_checksum = 0xFF;
  for (size_t i = 0; i < ota_data_size + 1; i++) {
    page_checksum ^= ota_data[i];
  }
  if (uart_write_bytes(stm32_ota->uart_port, &page_checksum, 1) != 1) {
    return ESP_ERR_INVALID_CRC;
  }

  // Await ACK from STM32
  esp_err_t err = _stm32_await_rx(stm32_ota->uart_port, 1);
  if (err != ESP_OK) {
    return err;
  }

  // Validate we received the ACK indicating successful memory write operation
  uint8_t response = 0x00;
  const int received_size =
      uart_read_bytes(stm32_ota->uart_port, &response, 1, STM32_UART_TIMEOUT);

  if (received_size == 0) {
    return ESP_ERR_TIMEOUT;
  }

  if (response != STM32_UART_ACK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_OK;
}
