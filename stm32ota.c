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
#include "esp_log.h"

/**
 * @brief Internal function to XOR all bytes in an stm32_loadaddress_t
 *
 * @param load_address
 * @return char XOR of all bytes in load_address
 */
static char _stm32_load_address_xor(stm32_loadaddress_t *load_address) {
  return load_address->high_byte ^ load_address->mid_high_byte ^ load_address->mid_low_byte ^ load_address->low_byte;
}

/**
 * @brief Internal function to await (blocking) response of expected size on
 * UART buffer
 *
 * @param stm32_ota
 * @param expected_size
 * @return esp_err_t
 */
static esp_err_t _stm32_await_rx(stm32_ota_t *stm32_ota, size_t expected_size) {
  // Setup awaiting expected response size in the UART buffer prior to
  // continuing, this is a blocking call and could be improved later
  uint32_t timer     = 0;
  size_t   read_size = 0;

  // Loop to check against timeout or if we got the expected size
  esp_err_t uart_err = ESP_OK;
  while (timer < STM32_UART_TIMEOUT && read_size < expected_size) {
    uart_err = uart_get_buffered_data_len(stm32_ota->uart_port, &read_size);

    if (uart_err != ESP_OK) {
      ESP_LOGE(STM32_TAG, "uart_get_buffered_data_len uart error code %d", uart_err);
      return uart_err;
    }

    // TODO: Don't use vTaskDelay
    vTaskDelay(1 / portTICK_PERIOD_MS);
    timer++;
  }

  if (timer >= STM32_UART_TIMEOUT) {
    ESP_LOGE(STM32_TAG, "uart_get_buffered_data_len timeout (read: %d, expected: %d)", read_size, expected_size);
    return ESP_ERR_TIMEOUT;
  }

  if (read_size < expected_size) {
    ESP_LOGE(STM32_TAG, "uart_get_buffered_data_len invalid size (read: %d, expected: %d)", read_size, expected_size);
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
static esp_err_t _stm32_write_bytes(stm32_ota_t *stm32_ota, const char *write_bytes, size_t write_size,
                                    size_t response_size) {
  // Prepare for reading back UART data for validity
  uint8_t bytes_read[response_size];
  STM32_ERROR_CHECK(uart_flush(stm32_ota->uart_port));

  // Send initial UART data and ensure response size is what we expect
  ESP_LOGD(STM32_TAG, "Write bytes size %d, expecting %d response bytes, write buff at %d (NULL: %d : Port: %d)",
           write_size, response_size, *write_bytes, write_bytes == NULL, stm32_ota->uart_port);
  int bytes_written = uart_write_bytes(stm32_ota->uart_port, write_bytes, write_size);
  if (bytes_written != write_size) {
    return ESP_ERR_INVALID_SIZE;
  }

  esp_err_t err = _stm32_await_rx(stm32_ota, response_size);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Error encountered when awaiting response from stm32 code %d", err);
    return err;
  }

  // Read back data
  const int read_size = uart_read_bytes(stm32_ota->uart_port, bytes_read, response_size, 1000 / portTICK_PERIOD_MS);

  // Ensure we got an acknowledgement or there is more than 0 response data
  if (bytes_read[0] != STM32_UART_ACK || read_size == 0) {
    ESP_LOGE(STM32_TAG, "Invalid response received during write read (ACK code: %02x, size: %d)", bytes_read[0],
             read_size);
    err = ESP_ERR_INVALID_RESPONSE;
  }

  return err;
}

esp_err_t stm32_init(stm32_ota_t *stm32_ota) {
  // Check for null pointer
  if (stm32_ota == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Setup the UART driver
  STM32_ERROR_CHECK(uart_driver_install(stm32_ota->uart_port, stm32_ota->uart_rx_buffer_size,
                                        stm32_ota->uart_tx_buffer_size, stm32_ota->uart_queue_size, NULL,
                                        stm32_ota->uart_intr_alloc_flags));

  STM32_ERROR_CHECK(uart_param_config(stm32_ota->uart_port, stm32_ota->uart_config));

  STM32_ERROR_CHECK(uart_set_pin(stm32_ota->uart_port, stm32_ota->stm_tx_pin, stm32_ota->stm_rx_pin, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

  // Setup GPIO pin for reset pin ~NRST
  STM32_ERROR_CHECK(gpio_set_direction(stm32_ota->stm_nrst_pin, GPIO_MODE_OUTPUT));

  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_nrst_pin, 1));

  // Setup GPIO pin for boot0 pin
  STM32_ERROR_CHECK(gpio_set_direction(stm32_ota->stm_boot0_pin, GPIO_MODE_OUTPUT));

  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_LOW));

  if (stm32_ota->disable_boot1_pin) {
    return ESP_OK;
  }

  // Setup GPIO pin for boot1 pin if used
  STM32_ERROR_CHECK(gpio_set_direction(stm32_ota->stm_boot1_pin, GPIO_MODE_OUTPUT));
  return gpio_set_level(stm32_ota->stm_boot1_pin, STM32_LOW);
}

esp_err_t stm32_reset(stm32_ota_t *stm32_ota_t) {
  ESP_LOGI(STM32_TAG, "resetting stm32...");
  // TODO: don't use vTaskDelay which is a blocking call
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota_t->stm_nrst_pin, STM32_LOW));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota_t->stm_nrst_pin, STM32_HIGH));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ESP_LOGI(STM32_TAG, "stm32 reset");

  return ESP_OK;
}

esp_err_t stm32_ota_begin(stm32_ota_t *stm32_ota) {
  // Set GPIO levels to being flashing STM32
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_HIGH));
  if (!stm32_ota->disable_boot1_pin) {
    STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot1_pin, STM32_LOW));
  }

  /**
   * @brief For more information on commands below, see the following PDF:
   *
   * https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
   */
  // TODO: split out the sizes and response sizes to the header file as defines
  STM32_ERROR_CHECK(stm32_reset(stm32_ota));

  // 1. Initial bootloader command
  char cmd_bootloader[] = {0x7F};
  ESP_LOGI(STM32_TAG, "INIT bootloader");
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_bootloader, 1, 1));
  ESP_LOGI(STM32_TAG, "INIT success");

  // 2. Run the Get command and validate we got expected response size
  // TODO: do something with response information
  char cmd_get[] = {0x00, 0xFF};
  ESP_LOGI(STM32_TAG, "GET request");
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_get, 2, 15));
  ESP_LOGI(STM32_TAG, "GET successful");

  // 3. Run the Get Version command and validate we get expected response size
  // TODO: do something with response information
  char cmd_get_version[] = {0x01, 0xFE};
  ESP_LOGI(STM32_TAG, "GET VERSION request");
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_get_version, 2, 5));
  ESP_LOGI(STM32_TAG, "GET VERSION success");

  // 4. Run the Get ID command and validate we get expected response size
  // TODO: do something with response information
  char cmd_get_id[] = {0x02, 0xFD};
  ESP_LOGI(STM32_TAG, "GET ID request");
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_get_id, 2, 5));
  ESP_LOGI(STM32_TAG, "GET ID success");

  // 5.0.1. Set STM32 to read unprotected (RPUN)
  // TODO: do something with response information
  char cmd_rpun[] = {0x92, 0x6D};  // wpun: {0x73, 0x8C};
  ESP_LOGI(STM32_TAG, "RPUN send");
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_rpun, 2, 2));
  ESP_LOGI(STM32_TAG, "RPUN success");
  vTaskDelay(500 / portTICK_PERIOD_MS);

  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_bootloader, 1, 1));

  // char cmd_wpun[] = {0x73, 0x8C};
  // ESP_LOGI(STM32_TAG, "sending bootloader write unprotect command");
  // STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_wpun, 2, 2));
  // ESP_LOGI(STM32_TAG, "bootloader write unprotect command successful");
  // vTaskDelay(200 / portTICK_PERIOD_MS);

  // STM32_ERROR_CHECK(stm32_reset(stm32_ota));
  //STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_bootloader, 1, 1));

  // 5.1. Erase all data on the STM32
  // TODO: do something with response information
  /* char cmd_erase[] = {0x43, 0xBC};
  ESP_LOGI(STM32_TAG, "ERASE send");
  if (_stm32_write_bytes(stm32_ota, cmd_erase, 2, 1) == ESP_OK) {
    ESP_LOGI(STM32_TAG, "ERASE success");

    // 5.2. Send a global erase once the erase has been acknowledged
    char cmd_erase_global[] = {0xFF, 0x00};
    ESP_LOGI(STM32_TAG, "ERASE GLOBAL send");
    if (_stm32_write_bytes(stm32_ota, cmd_erase_global, 2, 1) == ESP_OK) {
      ESP_LOGI(STM32_TAG, "ERASE GLOBAL success");
    } else {
      ESP_LOGE(STM32_TAG, "ERASE GLOBAL fail");

      return ESP_FAIL;
    }
  } else {
    ESP_LOGW(STM32_TAG, "ERASE fail");

    // 6.1. Send extended erase
    char cmd_extended_erase[] = {0x44, 0xBB};
    ESP_LOGI(STM32_TAG, "EXTENDED ERASE send");
    STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_extended_erase, 2, 1));
    ESP_LOGI(STM32_TAG, "EXTENDED ERASE success");

    // 6.2. Send mass erase special instruction to extended erase with checksum
    char cmd_extended_erase_mass[] = {0xFF, 0xFF, 0x00};
    ESP_LOGI(STM32_TAG, "MASS ERASE send");
    STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_extended_erase_mass, 3, 1));
    ESP_LOGI(STM32_TAG, "MASS ERASE success");
  } */

  // Begin procedure is finished, we are ready to start writing to the STM32
  return ESP_OK;
}

esp_err_t stm32_ota_end(stm32_ota_t *stm32_ota) {
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_LOW));
  if (!stm32_ota->disable_boot1_pin) {
    STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot1_pin, STM32_LOW));
  }

  return stm32_reset(stm32_ota);
}

esp_err_t stm32_ota_write_page(stm32_ota_t *stm32_ota, stm32_loadaddress_t *load_address, const char *ota_data,
                               size_t ota_data_size) {
  if (ota_data_size > STM32_MAX_PAGE_SIZE || ota_data_size == 0) {
    return ESP_ERR_INVALID_SIZE;
  }

  char cmd_write[] = {0x31, 0xCE};
  ESP_LOGI(STM32_TAG, "WRITE data");
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_write, 2, 1));

  // Send the load address with the last byte being an XOR of all bytes combined
  const char load_address_xor   = _stm32_load_address_xor(load_address);
  char       cmd_load_address[] = {
      load_address->high_byte, load_address->mid_high_byte, load_address->mid_low_byte, load_address->low_byte,
      load_address_xor,
  };
  ESP_LOGI(STM32_TAG, "LOAD ADDRESS %02x%02x%02x%02x (XOR: %02x)", load_address->high_byte,
           load_address->mid_high_byte, load_address->mid_low_byte, load_address->mid_low_byte, load_address_xor);
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_load_address, 5, 1));

  // Ensure we always send 1 to 256 bytes (0 < N <= 255)
  size_t ota_bytes_size = ota_data_size - 1;

  // Send the size of the data to be written
  if (uart_write_bytes(stm32_ota->uart_port, &ota_bytes_size, 1) != 1) {
    ESP_LOGE(STM32_TAG, "failed to write page size information (size: %d)", ota_bytes_size);
    return ESP_FAIL;
  }

  // Write the page to STM32 memory
  if (uart_write_bytes(stm32_ota->uart_port, ota_data, ota_data_size) != ota_data_size) {
    ESP_LOGE(STM32_TAG, "failed to write page (size: %d)", ota_data_size);
    return ESP_ERR_INVALID_SIZE;
  }

  // Calculate the XOR of all bytes for the checksum and send it to the STM32
  char page_checksum = ota_bytes_size;
  for (size_t i = 0; i < ota_data_size; i++) {
    page_checksum ^= ota_data[i];
  }
  if (uart_write_bytes(stm32_ota->uart_port, &page_checksum, 1) != 1) {
    ESP_LOGE(STM32_TAG, "failed to validate page checksum (%02x)", page_checksum);
    return ESP_ERR_INVALID_CRC;
  }

  // Await ACK from STM32
  STM32_ERROR_CHECK(_stm32_await_rx(stm32_ota, 1));

  // Validate we received the ACK indicating successful memory write operation
  uint8_t   response      = 0x00;
  const int received_size = uart_read_bytes(stm32_ota->uart_port, &response, 1, STM32_UART_TIMEOUT);

  if (received_size == 0) {
    ESP_LOGE(STM32_TAG, "timeout on writing page");
    return ESP_ERR_TIMEOUT;
  }

  if (response != STM32_UART_ACK) {
    ESP_LOGE(STM32_TAG, "ACK not received for writing page");
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_OK;
}
