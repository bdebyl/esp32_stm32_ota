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
 * @param timeout Maximum timeout iterations (use STM32_UART_TIMEOUT or STM32_UART_TIMEOUT_EXTENDED)
 * @return esp_err_t
 */
static esp_err_t _stm32_await_rx(stm32_ota_t *stm32_ota, size_t expected_size, uint32_t timeout) {
  // Setup awaiting expected response size in the UART buffer prior to
  // continuing, this is a blocking call and could be improved later
  uint32_t timer     = 0;
  size_t   read_size = 0;

  // Loop to check against timeout or if we got the expected size
  esp_err_t uart_err = ESP_OK;
  while (timer < timeout && read_size < expected_size) {
    uart_err = uart_get_buffered_data_len(stm32_ota->uart_port, &read_size);

    if (uart_err != ESP_OK) {
      ESP_LOGE(STM32_TAG, "uart_get_buffered_data_len uart error code %d", uart_err);
      return uart_err;
    }

    // Yield to scheduler with 10ms delay to allow IDLE task to run
    // This prevents starving the IDLE task watchdog even when called from high-priority tasks
    // (e.g., btController). The 10ms delay gives the scheduler enough time to preempt to
    // lower-priority tasks like IDLE, which must run periodically to reset its watchdog.
    vTaskDelay(10 / portTICK_PERIOD_MS);
    timer += 10;  // Increment by delay amount to maintain timeout accuracy
  }

  if (timer >= timeout) {
    ESP_LOGE(STM32_TAG, "uart_get_buffered_data_len timeout (read: %d, expected: %d, timer: %lu)", read_size, expected_size, timer);
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
 * @param timeout Maximum timeout iterations (use STM32_UART_TIMEOUT or STM32_UART_TIMEOUT_EXTENDED)
 * @return esp_err_t
 */
static esp_err_t _stm32_write_bytes(stm32_ota_t *stm32_ota, const char *write_bytes, size_t write_size,
                                    size_t response_size, uint32_t timeout) {
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

  esp_err_t err = _stm32_await_rx(stm32_ota, response_size, timeout);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Error encountered when awaiting response from stm32 code %d", err);
    return err;
  }

  // Read back data
  const int read_size = uart_read_bytes(stm32_ota->uart_port, bytes_read, response_size, 1000 / portTICK_PERIOD_MS);

  // Ensure we got an acknowledgement or there is more than 0 response data
  if (read_size == 0) {
    ESP_LOGE(STM32_TAG, "No response received (timeout or empty buffer)");
    err = ESP_ERR_INVALID_RESPONSE;
  } else if (bytes_read[0] == STM32_UART_NACK) {
    // NACK (0x1F) - bootloader rejected the command
    ESP_LOGE(STM32_TAG, "NACK (0x1F) received - bootloader rejected command (possible reasons: "
             "invalid command, checksum error, address out of range, protection active, malformed packet)");
    err = ESP_ERR_INVALID_RESPONSE;
  } else if (bytes_read[0] != STM32_UART_ACK) {
    // Unexpected response byte (neither ACK nor NACK)
    ESP_LOGE(STM32_TAG, "Unexpected response byte: 0x%02x (expected ACK 0x79 or NACK 0x1F)", bytes_read[0]);
    err = ESP_ERR_INVALID_RESPONSE;
  }

  return err;
}

esp_err_t stm32_increment_loadaddress(stm32_loadaddress_t *stm32_loadaddress, size_t addition) {
  uint32_t full_address = (stm32_loadaddress->high_byte << 24) | (stm32_loadaddress->mid_high_byte << 16) |
                          (stm32_loadaddress->mid_low_byte << 8) | stm32_loadaddress->low_byte;

  full_address += addition;

  if (full_address > 0x080FFFFF) {
    return ESP_ERR_INVALID_SIZE;
  }

  stm32_loadaddress->high_byte     = (full_address >> 24) & 0xFF;
  stm32_loadaddress->mid_high_byte = (full_address >> 16) & 0xFF;
  stm32_loadaddress->mid_low_byte  = (full_address >> 8) & 0xFF;
  stm32_loadaddress->low_byte      = (full_address) & 0xFF;

  return ESP_OK;
}

esp_err_t stm32_init(stm32_ota_t *stm32_ota) {
  // Check for null pointer
  if (stm32_ota == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Only initialize UART if not externally managed
  if (!stm32_ota->uart_externally_managed) {
    // Check for maximum values
    if (stm32_ota->uart_baudrate > STM32_MAX_BAUD_RATE) {
      return ESP_ERR_INVALID_ARG;
    }

    // Setup the UART driver
    STM32_ERROR_CHECK(uart_driver_install(stm32_ota->uart_port, stm32_ota->uart_rx_buffer_size,
                                          stm32_ota->uart_tx_buffer_size, stm32_ota->uart_queue_size, NULL,
                                          stm32_ota->uart_intr_alloc_flags));

    uart_config_t stm32_uart_config = {
        .baud_rate = stm32_ota->uart_baudrate,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
    };

    STM32_ERROR_CHECK(uart_param_config(stm32_ota->uart_port, &stm32_uart_config));

    STM32_ERROR_CHECK(uart_set_pin(stm32_ota->uart_port, stm32_ota->stm_tx_pin, stm32_ota->stm_rx_pin, UART_PIN_NO_CHANGE,
                                   UART_PIN_NO_CHANGE));

    ESP_LOGI(STM32_TAG, "UART initialized by STM32 OTA component (standalone mode)");
  } else {
    ESP_LOGI(STM32_TAG, "UART externally managed (shared with VCU protocol)");
  }

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
  ESP_LOGI(STM32_TAG, "Sending initial bootloader sync command (0x7F)");
  char cmd_bootloader[] = {0x7F};
  esp_err_t err = _stm32_write_bytes(stm32_ota, cmd_bootloader, 1, 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to sync with STM32 bootloader: %s", esp_err_to_name(err));
    return err;
  }

  // 2. Run the Get command and validate we got expected response size
  ESP_LOGI(STM32_TAG, "Sending Get command (0x00)");
  char cmd_get[] = {0x00, 0xFF};
  err = _stm32_write_bytes(stm32_ota, cmd_get, 2, 15, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to execute Get command: %s", esp_err_to_name(err));
    return err;
  }

  // 3. Run the Get Version command and validate we get expected response size
  ESP_LOGI(STM32_TAG, "Sending Get Version command (0x01)");
  char cmd_get_version[] = {0x01, 0xFE};
  err = _stm32_write_bytes(stm32_ota, cmd_get_version, 2, 5, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to get bootloader version: %s", esp_err_to_name(err));
    return err;
  }

  // 4. Run the Get ID command and validate we get expected response size
  ESP_LOGI(STM32_TAG, "Sending Get ID command (0x02)");
  char cmd_get_id[] = {0x02, 0xFD};
  err = _stm32_write_bytes(stm32_ota, cmd_get_id, 2, 5, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to get chip ID: %s", esp_err_to_name(err));
    return err;
  }

  // 5. Disable write protection (required for flash write operations)
  // Send Write Unprotect (0x73) - if protection enabled, will trigger system reset
  ESP_LOGI(STM32_TAG, "Sending Write Unprotect command (0x73)");
  char cmd_write_unprotect[] = {0x73, 0x8C};
  err = _stm32_write_bytes(stm32_ota, cmd_write_unprotect, 2, 1, STM32_UART_TIMEOUT);
  if (err == ESP_OK) {
    // ACK received - protection was active, chip will reset now
    ESP_LOGI(STM32_TAG, "Write protection disabled - chip resetting (waiting 500ms)");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Re-sync bootloader after automatic reset
    ESP_LOGI(STM32_TAG, "Re-syncing bootloader after write unprotect reset");
    STM32_ERROR_CHECK(stm32_reset(stm32_ota));
    char cmd_bootloader_resync[] = {0x7F};
    STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_bootloader_resync, 1, 1, STM32_UART_TIMEOUT));
  } else {
    // NACK or timeout - either already unprotected (good) or command unsupported (acceptable)
    ESP_LOGI(STM32_TAG, "Write Unprotect returned error (likely already unprotected): %s", esp_err_to_name(err));
    // Continue - this is expected if flash is already writable
  }

  // 6. Disable read protection (required for READ command during write verification)
  // Send Read Unprotect (0x92) - if protection enabled, will mass erase and reset
  ESP_LOGI(STM32_TAG, "Sending Read Unprotect command (0x92)");
  char cmd_read_unprotect[] = {0x92, 0x6D};
  err = _stm32_write_bytes(stm32_ota, cmd_read_unprotect, 2, 1, STM32_UART_TIMEOUT);
  if (err == ESP_OK) {
    // ACK received - protection was active, chip will mass erase + reset now
    ESP_LOGI(STM32_TAG, "Read protection disabled - chip mass erasing + resetting (waiting 2s)");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Mass erase takes longer than simple reset

    // Re-sync bootloader after automatic reset
    ESP_LOGI(STM32_TAG, "Re-syncing bootloader after read unprotect reset");
    STM32_ERROR_CHECK(stm32_reset(stm32_ota));
    char cmd_bootloader_resync[] = {0x7F};
    STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_bootloader_resync, 1, 1, STM32_UART_TIMEOUT));
  } else {
    // NACK or timeout - either already unprotected (good) or command unsupported (acceptable)
    ESP_LOGI(STM32_TAG, "Read Unprotect returned error (likely already unprotected): %s", esp_err_to_name(err));
    // Continue - this is expected if flash is already readable
  }

  // 7. Erase flash memory - try regular erase first, fall back to extended erase
  ESP_LOGI(STM32_TAG, "Sending Erase command (0x43) - this may take several seconds");
  char cmd_erase[] = {0x43, 0xBC};
  err = _stm32_write_bytes(stm32_ota, cmd_erase, 2, 1, STM32_UART_TIMEOUT);
  if (err == ESP_OK) {
    // Regular erase is supported, send global erase command
    ESP_LOGI(STM32_TAG, "Sending global erase command (may take up to 60 seconds)");
    char cmd_erase_global[] = {0xFF, 0x00};
    err = _stm32_write_bytes(stm32_ota, cmd_erase_global, 2, 1, STM32_UART_TIMEOUT_EXTENDED);
    if (err != ESP_OK) {
      ESP_LOGE(STM32_TAG, "Failed to execute global erase: %s", esp_err_to_name(err));
      return err;
    }
    ESP_LOGI(STM32_TAG, "Global erase completed successfully");
  } else {
    // Regular erase not supported, try extended erase (for chips with >256 pages)
    ESP_LOGI(STM32_TAG, "Regular erase not supported, trying Extended Erase command (0x44)");
    char cmd_erase_ext[] = {0x44, 0xBB};
    err = _stm32_write_bytes(stm32_ota, cmd_erase_ext, 2, 1, STM32_UART_TIMEOUT);
    if (err != ESP_OK) {
      ESP_LOGE(STM32_TAG, "Failed to initiate extended erase: %s", esp_err_to_name(err));
      return err;
    }

    // Send mass erase instruction (may take up to 60 seconds for large flash)
    ESP_LOGI(STM32_TAG, "Sending mass erase instruction (may take up to 60 seconds)");
    char cmd_erase_ext_mass[] = {0xFF, 0xFF, 0x00};
    err = _stm32_write_bytes(stm32_ota, cmd_erase_ext_mass, 3, 1, STM32_UART_TIMEOUT_EXTENDED);
    if (err != ESP_OK) {
      ESP_LOGE(STM32_TAG, "Failed to execute mass erase: %s", esp_err_to_name(err));
      return err;
    }
    ESP_LOGI(STM32_TAG, "Extended mass erase completed successfully");
  }

  ESP_LOGI(STM32_TAG, "STM32 bootloader initialization complete, ready for firmware write");
  return ESP_OK;
}

esp_err_t stm32_ota_end(stm32_ota_t *stm32_ota) {
  STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot0_pin, STM32_LOW));
  if (!stm32_ota->disable_boot1_pin) {
    STM32_ERROR_CHECK(gpio_set_level(stm32_ota->stm_boot1_pin, STM32_LOW));
  }

  return stm32_reset(stm32_ota);
}

esp_err_t stm32_ota_write_page_verified(stm32_ota_t *stm32_ota, stm32_loadaddress_t *load_address, const char *ota_data,
                                        size_t ota_data_size) {
  if (ota_data_size > STM32_MAX_PAGE_SIZE || ota_data_size == 0 || ota_data_size % 4 != 0) {
    ESP_LOGE(STM32_TAG, "Invalid page size: %d bytes (max: %d, must be multiple of 4)", ota_data_size,
             STM32_MAX_PAGE_SIZE);
    return ESP_ERR_INVALID_SIZE;
  }

  // Calculate flash address for error reporting
  uint32_t flash_addr = (load_address->high_byte << 24) | (load_address->mid_high_byte << 16) |
                        (load_address->mid_low_byte << 8) | load_address->low_byte;

  ESP_LOGD(STM32_TAG, "Writing %d bytes to flash address 0x%08lX", ota_data_size, flash_addr);

  char cmd_write[] = {0x31, 0xCE};
  esp_err_t err = _stm32_write_bytes(stm32_ota, cmd_write, 2, 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to send Write command for address 0x%08lX: %s", flash_addr, esp_err_to_name(err));
    return err;
  }

  // Send the load address with the last byte being an XOR of all bytes combined
  const char load_address_xor   = _stm32_load_address_xor(load_address);
  char       cmd_load_address[] = {
      load_address->high_byte, load_address->mid_high_byte, load_address->mid_low_byte, load_address->low_byte,
      load_address_xor,
  };
  ESP_LOGD(STM32_TAG, "LOAD ADDRESS  WRITE %02x%02x%02x%02x (XOR: %02x)", load_address->high_byte,
           load_address->mid_high_byte, load_address->mid_low_byte, load_address->low_byte, load_address_xor);
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_load_address, 5, 1, STM32_UART_TIMEOUT));

  // Ensure we always send 1 to 256 bytes as a size (0 < N <= 255)
  size_t ota_bytes_size = ota_data_size - 1;

  // Send the size of the data to be written
  if (uart_write_bytes(stm32_ota->uart_port, &ota_bytes_size, 1) != 1) {
    ESP_LOGE(STM32_TAG, "Failed to write page size for address 0x%08lX (size: %d)", flash_addr, ota_bytes_size);
    return ESP_FAIL;
  }

  // Write the page to STM32 memory
  if (uart_write_bytes(stm32_ota->uart_port, ota_data, ota_data_size) != ota_data_size) {
    ESP_LOGE(STM32_TAG, "Failed to write page data to address 0x%08lX (size: %d)", flash_addr, ota_data_size);
    return ESP_ERR_INVALID_SIZE;
  }

  // Calculate the XOR of all bytes for the checksum and send it to the STM32
  char page_checksum = ota_bytes_size;
  for (size_t i = 0; i < ota_data_size; i++) {
    page_checksum ^= ota_data[i];
  }
  if (uart_write_bytes(stm32_ota->uart_port, &page_checksum, 1) != 1) {
    ESP_LOGE(STM32_TAG, "Failed to send page checksum for address 0x%08lX (checksum: 0x%02X)", flash_addr,
             page_checksum);
    return ESP_ERR_INVALID_CRC;
  }

  // Await ACK from STM32
  err = _stm32_await_rx(stm32_ota, 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to receive ACK for write to address 0x%08lX: %s", flash_addr, esp_err_to_name(err));
    return err;
  }

  // Validate we received the ACK indicating successful memory write operation
  uint8_t   response      = 0x00;
  const int received_size = uart_read_bytes(stm32_ota->uart_port, &response, 1, STM32_UART_TIMEOUT);

  if (received_size == 0) {
    ESP_LOGE(STM32_TAG, "Timeout waiting for write ACK at address 0x%08lX", flash_addr);
    return ESP_ERR_TIMEOUT;
  }

  if (response != STM32_UART_ACK) {
    ESP_LOGE(STM32_TAG, "Invalid write ACK at address 0x%08lX (received: 0x%02X, expected: 0x%02X)", flash_addr,
             response, STM32_UART_ACK);
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Read back the data for verification
  ESP_LOGD(STM32_TAG, "Verifying write at address 0x%08lX (%d bytes)", flash_addr, ota_data_size);
  char cmd_read[] = {0x11, 0xEE};
  err = _stm32_write_bytes(stm32_ota, cmd_read, 2, 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to send Read command (0x11) for verification at address 0x%08lX: %s "
             "(if NACK received, check read protection status)", flash_addr, esp_err_to_name(err));
    return err;
  }

  // Send the load address with the last byte being an XOR of all bytes combined
  ESP_LOGD(STM32_TAG, "LOAD ADDRESS READ %02x%02x%02x%02x (XOR: %02x)", load_address->high_byte,
           load_address->mid_high_byte, load_address->mid_low_byte, load_address->low_byte, load_address_xor);
  STM32_ERROR_CHECK(_stm32_write_bytes(stm32_ota, cmd_load_address, 5, 1, STM32_UART_TIMEOUT));

  // Send the size of the data to be read along with it's checksum (complement)
  char ota_read_size[] = {
      ota_bytes_size,
      ota_bytes_size ^ 0xFF,
  };
  if (uart_write_bytes(stm32_ota->uart_port, &ota_read_size, 2) != 2) {
    ESP_LOGE(STM32_TAG, "Failed to send read size request for verification at address 0x%08lX (size: %d)", flash_addr,
             ota_bytes_size);
    return ESP_FAIL;
  }

  // Await ACK from STM32 to respond with data size and checksum
  err = _stm32_await_rx(stm32_ota, ota_data_size + 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE(STM32_TAG, "Failed to receive read data for verification at address 0x%08lX: %s", flash_addr,
             esp_err_to_name(err));
    return err;
  }

  // Prepare for reading back UART data for verification
  uint8_t ota_data_response[ota_data_size + 1];
  const int ota_data_response_size =
      uart_read_bytes(stm32_ota->uart_port, ota_data_response, ota_data_size + 1, 1000 / portTICK_PERIOD_MS);
  if (ota_data_response_size != ota_data_size + 1 || ota_data_response[0] != STM32_UART_ACK) {
    ESP_LOGE(STM32_TAG, "Invalid read response for verification at address 0x%08lX (expected: %d, received: %d)",
             flash_addr, ota_data_size + 1, ota_data_response_size);
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Verify that written data matches what we read back
  for (size_t i = 0; i < ota_data_size; i++) {
    // Note that we add 1 to the response index as the first byte in the
    // response is an ACK byte (0x79)
    if (ota_data[i] != ota_data_response[i + 1]) {
      ESP_LOGE(STM32_TAG,
               "Verification failed at address 0x%08lX offset %d (wrote: 0x%02X, read: 0x%02X)", flash_addr, i,
               ota_data[i], ota_data_response[i + 1]);
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  ESP_LOGD(STM32_TAG, "Verification complete for address 0x%08lX (%d bytes)", flash_addr, ota_data_size);

  STM32_ERROR_CHECK(stm32_increment_loadaddress(load_address, ota_data_size));

  return ESP_OK;
}
