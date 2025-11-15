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

// Forward declarations for internal helper functions
static esp_err_t _stm32_await_rx(stm32_ota_t *stm32_ota, size_t expected_size, uint32_t timeout);
static esp_err_t _stm32_write_bytes(stm32_ota_t *stm32_ota, const char *write_bytes, size_t write_size,
                                    size_t response_size, uint32_t timeout);

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
 * @brief Internal function to check protection status via Get Version command (0x01)
 *
 * Attempts to detect Read Protection (RDP) and Write Protection (WRP) status by parsing
 * the Get Version command response. Per AN3155, some bootloaders return protection status
 * in the option bytes.
 *
 * @param stm32_ota Pointer to stm32_ota_t struct
 * @param rdp_active Pointer to uint8_t - set to 1 if RDP detected, 0 if not (output parameter)
 * @param wrp_active Pointer to uint8_t - set to 1 if WRP detected, 0 if not (output parameter)
 * @return esp_err_t ESP_OK if detection succeeded, ESP_FAIL if Get Version not supported or parsing failed
 */
static esp_err_t _stm32_check_protection_status(stm32_ota_t *stm32_ota, uint8_t *rdp_active, uint8_t *wrp_active) {
  // Send Get Version command (0x01, 0xFE)
  ESP_LOGD(STM32_TAG, "Checking protection status via Get Version command (0x01)");
  char cmd_get_version[] = {0x01, 0xFE};

  // Flush UART before sending
  esp_err_t err = uart_flush(stm32_ota->uart_port);
  if (err != ESP_OK) {
    ESP_LOGW(STM32_TAG, "UART flush failed before Get Version: %s", esp_err_to_name(err));
    return ESP_FAIL;
  }

  // Send command
  int bytes_written = uart_write_bytes(stm32_ota->uart_port, cmd_get_version, 2);
  if (bytes_written != 2) {
    ESP_LOGW(STM32_TAG, "Failed to write Get Version command");
    return ESP_FAIL;
  }

  // Wait for response (ACK + N + version + option_byte1 + option_byte2 + ACK = 6 bytes min)
  // Some bootloaders may return more option bytes, so we read up to 10 bytes
  err = _stm32_await_rx(stm32_ota, 5, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGW(STM32_TAG, "Get Version command timeout or not supported: %s", esp_err_to_name(err));
    return ESP_FAIL;
  }

  // Read response
  uint8_t response[10];
  int read_size = uart_read_bytes(stm32_ota->uart_port, response, 10, 1000 / portTICK_PERIOD_MS);

  if (read_size < 5) {
    ESP_LOGW(STM32_TAG, "Get Version response too short: %d bytes", read_size);
    return ESP_FAIL;
  }

  // Validate first byte is ACK
  if (response[0] != STM32_UART_ACK) {
    ESP_LOGW(STM32_TAG, "Get Version did not return ACK: 0x%02x", response[0]);
    return ESP_FAIL;
  }

  // Parse response: ACK + N + version + option_bytes...
  uint8_t n = response[1];  // Number of bytes to follow
  uint8_t version = response[2];

  ESP_LOGD(STM32_TAG, "Bootloader version: 0x%02x, response length: %d", version, n);

  // Initialize outputs to "not detected"
  *rdp_active = 0;
  *wrp_active = 0;

  // Parse option bytes if available (bytes 3+)
  // Note: Protection status encoding varies by bootloader version and chip family
  // Conservative approach: check if option bytes suggest protection is active
  if (read_size >= 4) {
    uint8_t option_byte1 = response[3];

    // Per AN3155, option byte 1 may contain RDP status
    // Common encoding: 0xAA = unprotected, anything else = protected
    // However, this varies by chip, so we use heuristics
    if (option_byte1 != 0xAA && option_byte1 != 0x00) {
      ESP_LOGD(STM32_TAG, "Option byte 1 suggests protection may be active: 0x%02x", option_byte1);
      // Set rdp_active as a hint, but not definitive
      *rdp_active = 1;
    }
  }

  // Note: Write protection detection is chip-specific and not reliably detectable
  // via Get Version on all bootloaders. We default to *wrp_active = 0 (attempt unprotect)

  ESP_LOGD(STM32_TAG, "Protection status detection: RDP=%d, WRP=%d (heuristic)", *rdp_active, *wrp_active);
  return ESP_OK;
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
  // Check if we should skip write unprotect based on flags or detection
  uint8_t wrp_detected = 0;
  uint8_t rdp_detected = 0;

  if (!stm32_ota->skip_write_unprotect) {
    // Attempt to detect protection status via Get Version command
    esp_err_t detect_err = _stm32_check_protection_status(stm32_ota, &rdp_detected, &wrp_detected);

    if (detect_err == ESP_OK) {
      ESP_LOGI(STM32_TAG, "Protection detection succeeded - RDP: %s, WRP: %s",
               rdp_detected ? "active" : "inactive", wrp_detected ? "active" : "inactive");
    } else {
      ESP_LOGW(STM32_TAG, "Protection detection unavailable - will attempt unprotect commands");
      // Fall back to attempting unprotect (conservative approach)
      wrp_detected = 0;  // Unknown, so attempt unprotect
    }

    // Send Write Unprotect (0x73) - if protection enabled, will mass erase + trigger system reset
    ESP_LOGI(STM32_TAG, "Sending Write Unprotect command (0x73)");
    char cmd_write_unprotect[] = {0x73, 0x8C};
    err = _stm32_write_bytes(stm32_ota, cmd_write_unprotect, 2, 1, STM32_UART_TIMEOUT);
    if (err == ESP_OK) {
      // ACK received - protection was active, chip will mass erase + reset now
      ESP_LOGI(STM32_TAG, "Write protection disabled - chip mass erasing + resetting (waiting 2s)");
      vTaskDelay(pdMS_TO_TICKS(2000));  // Mass erase takes time

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
  } else {
    ESP_LOGI(STM32_TAG, "Skipping Write Unprotect (skip_write_unprotect = 1)");
  }

  // 6. Disable read protection (required for READ command during write verification)
  if (!stm32_ota->skip_read_unprotect) {
    // Note: rdp_detected was set by _stm32_check_protection_status() above (if successful)

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
  } else {
    ESP_LOGI(STM32_TAG, "Skipping Read Unprotect (skip_read_unprotect = 1)");
  }

  // 7. Erase flash memory - try regular erase first, fall back to extended erase
  // Note: If write/read protection was active, flash was already mass erased by steps 5-6
  // This ensures flash is clean regardless of protection state
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

esp_err_t stm32_ota_write_page(stm32_ota_t *stm32_ota, stm32_loadaddress_t *load_address, const char *ota_data,
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

  // Write successful - increment address for next page
  STM32_ERROR_CHECK(stm32_increment_loadaddress(load_address, ota_data_size));

  return ESP_OK;
}

/**
 * @brief Internal function to read and verify written data
 *
 * Performs read-back verification by reading the specified address and comparing with expected data.
 * This is extracted as a helper to enable retry logic.
 *
 * @param stm32_ota Pointer to stm32_ota_t struct
 * @param address Address to read from (must not be incremented)
 * @param expected_data Expected data to compare against
 * @param data_size Number of bytes to read and verify
 * @param flash_addr Flash address (for logging only)
 * @return esp_err_t ESP_OK if verification succeeds, error code otherwise
 */
static esp_err_t _stm32_read_and_verify(stm32_ota_t *stm32_ota, stm32_loadaddress_t *address,
                                        const char *expected_data, size_t data_size, uint32_t flash_addr) {
  // Send Read Memory command (0x11, 0xEE)
  char cmd_read[] = {0x11, 0xEE};
  esp_err_t err = _stm32_write_bytes(stm32_ota, cmd_read, 2, 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGD(STM32_TAG, "Read command (0x11) rejected at address 0x%08lX: %s", flash_addr, esp_err_to_name(err));
    return err;  // Return error (will trigger retry or skip verification)
  }

  // Send the load address with the last byte being an XOR of all bytes combined
  const char load_address_xor = _stm32_load_address_xor(address);
  char cmd_load_address[] = {
      address->high_byte, address->mid_high_byte, address->mid_low_byte,
      address->low_byte, load_address_xor,
  };
  ESP_LOGD(STM32_TAG, "LOAD ADDRESS READ %02x%02x%02x%02x (XOR: %02x)", address->high_byte,
           address->mid_high_byte, address->mid_low_byte, address->low_byte, load_address_xor);
  err = _stm32_write_bytes(stm32_ota, cmd_load_address, 5, 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGD(STM32_TAG, "Address send failed at 0x%08lX: %s", flash_addr, esp_err_to_name(err));
    return err;
  }

  // Send the size of the data to be read along with its checksum (complement)
  size_t bytes_size = data_size - 1;  // 0 < N <= 255
  char read_size_cmd[] = {
      bytes_size,
      bytes_size ^ 0xFF,
  };
  if (uart_write_bytes(stm32_ota->uart_port, &read_size_cmd, 2) != 2) {
    ESP_LOGD(STM32_TAG, "Failed to send read size request for address 0x%08lX (size: %d)", flash_addr, bytes_size);
    return ESP_FAIL;
  }

  // Await response data (ACK + data bytes)
  err = _stm32_await_rx(stm32_ota, data_size + 1, STM32_UART_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGD(STM32_TAG, "Failed to receive read data at address 0x%08lX: %s", flash_addr, esp_err_to_name(err));
    return err;
  }

  // Read response
  uint8_t response[data_size + 1];
  const int response_size = uart_read_bytes(stm32_ota->uart_port, response, data_size + 1, 1000 / portTICK_PERIOD_MS);
  if (response_size != data_size + 1 || response[0] != STM32_UART_ACK) {
    ESP_LOGD(STM32_TAG, "Invalid read response at address 0x%08lX (expected: %d, received: %d)",
             flash_addr, data_size + 1, response_size);
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Verify that written data matches what we read back
  for (size_t i = 0; i < data_size; i++) {
    // Note: response[0] is ACK, actual data starts at response[1]
    if (expected_data[i] != response[i + 1]) {
      ESP_LOGD(STM32_TAG, "Verification mismatch at address 0x%08lX offset %d (wrote: 0x%02X, read: 0x%02X)",
               flash_addr, i, expected_data[i], response[i + 1]);
      return ESP_ERR_INVALID_RESPONSE;
    }
  }

  ESP_LOGD(STM32_TAG, "Verification complete for address 0x%08lX (%d bytes)", flash_addr, data_size);
  return ESP_OK;
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

  // Save original load address for verification (write increments it)
  stm32_loadaddress_t original_address = *load_address;

  // Perform the write operation (increments load_address on success)
  esp_err_t write_err = stm32_ota_write_page(stm32_ota, load_address, ota_data, ota_data_size);
  if (write_err != ESP_OK) {
    return write_err;
  }

  // Progressive retry loop for read verification (3 attempts with increasing delays)
  ESP_LOGD(STM32_TAG, "Verifying write at address 0x%08lX (%d bytes)", flash_addr, ota_data_size);

  esp_err_t verify_err = ESP_OK;
  for (int retry = 0; retry < STM32_READ_VERIFY_RETRIES; retry++) {
    // Add progressive delay before retry (skip on first attempt)
    if (retry == 1) {
      ESP_LOGW(STM32_TAG, "Retry %d/%d after %dms delay (address 0x%08lX)", retry + 1, STM32_READ_VERIFY_RETRIES,
               STM32_RETRY_DELAY_1_MS, flash_addr);
      vTaskDelay(pdMS_TO_TICKS(STM32_RETRY_DELAY_1_MS));  // 500ms
    } else if (retry == 2) {
      ESP_LOGW(STM32_TAG, "Retry %d/%d after %dms delay (address 0x%08lX)", retry + 1, STM32_READ_VERIFY_RETRIES,
               STM32_RETRY_DELAY_2_MS, flash_addr);
      vTaskDelay(pdMS_TO_TICKS(STM32_RETRY_DELAY_2_MS));  // 2s
    } else if (retry == 3) {
      ESP_LOGW(STM32_TAG, "Retry %d/%d after %dms delay (address 0x%08lX)", retry + 1, STM32_READ_VERIFY_RETRIES,
               STM32_RETRY_DELAY_3_MS, flash_addr);
      vTaskDelay(pdMS_TO_TICKS(STM32_RETRY_DELAY_3_MS));  // 4s
    }

    // Attempt read verification
    verify_err = _stm32_read_and_verify(stm32_ota, &original_address, ota_data, ota_data_size, flash_addr);

    // Success: break out of retry loop
    if (verify_err == ESP_OK) {
      if (retry > 0) {
        ESP_LOGI(STM32_TAG, "Verification succeeded on attempt %d/%d (address 0x%08lX)", retry + 1,
                 STM32_READ_VERIFY_RETRIES, flash_addr);
      }
      break;
    }

    // Determine if we should retry based on error type
    // Retry only on NACK (ESP_ERR_INVALID_RESPONSE) and timeouts (ESP_ERR_TIMEOUT)
    if (verify_err == ESP_ERR_INVALID_RESPONSE || verify_err == ESP_ERR_TIMEOUT) {
      ESP_LOGW(STM32_TAG, "Verification failed (attempt %d/%d): %s - will retry", retry + 1, STM32_READ_VERIFY_RETRIES,
               esp_err_to_name(verify_err));
      continue;  // Retry
    } else {
      // Other error (e.g., read protection active): don't retry
      ESP_LOGW(STM32_TAG, "Read command rejected at address 0x%08lX: %s - skipping verification (likely read-protected)",
               flash_addr, esp_err_to_name(verify_err));
      break;  // Don't retry, gracefully skip verification
    }
  }

  // If all retries failed, log warning but continue (graceful degradation)
  if (verify_err != ESP_OK) {
    ESP_LOGW(STM32_TAG, "Verification failed after %d attempts at address 0x%08lX - continuing without verification",
             STM32_READ_VERIFY_RETRIES, flash_addr);
  }

  // Write succeeded (verification is optional) - load_address was already incremented by stm32_ota_write_page()
  return ESP_OK;
}
