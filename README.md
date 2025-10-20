# STM32 OTA Component

ESP32-IDF component for Over-The-Air (OTA) firmware updates to STM32 microcontrollers via UART using the STM32 bootloader protocol (AN3155).

## Features

- **UART Bootloader Protocol**: Implements STM32 AN3155 UART bootloader protocol
- **Flexible UART Management**: Supports both standalone and externally managed UART modes
- **Mass Erase**: Full flash memory erase with extended timeout support
- **Page-by-Page Programming**: Write firmware in configurable page sizes (up to 255 bytes)
- **GPIO Control**: Manages STM32 BOOT0, BOOT1 (optional), and NRST pins
- **Error Handling**: Comprehensive error codes and timeout handling

## UART Management Modes

### Standalone Mode (Default)

The component initializes and manages the UART peripheral internally.

```c
stm32_ota_t stm32_ota = {
    .stm_boot0_pin = GPIO_NUM_18,
    .stm_nrst_pin = GPIO_NUM_19,
    .stm_tx_pin = GPIO_NUM_17,
    .stm_rx_pin = GPIO_NUM_16,
    .uart_port = UART_NUM_1,
    .uart_baudrate = 115200,
    .uart_rx_buffer_size = 256,
    .uart_tx_buffer_size = 1024,
    .uart_queue_size = 0,
    .uart_intr_alloc_flags = 0,
    .uart_queue = NULL,
    .disable_boot1_pin = 1,
    .uart_externally_managed = 0  // Component handles UART init
};

stm32_init(&stm32_ota);
```

### Externally Managed Mode (Shared UART)

When the UART is shared with other protocols (e.g., VCU protocol), set `uart_externally_managed = 1`. The component will skip UART initialization and use the existing UART configuration.

**Important:** You must reconfigure the UART to STM32 bootloader parameters (115200 baud, 8E1) before calling `stm32_init()`.

```c
// Example: Reconfigure UART from VCU protocol (460800 baud, 8N1) to bootloader mode (115200 baud, 8E1)
uart_hal_reconfigure(uart_hal, 115200);

stm32_ota_t stm32_ota = {
    .stm_boot0_pin = GPIO_NUM_18,
    .stm_nrst_pin = GPIO_NUM_19,
    .stm_tx_pin = GPIO_NUM_17,
    .stm_rx_pin = GPIO_NUM_16,
    .uart_port = UART_NUM_1,
    .uart_baudrate = 115200,  // Still specify for reference
    .disable_boot1_pin = 1,
    .uart_externally_managed = 1,  // Skip UART init
    .uart_rx_buffer_size = 0,      // Not used
    .uart_tx_buffer_size = 0,      // Not used
    .uart_queue_size = 0,          // Not used
    .uart_intr_alloc_flags = 0,    // Not used
    .uart_queue = NULL
};

stm32_init(&stm32_ota);
```

## Configuration Parameters

| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `stm_boot0_pin` | `gpio_num_t` | GPIO connected to STM32 BOOT0 pin | Yes |
| `stm_boot1_pin` | `gpio_num_t` | GPIO connected to STM32 BOOT1 pin | If `disable_boot1_pin = 0` |
| `stm_nrst_pin` | `gpio_num_t` | GPIO connected to STM32 NRST (reset) pin | Yes |
| `stm_tx_pin` | `gpio_num_t` | ESP32 TX pin (STM32 RX) | If `uart_externally_managed = 0` |
| `stm_rx_pin` | `gpio_num_t` | ESP32 RX pin (STM32 TX) | If `uart_externally_managed = 0` |
| `uart_port` | `uart_port_t` | UART port number (e.g., `UART_NUM_1`) | Yes |
| `uart_baudrate` | `int` | Baud rate (115200 for bootloader) | Yes |
| `uart_externally_managed` | `uint8_t` | `1` = UART managed externally, `0` = component manages UART | Yes |
| `disable_boot1_pin` | `uint8_t` | `1` = BOOT1 pin not used, `0` = use BOOT1 pin | Yes |
| `uart_rx_buffer_size` | `int` | RX buffer size in bytes | If `uart_externally_managed = 0` |
| `uart_tx_buffer_size` | `int` | TX buffer size in bytes (e.g., 1024) | If `uart_externally_managed = 0` |
| `uart_queue_size` | `int` | UART event queue depth (0 = no queue) | If `uart_externally_managed = 0` |
| `uart_intr_alloc_flags` | `int` | UART interrupt allocation flags (0 for default) | If `uart_externally_managed = 0` |
| `uart_queue` | `QueueHandle_t*` | UART event queue handle (NULL = no queue) | If `uart_externally_managed = 0` |

## Usage Example

```c
#include "stm32ota.h"

// Initialize STM32 OTA
stm32_ota_t stm32_ota = { /* ... config ... */ };
esp_err_t err = stm32_init(&stm32_ota);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize STM32 OTA: %s", esp_err_to_name(err));
    return;
}

// Enter bootloader mode and erase flash
err = stm32_begin_flash(&stm32_ota);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enter bootloader mode: %s", esp_err_to_name(err));
    return;
}

// Load address for STM32 flash (typically 0x08000000)
stm32_loadaddress_t load_addr = {
    .high_byte = 0x08,
    .mid_high_byte = 0x00,
    .mid_low_byte = 0x00,
    .low_byte = 0x00
};

// Write firmware data page by page
size_t offset = 0;
while (offset < firmware_size) {
    size_t page_size = (firmware_size - offset > 256) ? 256 : (firmware_size - offset);

    err = stm32_write_page(&stm32_ota, &load_addr, &firmware_data[offset], page_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write page at offset %zu: %s", offset, esp_err_to_name(err));
        return;
    }

    offset += page_size;
}

// Reset STM32 to run new firmware
err = stm32_reset(&stm32_ota);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset STM32: %s", esp_err_to_name(err));
}
```

## API Reference

### `esp_err_t stm32_init(stm32_ota_t *stm32_ota)`

Initialize the STM32 OTA component with the given configuration.

- **Parameters**: Pointer to `stm32_ota_t` configuration struct
- **Returns**: `ESP_OK` on success, error code otherwise

### `esp_err_t stm32_begin_flash(stm32_ota_t *stm32_ota)`

Enter STM32 bootloader mode and perform mass erase of flash memory.

- **Parameters**: Pointer to initialized `stm32_ota_t` struct
- **Returns**: `ESP_OK` on success, error code otherwise
- **Timeout**: ~70 seconds for mass erase operation

### `esp_err_t stm32_write_page(stm32_ota_t *stm32_ota, stm32_loadaddress_t *load_addr, uint8_t *data, size_t len)`

Write a page of data to STM32 flash memory.

- **Parameters**:
  - `stm32_ota`: Pointer to initialized struct
  - `load_addr`: Target flash address (auto-increments after write)
  - `data`: Firmware data buffer
  - `len`: Data length (1-255 bytes)
- **Returns**: `ESP_OK` on success, error code otherwise

### `esp_err_t stm32_reset(stm32_ota_t *stm32_ota)`

Reset the STM32 to run mode (BOOT0=LOW, NRST toggle).

- **Parameters**: Pointer to initialized `stm32_ota_t` struct
- **Returns**: `ESP_OK` on success, error code otherwise

### `esp_err_t stm32_increment_loadaddress(stm32_loadaddress_t *load_addr, size_t addition)`

Increment the flash load address (handles overflow).

- **Parameters**:
  - `load_addr`: Pointer to load address struct
  - `addition`: Number of bytes to increment
- **Returns**: `ESP_OK` on success, error code otherwise

## Hardware Requirements

- STM32 microcontroller with UART bootloader support
- GPIO connections:
  - ESP32 TX → STM32 RX (bootloader UART)
  - ESP32 RX → STM32 TX (bootloader UART)
  - ESP32 GPIO → STM32 BOOT0
  - ESP32 GPIO → STM32 NRST
  - (Optional) ESP32 GPIO → STM32 BOOT1

## Protocol Details

- **Baud Rate**: 115200 (STM32 bootloader maximum)
- **Data Format**: 8 data bits, Even parity, 1 stop bit (8E1)
- **Flow Control**: None
- **Timeout**: 16 seconds (normal operations), 70 seconds (mass erase)
- **ACK Byte**: `0x79`
- **Max Page Size**: 255 bytes

## References

- [AN3155: USART protocol used in the STM32 bootloader](https://www.st.com/resource/en/application_note/an3155-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)

## License

See LICENSE file.
