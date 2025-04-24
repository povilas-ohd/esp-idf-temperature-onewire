#include "temp_onewire.h"
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "onewire_cmd.h"
#include "onewire_crc.h"
#include "onewire_device.h"

static const char *TAG = "temp_onewire";

#define MAX_DEVICES 4  // Match BENTO_MAX_DEVICES
#define DS18B20_FAMILY_CODE 0x28
#define MAX31850_FAMILY_CODE 0x3B
#define DS18B20_CONVERT_CMD 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE
#define MAX31850_READ_SCRATCHPAD 0xBE // Correct command for reading scratchpad

/**
 * @brief Structure of DS18B20 scratchpad
 */
typedef struct {
    uint8_t temp_lsb;
    uint8_t temp_msb;
    uint8_t th_user1;
    uint8_t tl_user2;
    uint8_t configuration;
    uint8_t _reserved1;
    uint8_t _reserved2;
    uint8_t _reserved3;
    uint8_t crc_value;
} __attribute__((packed)) ds18b20_scratchpad_t;

/**
 * @brief Structure of MAX31850 scratchpad
 */
typedef struct {
    uint8_t tc_temp_lsb; // Thermocouple temperature
    uint8_t tc_temp_msb;
    uint8_t cj_temp_lsb; // Cold-junction temperature
    uint8_t cj_temp_msb;
    uint8_t configuration; // AD0-AD3 in bits 0-3
    uint8_t _reserved1;
    uint8_t configuration2;
    uint8_t _reserved2;
    uint8_t crc_value;
} __attribute__((packed)) max31850_scratchpad_t;

/**
 * @brief Temperature device structure
 */
typedef struct temp_onewire_device_t {
    onewire_bus_handle_t bus;
    onewire_device_address_t address;
    temp_onewire_device_type_t type;
    uint8_t device_id; // 0-3 for TEMP0-3
} temp_onewire_device_t;

/**
 * @brief Static array of devices
 */
static temp_onewire_device_t *s_devices[MAX_DEVICES];
static uint32_t s_device_count = 0;

/**
 * @brief Send command to device
 */
static esp_err_t send_command(temp_onewire_device_handle_t device, uint8_t cmd)
{
    ESP_RETURN_ON_FALSE(device, ESP_ERR_INVALID_ARG, TAG, "invalid device");
    uint8_t tx_buffer[10] = {0};
    tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
    memcpy(&tx_buffer[1], &device->address, sizeof(device->address));
    tx_buffer[9] = cmd;
    return onewire_bus_write_bytes(device->bus, tx_buffer, 10);
}

/**
 * @brief Read MAX31850 configuration register to get ID
 */
static esp_err_t max31850_read_config_id(onewire_bus_handle_t bus, onewire_device_address_t address, uint8_t *id)
{
    ESP_RETURN_ON_ERROR(onewire_bus_reset(bus), TAG, "reset bus error");
    uint8_t tx_buffer[9] = {ONEWIRE_CMD_MATCH_ROM};
    memcpy(&tx_buffer[1], &address, 8);
    ESP_RETURN_ON_ERROR(onewire_bus_write_bytes(bus, tx_buffer, 9), TAG, "send MATCH_ROM failed");
    ESP_RETURN_ON_ERROR(onewire_bus_write_bytes(bus, (uint8_t[]){MAX31850_READ_SCRATCHPAD}, 1), TAG, "send READ_SCRATCHPAD failed");
    max31850_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR(onewire_bus_read_bytes(bus, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "read scratchpad failed");
    
    // Validate CRC
    uint8_t calculated_crc = onewire_crc8(0, (uint8_t *)&scratchpad, sizeof(scratchpad) - 1);
    if (calculated_crc != scratchpad.crc_value) {
        ESP_LOGE(TAG, "Scratchpad CRC error: calculated 0x%02X, expected 0x%02X", calculated_crc, scratchpad.crc_value);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Log full scratchpad for debugging
    ESP_LOGI(TAG, "MAX31850 scratchpad: TC_LSB=0x%02X TC_MSB=0x%02X CJ_LSB=0x%02X CJ_MSB=0x%02X Config=0x%02X Config2=0x%02X CRC=0x%02X",
             scratchpad.tc_temp_lsb, scratchpad.tc_temp_msb, scratchpad.cj_temp_lsb, scratchpad.cj_temp_msb,
             scratchpad.configuration, scratchpad.configuration2, scratchpad.crc_value);
    
    *id = scratchpad.configuration & 0x0F; // Lower 4 bits are AD0-AD3
    return ESP_OK;
}

/**
 * @brief Initialize and discover devices
 */
esp_err_t temp_onewire_init(const temp_onewire_config_t *config, onewire_bus_handle_t *ret_bus)
{
    ESP_RETURN_ON_FALSE(config && ret_bus, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->max_devices <= MAX_DEVICES, ESP_ERR_INVALID_ARG, TAG, "max_devices exceeds limit");

    // Initialize 1-Wire bus
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = config->gpio_pin,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1-byte command + 8-byte ROM + 1-byte device command
    };
    onewire_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus), TAG, "create 1-Wire bus failed");

    // Discover devices
    onewire_device_iter_handle_t iter;
    ESP_RETURN_ON_ERROR(onewire_new_device_iter(bus, &iter), TAG, "create device iterator failed");
    onewire_device_t onewire_dev;
    esp_err_t search_result;

    s_device_count = 0;
    ESP_LOGI(TAG, "Scanning 1-Wire bus for devices...");
    do {
        search_result = onewire_device_iter_get_next(iter, &onewire_dev);
        if (search_result == ESP_OK) {
            uint8_t family_code = onewire_dev.address & 0xFF;
            ESP_LOGI(TAG, "Found device %016llX, family code: 0x%02X", onewire_dev.address, family_code);
            if (family_code == DS18B20_FAMILY_CODE || family_code == MAX31850_FAMILY_CODE) {
                if (s_device_count >= config->max_devices) {
                    ESP_LOGW(TAG, "Device %016llX ignored: max_devices (%" PRIu32 ") reached", onewire_dev.address, config->max_devices);
                    continue;
                }
                temp_onewire_device_t *device = calloc(1, sizeof(temp_onewire_device_t));
                ESP_RETURN_ON_FALSE(device, ESP_ERR_NO_MEM, TAG, "no mem for device");
                device->bus = bus;
                device->address = onewire_dev.address;
                device->type = (family_code == DS18B20_FAMILY_CODE) ? TEMP_ONEWIRE_TYPE_DS18B20 : TEMP_ONEWIRE_TYPE_MAX31850;
                if (device->type == TEMP_ONEWIRE_TYPE_MAX31850) {
                    esp_err_t ret = max31850_read_config_id(bus, onewire_dev.address, &device->device_id);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to read MAX31850 config for %016llX: %s", onewire_dev.address, esp_err_to_name(ret));
                        free(device);
                        continue;
                    }
                    ESP_LOGI(TAG, "MAX31850 device %016llX, ID: 0x%02X", onewire_dev.address, device->device_id);
                    // Map MAX31850 IDs (0x1-0x4) to TEMP0-TEMP3
                    if (device->device_id >= 0x1 && device->device_id <= 0x4) {
                        device->device_id = device->device_id - 0x1; // Map 0x1->TEMP0, 0x2->TEMP1, 0x3->TEMP2, 0x4->TEMP3
                        ESP_LOGI(TAG, "Mapped MAX31850 %016llX to TEMP%u", onewire_dev.address, device->device_id);
                    } else {
                        ESP_LOGW(TAG, "MAX31850 %016llX has invalid ID 0x%02X, ignoring", onewire_dev.address, device->device_id);
                        free(device);
                        continue;
                    }
                } else {
                    device->device_id = s_device_count; // Sequential ID for DS18B20
                    ESP_LOGI(TAG, "DS18B20 device %016llX, ID: %u (TEMP%u)", onewire_dev.address, device->device_id, device->device_id);
                }
                s_devices[s_device_count] = device;
                s_device_count++;
            } else {
                ESP_LOGW(TAG, "Ignoring unknown device %016llX, family code: 0x%02X", onewire_dev.address, family_code);
            }
        }
    } while (search_result == ESP_OK);
    ESP_RETURN_ON_ERROR(onewire_del_device_iter(iter), TAG, "delete device iterator failed");

    ESP_LOGI(TAG, "Found %" PRIu32 " temperature devices", s_device_count);
    *ret_bus = bus;
    return ESP_OK;
}

/**
 * @brief Get device by ID
 */
esp_err_t temp_onewire_get_device_by_id(uint8_t id, temp_onewire_device_handle_t *ret_device)
{
    ESP_RETURN_ON_FALSE(ret_device, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(id <= 3, ESP_ERR_INVALID_ARG, TAG, "invalid device ID (must be 0-3)");
    for (uint32_t i = 0; i < s_device_count; i++) {
        if (s_devices[i] && s_devices[i]->device_id == id) {
            *ret_device = s_devices[i];
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "No device found with ID %u", id);
    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Trigger temperature conversion
 */
esp_err_t temp_onewire_trigger_temperature_conversion(temp_onewire_device_handle_t device)
{
    ESP_RETURN_ON_FALSE(device, ESP_ERR_INVALID_ARG, TAG, "invalid device");
    ESP_RETURN_ON_ERROR(onewire_bus_reset(device->bus), TAG, "reset bus error");
    ESP_RETURN_ON_ERROR(send_command(device, DS18B20_CONVERT_CMD), TAG, "send CONVERT_T failed");

    // Delay for conversion (750ms for DS18B20 12-bit, 250ms for MAX31850)
    vTaskDelay(pdMS_TO_TICKS(device->type == TEMP_ONEWIRE_TYPE_DS18B20 ? 750 : 250));
    return ESP_OK;
}

/**
 * @brief Get temperature
 */
esp_err_t temp_onewire_get_temperature(temp_onewire_device_handle_t device, float *temperature)
{
    ESP_RETURN_ON_FALSE(device && temperature, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(onewire_bus_reset(device->bus), TAG, "reset bus error");
    ESP_RETURN_ON_ERROR(send_command(device, DS18B20_READ_SCRATCHPAD), TAG, "send READ_SCRATCHPAD failed");

    if (device->type == TEMP_ONEWIRE_TYPE_DS18B20) {
        ds18b20_scratchpad_t scratchpad;
        ESP_RETURN_ON_ERROR(onewire_bus_read_bytes(device->bus, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "read scratchpad failed");
        ESP_RETURN_ON_FALSE(onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc_value, ESP_ERR_INVALID_CRC, TAG, "scratchpad CRC error");
        int16_t temp_raw = ((int16_t)scratchpad.temp_msb << 8) | scratchpad.temp_lsb;
        *temperature = temp_raw / 16.0f;
        ESP_LOGD(TAG, "DS18B20 TEMP%u: %.2fC", device->device_id, *temperature);
    } else {
        max31850_scratchpad_t scratchpad;
        ESP_RETURN_ON_ERROR(onewire_bus_read_bytes(device->bus, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "read scratchpad failed");
        ESP_RETURN_ON_FALSE(onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc_value, ESP_ERR_INVALID_CRC, TAG, "scratchpad CRC error");
        if (scratchpad.cj_temp_lsb & 0x07) { // Fault bits: bit 0=Open, bit 1=Short to GND, bit 2=Short to VDD
            ESP_LOGE(TAG, "MAX31850 TEMP%u fault detected: 0x%02X", device->device_id, scratchpad.cj_temp_lsb & 0x07);
            return ESP_FAIL;
        }
        int16_t temp_raw = ((int16_t)scratchpad.tc_temp_msb << 8) | scratchpad.tc_temp_lsb;
        *temperature = temp_raw / 16.0f; // Thermocouple temperature
        ESP_LOGD(TAG, "MAX31850 TEMP%u: %.2fC", device->device_id, *temperature);
    }
    return ESP_OK;
}

/**
 * @brief Delete devices and bus
 */
esp_err_t temp_onewire_del(onewire_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    for (uint32_t i = 0; i < s_device_count; i++) {
        if (s_devices[i]) {
            free(s_devices[i]);
            s_devices[i] = NULL;
        }
    }
    s_device_count = 0;
    ESP_RETURN_ON_ERROR(onewire_bus_del(bus), TAG, "delete 1-Wire bus failed");
    return ESP_OK;
}