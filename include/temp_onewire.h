#ifndef TEMP_ONEWIRE_H
#define TEMP_ONEWIRE_H

#include <esp_err.h>
#include "onewire_bus.h"

typedef enum {
    TEMP_ONEWIRE_TYPE_DS18B20,
    TEMP_ONEWIRE_TYPE_MAX31850
} temp_onewire_device_type_t;

typedef struct temp_onewire_device_t *temp_onewire_device_handle_t;

typedef struct {
    int gpio_pin;           ///< GPIO pin for 1-Wire bus
    uint32_t max_devices;   ///< Maximum number of devices to discover
} temp_onewire_config_t;

esp_err_t temp_onewire_init(const temp_onewire_config_t *config, onewire_bus_handle_t *ret_bus);
esp_err_t temp_onewire_get_device_by_id(uint8_t id, temp_onewire_device_handle_t *ret_device);
esp_err_t temp_onewire_trigger_temperature_conversion(temp_onewire_device_handle_t device);
esp_err_t temp_onewire_get_temperature(temp_onewire_device_handle_t device, float *temperature);
esp_err_t temp_onewire_del(onewire_bus_handle_t bus);

#endif // TEMP_ONEWIRE_H