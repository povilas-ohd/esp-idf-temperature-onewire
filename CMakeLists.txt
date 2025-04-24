idf_component_register(
    SRCS "src/temp_onewire.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_common onewire_bus
)