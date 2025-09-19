idf_component_register(SRCS "main.c"
                    REQUIRES nvs_flash esp_driver_uart modbus_handler mercury236
                    INCLUDE_DIRS ".")
