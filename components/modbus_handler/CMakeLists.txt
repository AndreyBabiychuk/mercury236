idf_component_register(
    SRCS "modbus_handler.c"
    INCLUDE_DIRS "include"
    REQUIRES esp-modbus driver  # Зависимости: esp-modbus для Modbus, driver для UART
    
)