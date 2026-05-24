set(srcs
    "i2s_lcd_esp32_driver.c"
    "i2s_lcd_esp32s2_driver.c"
    "iic_bus.c"
    "spi_bus.c"
)
set(requires driver)

if(IDF_VERSION_MAJOR VERSION_LESS 6)
    list(APPEND srcs "8080_lcd_esp32s3.c")
else()
    list(APPEND requires esp_driver_gpio esp_driver_i2c esp_driver_i2s esp_driver_spi)
endif()

idf_component_register(SRCS ${srcs}
                        INCLUDE_DIRS "include"
                        REQUIRES ${requires})
