set(requires driver)

if(${IDF_VERSION_MAJOR} GREATER_EQUAL 6)
    list(APPEND requires esp_driver_gpio esp_driver_spi)
endif()

idf_component_register(
    SRC_DIRS .
    INCLUDE_DIRS .
    REQUIRES ${requires}
)
