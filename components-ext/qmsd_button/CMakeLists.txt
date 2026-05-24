set(srcs
    "qmsd_button.c"
    "qmsd_button_gpio.c"
)
set(requires)

if("${IDF_VERSION_MAJOR}" VERSION_GREATER_EQUAL "6")
    list(APPEND requires esp_driver_gpio)
elseif("${IDF_VERSION_MAJOR}" VERSION_GREATER_EQUAL "5")
    list(APPEND srcs "qmsd_button_adc.c")
    list(APPEND requires driver)
    list(APPEND requires esp_adc)
else() 
    list(APPEND srcs "qmsd_button_adc.c")
    list(APPEND requires driver)
    list(APPEND requires esp_adc_cal)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS .
    REQUIRES ${requires}
)
