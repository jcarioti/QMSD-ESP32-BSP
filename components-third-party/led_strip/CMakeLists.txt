set(requires driver)
set(src_dirs .)

if(${IDF_VERSION_MAJOR} GREATER_EQUAL 6)
    list(APPEND requires esp_driver_rmt)
    set(src_dirs)
endif()

idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS include
    REQUIRES ${requires}
)
