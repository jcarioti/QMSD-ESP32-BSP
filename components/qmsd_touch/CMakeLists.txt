set(requires driver i2c_bus qmsd_utils)

if("${IDF_VERSION_MAJOR}" VERSION_GREATER_EQUAL "6")
    list(APPEND requires esp_driver_gpio)
endif()

idf_component_register(
	SRC_DIRS . ft5x06 gt2863 cst816t gt911 chsc6540 sp2010 cst328 cst3240
	INCLUDE_DIRS . ft5x06
    REQUIRES ${requires} 
)
