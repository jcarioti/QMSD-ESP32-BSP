idf_component_register(REQUIRES lvgl)

target_compile_definitions(${COMPONENT_LIB} INTERFACE LV_CONF_INCLUDE_SIMPLE=1)
