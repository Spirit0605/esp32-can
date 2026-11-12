idf_component_register(
    SRCS "can.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_driver_twai
)