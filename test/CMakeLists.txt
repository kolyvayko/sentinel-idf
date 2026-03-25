idf_component_register(
    SRCS "test_bearing.c" "../main/bearing/bearing.c"
    INCLUDE_DIRS "../main" "../main/bearing"
    REQUIRES unity esp_adc
)
