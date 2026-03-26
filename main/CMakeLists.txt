idf_component_register(
    SRCS
        "main.c"
        "adc/adc.c"
        "display/display.c"
        "bearing/bearing.c"
        "config/config.c"
    INCLUDE_DIRS "." "adc" "display" "bearing" "config"
)
