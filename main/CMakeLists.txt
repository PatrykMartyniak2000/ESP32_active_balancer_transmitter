idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash bt
                    REQUIRES bt
                    REQUIRES esp_adc)
