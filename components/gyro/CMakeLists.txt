idf_component_register(SRCS "gyro.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_adc
                       PRIV_REQUIRES driver
                       )
