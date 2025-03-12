idf_component_register(SRCS "app_main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver i2c_ssd1306_lib esp_gdbstub)
