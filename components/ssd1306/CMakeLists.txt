# cmake_minimum_required (VERSION 3.5)

# file(GLOB_RECURSE SOURCE_FILES src/*.cpp src/*.c)
# file(GLOB_RECURSE HEADER_FILES src/*.h)

# if (NOT DEFINED COMPONENT_DIR)

#     project (lcdgfx)

#     include_directories(src)

#     add_library(ssd1306 STATIC ${HEADER_FILES} ${SOURCE_FILES})

# else()

#     idf_component_register(SRCS ${SOURCE_FILES}
#                            INCLUDE_DIRS "src"
#                            PRIV_REQUIRES driver)

# endif()

# ####################################################################################################################################################################################
##############################           CMakeFile for Kconfig Menu              #####################################################################################################
######################################################################################################################################################################################

if(NOT DEFINED COMPONENT_DIR)
    file(GLOB_RECURSE SOURCE_FILES src/*.cpp src/*.c)
    file(GLOB_RECURSE HEADER_FILES src/*.h)

    project(lcdgfx)

    include_directories(src)

    add_library(ssd1306 STATIC ${HEADER_FILES} ${SOURCE_FILES})

else()

    file(GLOB SRCS "src/*.cpp" "src/*.c")
    # Always include common files used by all displays
    file(GLOB COMMON_SOURCES "src/lcd/lcd_common.c")
    file(GLOB INTF_SRCS "src/intf/*.cpp" "src/intf/*.c")
    file(GLOB COMMON_HAL_SRCS "src/ssd1306_hal/*.cpp" "src/ssd1306_hal/*.c")
    if(CONFIG_ESP32)
        set(ESP_DEVICE "esp")
    elseif(CONFIG_ARDUINO)
        set(ESP_DEVICE "arduino")
    elseif(CONFIG_LINUX)
        set(ESP_DEVICE "linux")
    elseif(CONFIG_STM32)
        set(ESP_DEVICE "stm32")
    elseif(CONFIG_ENERGIA)
        set(ESP_DEVICE "energia")
    elseif(CONFIG_AVR)
        set(ESP_DEVICE "avr")
    elseif(CONFIG_CH32V)
        set(ESP_DEVICE "ch32v")
    elseif(CONFIG_MINGW)
        set(ESP_DEVICE "mingw")
    endif()

    file(GLOB ESP_DEVICE_HAL_SRCS "src/ssd1306_hal/${ESP_DEVICE}/*.cpp" "src/ssd1306_hal/${ESP_DEVICE}/*.c")

    # Append the common files to SRCS
    list(APPEND SRCS ${COMMON_SOURCES} ${INTF_SRCS} ${COMMON_HAL_SRCS} ${ESP_DEVICE_HAL_SRCS})

    # Include interface files
    if(CONFIG_USE_I2C_INTERFACE)
        file(GLOB I2C_SOURCE_FILES "src/intf/i2c/*.cpp" "src/intf/i2c/*.c")
        file(GLOB I2C_HEADER_FILES "src/intf/i2c/*.h")
        list(APPEND SRCS ${I2C_SOURCE_FILES})
    endif()

    if(CONFIG_USE_SPI_INTERFACE)
        file(GLOB SPI_SOURCE_FILES "src/intf/spi/*.cpp" "src/intf/spi/*.c")
        file(GLOB SPI_HEADER_FILES "src/intf/spi/*.h")
        list(APPEND SRCS ${SPI_SOURCE_FILES})
    endif()

    # Conditionally include source files based on selected display type and interface
    if(CONFIG_SSD1306)
        file(GLOB SSD1306_SOURCES "src/lcd/oled_ssd1306.c")
        list(APPEND SRCS ${SSD1306_SOURCES})
    endif()
    if(CONFIG_SH1106)
        file(GLOB SH1106_SOURCES "src/lcd/oled_sh1106.c")
        list(APPEND SRCS ${SH1106_SOURCES})
    endif()
    if(CONFIG_SSD1325)
        file(GLOB SSD1325_SOURCES "src/lcd/oled_ssd1325.c")
        list(APPEND SRCS ${SSD1325_SOURCES})
    endif()
    if(CONFIG_SSD1327)
        file(GLOB SSD1327_SOURCES "src/lcd/oled_ssd1327.c")
        list(APPEND SRCS ${SSD1327_SOURCES})
    endif()
    if(CONFIG_SSD1331)
        file(GLOB SSD1331_SOURCES "src/lcd/oled_ssd1331.c")
        list(APPEND SRCS ${SSD1331_SOURCES})
    endif()
    if(CONFIG_SSD1351)
        file(GLOB SSD1351_SOURCES "src/lcd/oled_ssd1351.c")
        list(APPEND SRCS ${SSD1351_SOURCES})
    endif()
    if(CONFIG_PCD8544)
        file(GLOB PCD8544_SOURCES "src/lcd/lcd_pcd8544.c")
        list(APPEND SRCS ${PCD8544_SOURCES})
    endif()
    if(CONFIG_ST7735)
        file(GLOB ST7735_SOURCES "src/lcd/lcd_st7735.c")
        list(APPEND SRCS ${ST7735_SOURCES})
    endif()
    if(CONFIG_ILI9341)
        file(GLOB ILI9341_SOURCES "src/lcd/lcd_ili9341.c")
        list(APPEND SRCS ${ILI9341_SOURCES})
    endif()
    if(CONFIG_IL9163)
        file(GLOB IL9163_SOURCES "src/lcd/lcd_il9163.c")
        list(APPEND SRCS ${IL9163_SOURCES})
    endif()
    if(CONFIG_VGA)
        file(GLOB VGA_SOURCES "src/lcd/vga_monitor.c")
        list(APPEND SRCS ${VGA_SOURCES})
    endif()
    if(CONFIG_COMPOSITE_VIDEO)
        file(GLOB COMPOSITE_VIDEO_SOURCES "src/lcd/composite_video.c")
        list(APPEND SRCS ${COMPOSITE_VIDEO_SOURCES})
    endif()

    idf_component_register(SRCS ${SRCS}
        INCLUDE_DIRS "src"
        PRIV_REQUIRES driver)
endif()

# Log all custom macros
message(STATUS "CONFIG_ESP32: ${CONFIG_ESP32}")
message(STATUS "ESP_DEVICE: ${ESP_DEVICE}")
message(STATUS "CONFIG_USE_I2C_INTERFACE: ${CONFIG_USE_I2C_INTERFACE}")
message(STATUS "CONFIG_USE_SPI_INTERFACE: ${CONFIG_USE_SPI_INTERFACE}")
message(STATUS "CONFIG_SSD1306: ${CONFIG_SSD1306}")
message(STATUS "COMMON_SOURCES: ${COMMON_SOURCES}")
message(STATUS "INTF_SRCS: ${INTF_SRCS}")
message(STATUS "COMMON_HAL_SRCS: ${COMMON_HAL_SRCS}")
message(STATUS "ESP_DEVICE_HAL_SRCS: ${ESP_DEVICE_HAL_SRCS}")
message(STATUS "I2C_SOURCE_FILES: ${I2C_SOURCE_FILES}")
message(STATUS "SPI_SOURCE_FILES: ${SPI_SOURCE_FILES}")
message(STATUS "SRCS: ${SRCS}")
