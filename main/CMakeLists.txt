set(SRCS "app_main.cpp" "srcs/animations.cpp")
idf_component_register(SRCS ${SRCS}
INCLUDE_DIRS "." "../img" "./srcs"
REQUIRES driver lcdgfx esp_gdbstub)