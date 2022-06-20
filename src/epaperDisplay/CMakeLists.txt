cmake_minimum_required(VERSION 3.10)

# set the project name and version

set(PROJECT_NAME epaperDisplay)
project(${PROJECT_NAME} VERSION 1.0)

# add_library(DEV_config INTERFACE
# )

# target_sources(DEV_config INTERFACE
# ${CMAKE_CURRENT_LIST_DIR}/main.cpp 
# ${CMAKE_CURRENT_LIST_DIR}/DEV_Config.h 
# ${CMAKE_CURRENT_LIST_DIR}/DEV_Config.c
# )

# target_link_libraries(DEV_config INTERFACE
# pico_stdlib
# hardware_pio
# hardware_interp
# hardware_spi
# )

add_executable(${PROJECT_NAME}
main.cpp
# DEV_Config.h
# DEV_Config.c
# EPD_2in9b_V3.c
# EPD_2in9b_V3.h
# font8.c
# font12.c
# font16.c
# font20.c
# font24.c
# fonts.h
# GUI_Paint.c
# GUI_Paint.h
)

target_sources(${PROJECT_NAME} PRIVATE main.cpp)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME}
pico_stdlib
hardware_pio
hardware_interp
hardware_spi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
