cmake_minimum_required(VERSION 3.10)

# set the project name and version

set(PROJECT_NAME epaperdisplay)
project(${PROJECT_NAME} VERSION 1.0)


add_executable(${PROJECT_NAME}
main.c
DEV_Config.c
DEV_Config.h
EPD_2in9b_V3.c 
EPD_2in9b_V3.h
)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} 
pico_stdlib
hardware_pio
hardware_interp
hardware_spi)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
