cmake_minimum_required(VERSION 3.10)

# set the project name and version

set(PROJECT_NAME tftDisplay)
project(${PROJECT_NAME} VERSION 1.0)

add_executable(${PROJECT_NAME}
# z144_TFT.h
# z144_TFT.cpp
main.cpp
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

target_sources(${PROJECT_NAME} PRIVATE main.cpp)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pio hardware_interp)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
