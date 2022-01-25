cmake_minimum_required(VERSION 3.10)

# set the project name and version

set(PROJECT_NAME serialcomm)
project(${PROJECT_NAME} VERSION 1.0)


add_executable(${PROJECT_NAME}
serialcomm.c
)

# pull in common dependencies
target_link_libraries(${PROJECT_NAME} pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
