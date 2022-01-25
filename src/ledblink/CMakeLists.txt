add_executable(pico_display
    main.c
    )

# pull in common dependencies
target_link_libraries(pico_display pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_display)