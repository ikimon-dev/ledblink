add_executable(pico_display
    main.c
    )

# pull in common dependencies
target_link_libraries(pico_display pico_stdlib)

