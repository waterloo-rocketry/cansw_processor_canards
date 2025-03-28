# Hardware-in-the-Loop (HIL) tick manipulation module
add_library(hil
    hil.c
    # Uncomment to include the example
    # example_hil_usage.c
)

target_include_directories(hil
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(hil
    PUBLIC
        # Only FreeRTOS is needed for basic tick manipulation
        FreeRTOS
) 