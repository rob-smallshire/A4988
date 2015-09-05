
add_subdirectory(elapsed)

add_avr_executable(
        stepper
        main.cpp
        stream.c
        uart.c
)

avr_target_link_libraries(stepper elapsed-lib)