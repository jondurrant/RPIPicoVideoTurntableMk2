# Add library cpp files
add_library(pico_ssd1306 STATIC)
target_sources(pico_ssd1306 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/pico-ssd1306/ssd1306.c
)

# Add include directory
target_include_directories(pico_ssd1306 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/pico-ssd1306/)

# Add the standard library to the build
target_link_libraries(pico_ssd1306 PUBLIC pico_stdlib hardware_i2c)