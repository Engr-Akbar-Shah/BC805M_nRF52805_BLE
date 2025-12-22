cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Gleam-Logic)

target_sources(app PRIVATE src/main.c)

# Add the component gleam_ble
target_sources(app PRIVATE
    components/gleam_ble/gleam_ble.c)
target_include_directories(app
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/gleam_ble
)

# Add the component gleam_ledstrip
target_sources(app PRIVATE
    components/gleam_ledstrip/gleam_ledstrip.c)
target_include_directories(app
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/gleam_ledstrip
)

# Add the component gleam_bq25188
target_sources(app PRIVATE
    components/gleam_bq25188/gleam_bq25188.c)
target_include_directories(app
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/gleam_bq25188
)

# Add the component gleam_adc
target_sources(app PRIVATE
    components/gleam_adc/gleam_adc.c)
target_include_directories(app
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/gleam_adc
)