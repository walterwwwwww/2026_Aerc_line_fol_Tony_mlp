cmake_minimum_required(VERSION 3.22)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/arm-gnu.cmake)
project(sensor_board LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(SENSOR_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB SENSOR_SOURCES CONFIGURE_DEPENDS ${SENSOR_SOURCE_DIR}/Core/Src/*.c)
set(HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver)
set(HAL_MODULES hal hal_adc hal_adc_ex hal_cortex hal_dma hal_exti hal_flash
    hal_flash_ex hal_gpio hal_gpio_ex hal_pwr hal_rcc hal_rcc_ex hal_spi)
foreach(MODULE IN LISTS HAL_MODULES)
    list(APPEND HAL_SOURCES ${HAL_DIR}/Src/stm32f1xx_${MODULE}.c)
endforeach()
add_executable(sensor_board.elf ${SENSOR_SOURCES} ${HAL_SOURCES} startup_stm32f103xb.s)
target_include_directories(sensor_board.elf PRIVATE ${SENSOR_SOURCE_DIR}/Core/Inc
    ${HAL_DIR}/Inc ${HAL_DIR}/Inc/Legacy Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F1xx/Include)
target_compile_definitions(sensor_board.elf PRIVATE USE_HAL_DRIVER STM32F103xB DEBUG)
set(CPU_FLAGS -mcpu=cortex-m3 -mthumb -mfloat-abi=soft)
target_compile_options(sensor_board.elf PRIVATE ${CPU_FLAGS} -Og -g3
    -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(sensor_board.elf PRIVATE ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103R8_FLASH.ld
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections,--print-memory-usage,-Map=${CMAKE_BINARY_DIR}/sensor_board.map)
target_link_libraries(sensor_board.elf PRIVATE m)
set_property(TARGET sensor_board.elf APPEND PROPERTY LINK_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103R8_FLASH.ld)
add_custom_command(TARGET sensor_board.elf POST_BUILD
    COMMAND ${ARM_GNU_OBJCOPY} -O binary $<TARGET_FILE:sensor_board.elf> ${CMAKE_BINARY_DIR}/sensor_board.bin
    COMMAND ${ARM_GNU_OBJCOPY} -O ihex $<TARGET_FILE:sensor_board.elf> ${CMAKE_BINARY_DIR}/sensor_board.hex
    COMMAND ${ARM_GNU_SIZE} $<TARGET_FILE:sensor_board.elf>)
