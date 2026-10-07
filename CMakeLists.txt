cmake_minimum_required(VERSION 3.22)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-gnu.cmake)
project(linfol_final LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS Core/Src/*.c)
# This file contains commented integration examples rather than application code.
list(FILTER APP_SOURCES EXCLUDE REGEX "/main_user_code\\.c$")
set(VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Vendor/STM32F4)
set(HAL_DIR ${VENDOR_DIR}/STM32F4xx_HAL_Driver)
set(HAL_MODULES hal hal_cortex hal_dma hal_dma_ex hal_exti hal_flash
    hal_flash_ex hal_flash_ramfunc hal_gpio hal_pwr hal_pwr_ex hal_rcc
    hal_rcc_ex hal_spi hal_tim hal_tim_ex hal_uart)
foreach(MODULE IN LISTS HAL_MODULES)
    list(APPEND HAL_SOURCES ${HAL_DIR}/Src/stm32f4xx_${MODULE}.c)
endforeach()
add_executable(linfol_final.elf ${APP_SOURCES} ${HAL_SOURCES} startup_stm32f407xx.s)
target_include_directories(linfol_final.elf PRIVATE Core/Inc ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy ${VENDOR_DIR}/CMSIS/Include ${VENDOR_DIR}/CMSIS/Device/ST/STM32F4xx/Include)
target_compile_definitions(linfol_final.elf PRIVATE USE_HAL_DRIVER STM32F407xx DEBUG)
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(linfol_final.elf PRIVATE ${CPU_FLAGS} -Og -g3
    -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(linfol_final.elf PRIVATE ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407_512K_FLASH.ld
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections,--print-memory-usage,-Map=${CMAKE_BINARY_DIR}/linfol_final.map
    -u _printf_float)
target_link_libraries(linfol_final.elf PRIVATE m)
set_property(TARGET linfol_final.elf APPEND PROPERTY LINK_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/STM32F407_512K_FLASH.ld)
add_custom_command(TARGET linfol_final.elf POST_BUILD
    COMMAND ${ARM_GNU_OBJCOPY} -O binary $<TARGET_FILE:linfol_final.elf> ${CMAKE_BINARY_DIR}/linfol_final.bin
    COMMAND ${ARM_GNU_OBJCOPY} -O ihex $<TARGET_FILE:linfol_final.elf> ${CMAKE_BINARY_DIR}/linfol_final.hex
    COMMAND ${ARM_GNU_SIZE} $<TARGET_FILE:linfol_final.elf>)
