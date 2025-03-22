cmake_minimum_required(VERSION 3.21)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/toolchain-arm-none-eabi.cmake)

project(NooBot-PowerModule)

enable_language(ASM)
enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OUTPUT_EXE_NAME ${CMAKE_PROJECT_NAME}.elf)

set(DRIVER_DIR ${CMAKE_CURRENT_LIST_DIR}/Drivers/AIR001xx_HAL_Driver)
set(CMSIS_DIR ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS)

set(USER_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(USER_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)


add_compile_definitions(
    AIR001_DEV
    USE_FULL_LL_DRIVER
)


set(LL_DRIVER_SRC
    ${DRIVER_DIR}/Src/air001xx_ll_adc.c
    ${DRIVER_DIR}/Src/air001xx_ll_comp.c
    ${DRIVER_DIR}/Src/air001xx_ll_crc.c
    ${DRIVER_DIR}/Src/air001xx_ll_crs.c
    ${DRIVER_DIR}/Src/air001xx_ll_dac.c
    ${DRIVER_DIR}/Src/air001xx_ll_dma.c
    ${DRIVER_DIR}/Src/air001xx_ll_exti.c
    ${DRIVER_DIR}/Src/air001xx_ll_gpio.c
    ${DRIVER_DIR}/Src/air001xx_ll_i2c.c
    ${DRIVER_DIR}/Src/air001xx_ll_led.c
    ${DRIVER_DIR}/Src/air001xx_ll_lptim.c
    ${DRIVER_DIR}/Src/air001xx_ll_pwr.c
    ${DRIVER_DIR}/Src/air001xx_ll_rcc.c
    ${DRIVER_DIR}/Src/air001xx_ll_rtc.c
    ${DRIVER_DIR}/Src/air001xx_ll_spi.c
    ${DRIVER_DIR}/Src/air001xx_ll_tim.c
    ${DRIVER_DIR}/Src/air001xx_ll_usart.c
    # ${DRIVER_DIR}/Src/air001xx_ll_usb.c
    ${DRIVER_DIR}/Src/air001xx_ll_utils.c
)

set(CMSIS_SRC
    ${CMSIS_DIR}/Device/AIR001xx/Source/gcc/startup_air001xx.s
    ${CMSIS_DIR}/Device/AIR001xx/Source/system_air001xx.c
)

add_library(Air001_LL_Driver INTERFACE)

target_sources(Air001_LL_Driver INTERFACE
    ${LL_DRIVER_SRC}
    ${CMSIS_SRC}
)


target_include_directories(Air001_LL_Driver INTERFACE
    ${DRIVER_DIR}/Inc
    ${CMSIS_DIR}/Device/AIR001xx/Include
    ${CMSIS_DIR}/Include
)


add_executable(${OUTPUT_EXE_NAME}
    ${USER_SRC_DIR}/main.cpp
    ${USER_SRC_DIR}/syscalls.c
    ${USER_SRC_DIR}/clock.c
)


target_include_directories(${OUTPUT_EXE_NAME} PRIVATE ${USER_INCLUDE_DIR})
target_link_libraries(${OUTPUT_EXE_NAME} Air001_LL_Driver)

set(LD_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/ldscript.ld)
target_link_options(${OUTPUT_EXE_NAME} PRIVATE
    -T "${LD_SCRIPT_PATH}"
)
