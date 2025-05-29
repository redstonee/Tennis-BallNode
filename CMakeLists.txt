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

set(HAL_DRIVER_DIR ${CMAKE_CURRENT_LIST_DIR}/Drivers/AIR001xx_HAL_Driver)
set(CMSIS_DIR ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS)

set(USER_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(USER_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)


add_compile_definitions(
    AIR001_DEV
)


set(HAL_DRIVER_SRC
    ${HAL_DRIVER_DIR}/Src/air001xx_hal.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_cortex.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_dma.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_exti.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_gpio.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_rcc.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_tim.c
    ${HAL_DRIVER_DIR}/Src/air001xx_hal_i2c.c
)


set(CMSIS_SRC
    ${CMSIS_DIR}/Device/AIR001xx/Source/gcc/startup_air001xx.s
    ${CMSIS_DIR}/Device/AIR001xx/Source/system_air001xx.c
)

add_library(Air001_HAL_Driver INTERFACE)

target_sources(Air001_HAL_Driver INTERFACE
    ${HAL_DRIVER_SRC}
    ${CMSIS_SRC}
)


target_include_directories(Air001_HAL_Driver INTERFACE
    ${HAL_DRIVER_DIR}/Inc
    ${CMSIS_DIR}/Device/AIR001xx/Include
    ${CMSIS_DIR}/Include
)


add_executable(${OUTPUT_EXE_NAME}
    ${USER_SRC_DIR}/main.cpp
    ${USER_SRC_DIR}/syscalls.c
    ${USER_SRC_DIR}/clock.c
    ${USER_SRC_DIR}/it.c
    ${USER_SRC_DIR}/msp.c
    ${USER_SRC_DIR}/APDS9960.cpp
)


target_include_directories(${OUTPUT_EXE_NAME} PRIVATE ${USER_INCLUDE_DIR})
target_link_libraries(${OUTPUT_EXE_NAME} Air001_HAL_Driver)

set(LD_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/ldscript.ld)
target_link_options(${OUTPUT_EXE_NAME} PRIVATE
    -T "${LD_SCRIPT_PATH}"
)

add_custom_target(
        showSize ALL
        ${TOOLCHAIN_PREFIX}/bin/${TOOLCHAIN}-size "${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_EXE_NAME}"
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_EXE_NAME})

find_program(PYOCD_FOUND pyocd)

if(PYOCD_FOUND)
        add_custom_target(BuildAndFlash COMMAND DEPENDS flash-cmd)
        add_custom_command(
                USES_TERMINAL 
                OUTPUT flash-cmd
                COMMAND pyocd load ${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_EXE_NAME} --pack ${CMAKE_CURRENT_LIST_DIR}/1.1.2.pack -t air001dev
                DEPENDS showSize
        )
else()
        message(WARNING "pyocd not found, flashing will not be available")
endif()