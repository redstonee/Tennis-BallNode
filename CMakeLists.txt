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


set(ARDUINO_DIR ${CMAKE_CURRENT_LIST_DIR}/arduino)
set(ARDUINO_VARIANT_DIR ${ARDUINO_DIR}/variants/AIR001/AIR001_DEV)
set(ARDUINO_CORE_DIR ${ARDUINO_DIR}/cores/AirMCU)
set(BSP_DRIVER_DIR ${ARDUINO_DIR}/system)
set(AIR001_HAL_DIR ${ARDUINO_DIR}/system/Air001-Drivers/AIR001xx_HAL_Driver)
set(SRCWRAPPER_DIR ${ARDUINO_DIR}/libraries/SrcWrapper)

set(USER_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(USER_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)


add_compile_definitions(
    AIR001xx
    AIR001_DEV
    ARDUINO_AIR001_DEV
    VDD_3V3
    VARIANT_H="variant_generic.h"
    AIR001xx_HSI_16M_HCLK_16M
    F_CPU=16000000
    AIR001xx_LSC_LSI
)

# file(GLOB SRCWRAPPER_SRC
#     ${SRCWRAPPER_DIR}/src/*.c
#     ${SRCWRAPPER_DIR}/src/*.cpp
#     ${SRCWRAPPER_DIR}/src/air/*.c
#     ${SRCWRAPPER_DIR}/src/air/*.cpp
# )

set(SRCWRAPPER_SRC
    ${SRCWRAPPER_DIR}/src/syscalls.c
    ${SRCWRAPPER_DIR}/src/HardwareTimer.cpp
    ${SRCWRAPPER_DIR}/src/air/air_def.c
    ${SRCWRAPPER_DIR}/src/air/analog.cpp
    ${SRCWRAPPER_DIR}/src/air/bootloader.c
    ${SRCWRAPPER_DIR}/src/air/clock.c
    ${SRCWRAPPER_DIR}/src/air/core_callback.c
    ${SRCWRAPPER_DIR}/src/air/dwt.c
    ${SRCWRAPPER_DIR}/src/air/hw_config.c
    ${SRCWRAPPER_DIR}/src/air/interrupt.cpp
    ${SRCWRAPPER_DIR}/src/air/otp.c
    ${SRCWRAPPER_DIR}/src/air/pinmap.c
    ${SRCWRAPPER_DIR}/src/air/PortNames.c
    ${SRCWRAPPER_DIR}/src/air/timer.c
    ${SRCWRAPPER_DIR}/src/air/uart.c
)

set(ARDUINO_CORE_SRC
    ${ARDUINO_CORE_DIR}/abi.cpp
    ${ARDUINO_CORE_DIR}/avr/dtostrf.c
    ${ARDUINO_CORE_DIR}/board.c
    ${ARDUINO_CORE_DIR}/core_debug.c
    ${ARDUINO_CORE_DIR}/HardwareSerial.cpp
    ${ARDUINO_CORE_DIR}/hooks.c
    ${ARDUINO_CORE_DIR}/IPAddress.cpp
    ${ARDUINO_CORE_DIR}/itoa.c
    ${ARDUINO_CORE_DIR}/main.cpp
    ${ARDUINO_CORE_DIR}/pins_arduino.c
    ${ARDUINO_CORE_DIR}/Print.cpp
    ${ARDUINO_CORE_DIR}/RingBuffer.cpp
    ${ARDUINO_CORE_DIR}/air/startup_airyyxx.S
    ${ARDUINO_CORE_DIR}/Stream.cpp
    ${ARDUINO_CORE_DIR}/Tone.cpp
    ${ARDUINO_CORE_DIR}/USBSerial.cpp
    ${ARDUINO_CORE_DIR}/VirtIOSerial.cpp
    ${ARDUINO_CORE_DIR}/WInterrupts.cpp
    ${ARDUINO_CORE_DIR}/wiring_analog.c
    ${ARDUINO_CORE_DIR}/wiring_digital.c
    ${ARDUINO_CORE_DIR}/wiring_pulse.cpp
    ${ARDUINO_CORE_DIR}/wiring_shift.c
    ${ARDUINO_CORE_DIR}/wiring_time.c
    ${ARDUINO_CORE_DIR}/WMath.cpp
    ${ARDUINO_CORE_DIR}/WSerial.cpp
    ${ARDUINO_CORE_DIR}/WString.cpp
)

set(ARDUINO_VARIANT_SRC
    ${ARDUINO_VARIANT_DIR}/variant_generic.cpp
    ${ARDUINO_VARIANT_DIR}/PeripheralPins.c
    ${ARDUINO_VARIANT_DIR}/generic_clock.c
)

set(AIR001_DRIVER_SRC
    ${AIR001_HAL_DIR}/Src/air001xx_hal_adc.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_adc_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_can.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_cec.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_comp.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_cortex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_crc.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_crc_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_dac.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_dac_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_dma.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_exti.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_flash.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_gpio.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_i2c.c
    # ${AIR001_HAL_DIR}/Src/air001xx_hal_i2c_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_i2s.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_irda.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_iwdg.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_led.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_lptim.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_pcd.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_pwr.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_pwr_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_rcc.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_rcc_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_rtc.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_rtc_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_smartcard.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_smartcard_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_smbus.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_spi.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_spi_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_tim.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_tim_ex.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_tsc.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_uart.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_usart.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal_wwdg.c
    ${AIR001_HAL_DIR}/Src/air001xx_hal.c
    ${BSP_DRIVER_DIR}/AIR001xx/system_air001xx.c
)

add_library(arduino INTERFACE)

target_sources(arduino INTERFACE
    ${AIR001_DRIVER_SRC}
    ${ARDUINO_CORE_SRC}
    ${ARDUINO_VARIANT_SRC}
    ${SRCWRAPPER_SRC}
)


target_include_directories(arduino INTERFACE
    ${ARDUINO_CORE_DIR}
    ${ARDUINO_CORE_DIR}/avr
    ${ARDUINO_CORE_DIR}/air
    ${ARDUINO_CORE_DIR}/air/LL
    ${ARDUINO_CORE_DIR}/air/usb
    ${ARDUINO_VARIANT_DIR}

    ${AIR001_HAL_DIR}/Inc
    ${AIR001_HAL_DIR}/Inc/Legacy
    ${BSP_DRIVER_DIR}/Air001-Drivers/CMSIS/Device/AIR001xx/Include
    ${BSP_DRIVER_DIR}/Air001-Drivers/CMSIS/Device/AIR001xx/Source/gcc
    ${BSP_DRIVER_DIR}/Air001-Drivers/CMSIS/Include
    ${BSP_DRIVER_DIR}/AIR001xx
)

target_include_directories(arduino INTERFACE
    ${AIR001_HAL_DIR}/Src
)


add_executable(${OUTPUT_EXE_NAME} ${USER_SRC_DIR}/main.cpp)
target_include_directories(${OUTPUT_EXE_NAME} PRIVATE ${USER_INCLUDE_DIR})
target_link_libraries(${OUTPUT_EXE_NAME} arduino)

set(LD_SCRIPT_PATH ${CMAKE_CURRENT_LIST_DIR}/ldscript.ld)
target_link_options(${OUTPUT_EXE_NAME} PRIVATE
    -T "${LD_SCRIPT_PATH}"
)
