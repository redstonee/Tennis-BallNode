#pragma once

#include <air001xx_hal_gpio.h>
#include <air001_dev.h>

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_2

#define VSENS_PORT GPIOA
#define VSENS_PIN GPIO_PIN_4
#define VSENS_ADC_CHANNEL ADC_CHANNEL_4
#define VSENS_ADC_CHANNEL_NUM 4

#define BALL_OUT_PORT GPIOA
#define BALL_OUT_PIN GPIO_PIN_0

#define APDS_INT_PORT GPIOA
#define APDS_INT_PIN GPIO_PIN_5
#define APDS_INT_EXTI_LINE EXTI_LINE_5
#define APDS_INT_EXTI_IRQn EXTI4_15_IRQn
#define PROX_INT_HIGH 600 // Proximity level for interrupt
#define PROX_INT_LOW 0    // No far interrupt

#define I2C_PORT GPIOF
#define I2C_SDA_PIN GPIO_PIN_0
#define I2C_SCL_PIN GPIO_PIN_1
#define I2C_AF GPIO_AF12_I2C
