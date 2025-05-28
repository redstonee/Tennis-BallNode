#include <air001xx_ll_gpio.h>
#include <air001xx_ll_utils.h>
#include <air001xx_ll_bus.h>
#include <air001xx_ll_i2c.h>

#include "config.h"

extern "C"
{
    extern void SystemClock_Config(void);
}

int main()
{
    SystemClock_Config();

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

    LL_GPIO_SetPinMode(LED_PORT, LED_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(LED_PORT, LED_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(LED_PORT, LED_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinMode(VSENS_PORT, VSENS_PIN, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(VSENS_PORT, VSENS_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(VSENS_PORT, VSENS_PIN, LL_GPIO_SPEED_FREQ_HIGH);

    LL_GPIO_SetPinMode(BALL_OUT_PORT, BALL_OUT_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(BALL_OUT_PORT, BALL_OUT_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinOutputType(BALL_OUT_PORT, BALL_OUT_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(BALL_OUT_PORT, BALL_OUT_PIN, LL_GPIO_SPEED_FREQ_HIGH);

    LL_GPIO_SetPinMode(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetAFPin_0_7(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN, LL_GPIO_AF_4);

    LL_I2C_InitTypeDef i2c_init{
        .ClockSpeed = 100000,
        .DutyCycle = LL_I2C_DUTYCYCLE_2,
        .OwnAddress1 = 0x00,
        .TypeAcknowledge = LL_I2C_ACK,
    };

    LL_I2C_Init(I2C1, &i2c_init);
    LL_I2C_ConfigSpeed(I2C1, 8000000, 100000, LL_I2C_DUTYCYCLE_2);

    uint32_t lastBlinkTime = 0;
    while (1)
    {
        LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);
        LL_mDelay(100);
        LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
        LL_mDelay(900);
    }
}