#include <air001xx_hal.h>
#include "APDS9960.h"

#include "config.h"

extern "C"
{
    extern void SystemClock_Config(void);
}

int main()
{
    SystemClock_Config();
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = LED_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VSENS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(VSENS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BALL_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(BALL_OUT_PORT, &GPIO_InitStruct);

    // SparkFun_APDS9960 sensor;
    // sensor.init(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_AF);
    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
}