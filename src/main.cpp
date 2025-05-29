#include <air001xx_hal.h>
#include "APDS9930.h"

#include "config.h"

extern "C"
{
    extern void SystemClock_Config(void);
}

void onError()
{
    // Error handling function
    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
}

volatile bool sensorInterruptFlag = false;
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

    GPIO_InitStruct.Pin = APDS_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(APDS_INT_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(APDS_INT_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(APDS_INT_EXTI_IRQn);

    APDS9930 sensor;
    auto sensorOK = sensor.init(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_AF);
    // Set proximity interrupt thresholds
    sensorOK = sensor.setProximityIntLowThreshold(PROX_INT_LOW);
    sensorOK = sensor.setProximityIntHighThreshold(PROX_INT_HIGH);
    // Start running the APDS-9930 proximity sensor (interrupts)
    sensorOK = sensor.enableProximitySensor(true);
    if (!sensorOK)
        onError();

    uint32_t lastBlinkTime = HAL_GetTick();
    while (1)
    {
        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        HAL_ResumeTick();

        if (sensorInterruptFlag)
        {
            HAL_GPIO_WritePin(BALL_OUT_PORT, BALL_OUT_PIN, GPIO_PIN_SET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(BALL_OUT_PORT, BALL_OUT_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

            uint16_t proximityValue;
            do
            {
                if (!sensor.readProximity(proximityValue))
                    onError();

                HAL_Delay(100);
            } while (proximityValue > PROX_INT_HIGH);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

            sensor.clearProximityInt();
            sensorInterruptFlag = false;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    UNUSED(pin);
    sensorInterruptFlag = true;
}