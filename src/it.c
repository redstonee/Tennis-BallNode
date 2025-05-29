#include <air001xx_hal.h>
#include "config.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void EXTI4_15_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(APDS_INT_PIN);
}