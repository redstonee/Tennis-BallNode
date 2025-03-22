#include "air001xx_ll_rcc.h"
#include "air001xx_ll_utils.h"
#include "air001xx_ll_bus.h"

void SystemClock_Config(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    // Low speed internal clock source
    LL_RCC_LSI_Enable();
    while (!LL_RCC_LSI_IsReady())
        ;

    // High speed internal clock source
    LL_RCC_HSI_Enable();
    LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_1);
    LL_RCC_HSI_SetCalibTrimming(LL_RCC_HSICALIBRATION_8MHz);
    LL_RCC_PLL_Disable();
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
        ;
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    LL_Init1msTick(8000000);
    LL_SetSystemCoreClock(8000000);
}