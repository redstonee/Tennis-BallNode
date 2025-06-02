#include <air001xx_hal.h>

#include "BattMon.h"
#include "config.h"

namespace BattMon
{
    static ADC_HandleTypeDef adcHandle;

    bool init()
    {
        __HAL_RCC_ADC_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = VSENS_PIN,
            .Mode = GPIO_MODE_ANALOG,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW,
        };
        HAL_GPIO_Init(VSENS_PORT, &GPIO_InitStruct);

        adcHandle = {
            .Instance = ADC1,
            .Init = {
                .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1,
                .Resolution = ADC_RESOLUTION_12B,
                .DataAlign = ADC_DATAALIGN_RIGHT,
                .ScanConvMode = ADC_SCAN_DIRECTION_FORWARD,
                .EOCSelection = ADC_EOC_SINGLE_CONV,
                .LowPowerAutoWait = ENABLE,
                .ContinuousConvMode = DISABLE,
                .DiscontinuousConvMode = DISABLE,
                .ExternalTrigConv = ADC_SOFTWARE_START,
                .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
                .DMAContinuousRequests = DISABLE,
                .Overrun = ADC_OVR_DATA_OVERWRITTEN,
                .SamplingTimeCommon = ADC_SAMPLETIME_28CYCLES_5,
            }};

        if (HAL_ADC_Calibration_Start(&adcHandle) != HAL_OK)
            return false; // Calibration failed

        if (HAL_ADC_Init(&adcHandle) != HAL_OK)
            return false; // Initialization failed

        ADC_ChannelConfTypeDef chConfig{
            .Channel = VSENS_ADC_CHANNEL,
            .Rank = ADC_RANK_CHANNEL_NUMBER,
            .SamplingTime = ADC_SAMPLETIME_28CYCLES_5,
        };
        if (HAL_ADC_ConfigChannel(&adcHandle, &chConfig) != HAL_OK)
            return false; // Channel configuration failed

        return true;
    }

    float readVoltage()
    {
        HAL_ADC_Start(&adcHandle);
        HAL_ADC_PollForConversion(&adcHandle, HAL_MAX_DELAY);
        uint32_t adcValue = HAL_ADC_GetValue(&adcHandle);
        HAL_ADC_Stop(&adcHandle);

        auto voltage = (adcValue * 3.3f) / (1 << 12); // 12-bit ADC
        return voltage * VBAT_DIVIDER;                // Apply voltage divider factor
    }
}
