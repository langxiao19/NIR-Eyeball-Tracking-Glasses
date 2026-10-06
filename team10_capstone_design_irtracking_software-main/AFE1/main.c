#include "main.h"
#include "task_console.h"

/* ── ADC for IR sensor on P10_0 ── */
static cyhal_adc_t         adc_obj;
static cyhal_adc_channel_t adc_channel;

static const cyhal_adc_channel_config_t adc_ch_config = {
    .enable_averaging   = false,
    .min_acquisition_ns = 1000u,
    .enabled            = true
};

/* ── Sampling control ── */
static volatile bool     sampling      = false;
static volatile uint32_t samples_left  = 0;   /* countdown: 5 samples/sec * N sec */

/*------------------------------------------------------------------
 * ADC sampling task — runs continuously, samples at 5 Hz when active
 *------------------------------------------------------------------*/
static void task_adc(void *pvParameters)
{
    for (;;)
    {
        if (sampling && samples_left > 0)
        {
            int32_t mv  = cyhal_adc_read_uv(&adc_channel) / 1000;
            int32_t dev = mv - 1650;
            task_print("IR: %ld mV  (offset: %+ld mV)\r\n", (long)mv, (long)dev);

            samples_left--;
            if (samples_left == 0)
            {
                sampling = false;
                task_print("Done.\r\n");
            }
            vTaskDelay(pdMS_TO_TICKS(200));   /* 200 ms = 5 Hz */
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/*------------------------------------------------------------------
 * CLI command: "on <seconds>"
 *------------------------------------------------------------------*/
static BaseType_t on_handler(char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString)
{
    BaseType_t param_len;
    const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);

    if (param == NULL || param_len <= 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: on <seconds>\r\n");
        return pdFALSE;
    }

    char tmp[16] = {0};
    if (param_len >= (BaseType_t)sizeof(tmp))
        param_len = (BaseType_t)sizeof(tmp) - 1;
    strncpy(tmp, param, (size_t)param_len);

    int secs = atoi(tmp);
    if (secs <= 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: on <seconds>\r\n");
        return pdFALSE;
    }

    samples_left = (uint32_t)secs * 5;
    sampling     = true;
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "Sampling for %d s at 5 Hz (%d samples). Type 'off' to stop.\r\n",
             secs, secs * 5);
    return pdFALSE;
}

static const CLI_Command_Definition_t on_cmd = {
    "on",
    "on <seconds>: Sample IR sensor at 5 Hz for N seconds\r\n",
    on_handler,
    1
};

/*------------------------------------------------------------------
 * CLI command: "off"
 *------------------------------------------------------------------*/
static BaseType_t off_handler(char *pcWriteBuffer,
                              size_t xWriteBufferLen,
                              const char *pcCommandString)
{
    sampling     = false;
    samples_left = 0;
    snprintf(pcWriteBuffer, xWriteBufferLen, "Sampling stopped.\r\n");
    return pdFALSE;
}

static const CLI_Command_Definition_t off_cmd = {
    "off",
    "off: Stop sampling\r\n",
    off_handler,
    0
};

/*------------------------------------------------------------------
 * main
 *------------------------------------------------------------------*/
int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    __enable_irq();

    /* ── ADC init on P10_0 (single-ended vs GND) ── */
    result = cyhal_adc_init(&adc_obj, P10_0, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cyhal_adc_channel_init_diff(&adc_channel, &adc_obj,
                                         P10_0, CYHAL_ADC_VNEG,
                                         &adc_ch_config);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* ── Register CLI commands ── */
    FreeRTOS_CLIRegisterCommand(&on_cmd);
    FreeRTOS_CLIRegisterCommand(&off_cmd);

    /* ── Start console (UART + CLI tasks) ── */
    task_console_init();

    /* ── ADC sampling task ── */
    xTaskCreate(task_adc, "ADC", configMINIMAL_STACK_SIZE * 4,
                NULL, configMAX_PRIORITIES - 7, NULL);

    vTaskStartScheduler();

    for (;;) {}
}
