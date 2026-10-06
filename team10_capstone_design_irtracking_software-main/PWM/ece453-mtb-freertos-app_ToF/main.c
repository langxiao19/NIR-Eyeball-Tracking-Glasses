#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* ToF driver */
#include "vl53lx_api.h"
#include "vl53lx_platform.h"
#include "vl53lx_platform_init.h"

/* LSM6DSM IMU driver + PSoC6 platform layer */
#include "lsm6dsm_platform.h"

#include <stdlib.h>
#include <string.h>

/* ── Pin definitions ── */
#define I2C_SCL     P9_0
#define I2C_SDA     P9_1
#define TOF_XSHUT   P9_4
#define LED_PWM_PIN P9_6

#define MAX_INPUT_LEN 64

/* ── Global ToF device ── */
VL53LX_Dev_t dev;

/* ── PWM for LED brightness ── */
static cyhal_pwm_t led_pwm;

/* ── Measurement control ── */
static volatile bool     measuring        = false;
static volatile uint32_t measure_limit_ms = 0;

/* ── Mutex to protect printf (non-static: shared with lsm6dsm_platform.c) ── */
SemaphoreHandle_t print_mutex;

#define SAFE_PRINTF(...) do { \
    xSemaphoreTake(print_mutex, portMAX_DELAY); \
    printf(__VA_ARGS__); \
    fflush(stdout); \
    xSemaphoreGive(print_mutex); \
} while(0)

/*------------------------------------------------------------------
 * CLI command: "on [seconds]"
 *------------------------------------------------------------------*/
static BaseType_t on_handler(char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString)
{
    BaseType_t param_len;
    const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);

    if (param != NULL && param_len > 0)
    {
        char tmp[16] = {0};
        if (param_len >= (BaseType_t)sizeof(tmp))
            param_len = (BaseType_t)sizeof(tmp) - 1;
        strncpy(tmp, param, (size_t)param_len);

        int secs = atoi(tmp);
        if (secs <= 0)
        {
            snprintf(pcWriteBuffer, xWriteBufferLen,
                     "Invalid. Usage: on <seconds> or just: on\r\n");
            return pdFALSE;
        }
        measure_limit_ms = (uint32_t)secs * 1000U;
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Measuring for %d second(s). Type 'off' to stop early.\r\n", secs);
    }
    else
    {
        measure_limit_ms = 0;
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Measuring indefinitely. Type 'off' to stop.\r\n");
    }

    measuring = true;
    return pdFALSE;
}

static const CLI_Command_Definition_t on_cmd =
{
    "on",
    "on [seconds]: Start measuring (optional duration)\r\n",
    on_handler,
    -1
};

/*------------------------------------------------------------------
 * CLI command: "off"
 *------------------------------------------------------------------*/
static BaseType_t off_handler(char *pcWriteBuffer,
                              size_t xWriteBufferLen,
                              const char *pcCommandString)
{
    measuring = false;
    measure_limit_ms = 0;
    snprintf(pcWriteBuffer, xWriteBufferLen, "Measurement stopped.\r\n");
    return pdFALSE;
}

static const CLI_Command_Definition_t off_cmd =
{
    "off",
    "off: Stop measurement\r\n",
    off_handler,
    0
};

/*------------------------------------------------------------------
 * CLI command: "pwm <0-100>"
 *------------------------------------------------------------------*/
static BaseType_t pwm_handler(char *pcWriteBuffer,
                              size_t xWriteBufferLen,
                              const char *pcCommandString)
{
    BaseType_t param_len;
    const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);

    if (param == NULL || param_len <= 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: pwm <0-100>\r\n");
        return pdFALSE;
    }

    char tmp[8] = {0};
    if (param_len >= (BaseType_t)sizeof(tmp))
        param_len = (BaseType_t)sizeof(tmp) - 1;
    strncpy(tmp, param, (size_t)param_len);

    int duty = atoi(tmp);
    if (duty < 0 || duty > 100)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid value. Use 0 (off) to 100 (full brightness).\r\n");
        return pdFALSE;
    }

    cyhal_pwm_set_duty_cycle(&led_pwm, (float)duty, 1000);
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "LED brightness set to %d%%\r\n", duty);
    return pdFALSE;
}

static const CLI_Command_Definition_t pwm_cmd =
{
    "pwm",
    "pwm <0-100>: Set LED brightness via PWM duty cycle\r\n",
    pwm_handler,
    1
};

/*------------------------------------------------------------------
 * Measurement Task
 * Sensor init happens HERE, after scheduler has started,
 * so VL53LX_WaitMs and VL53LX_GetTickCount work correctly.
 *------------------------------------------------------------------*/
static void measure_task(void *pvParameters)
{
    VL53LX_Error status;

    /* ── Sensor init (must be after vTaskStartScheduler) ── */
    dev.pin_scl   = I2C_SCL;
    dev.pin_sda   = I2C_SDA;
    dev.pin_xshut = TOF_XSHUT;

    /* ── Hard reset sensor via XSHUT so every power-on starts clean ── */
    cyhal_gpio_init(TOF_XSHUT, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  /* hold low 10ms = sensor in reset */
    cyhal_gpio_write(TOF_XSHUT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  /* wait 10ms after release before talking I2C */
    cyhal_gpio_free(TOF_XSHUT);     /* release pin so platform_init can manage it */

    status = VL53LX_platform_init(&dev, 0x29, 0, 400);
    if (status != VL53LX_ERROR_NONE) {
        SAFE_PRINTF("Failed to initialize platform: %d\r\n", status);
        vTaskDelete(NULL);
    }

    SAFE_PRINTF("Waiting for sensor boot...\r\n");
    status = VL53LX_WaitDeviceBooted(&dev);
    if (status != VL53LX_ERROR_NONE) {
        SAFE_PRINTF("Failed to wait for device boot: %d\r\n", status);
        vTaskDelete(NULL);
    }
    SAFE_PRINTF("Booted\r\n");

    status = VL53LX_DataInit(&dev);
    if (status != VL53LX_ERROR_NONE) {
        SAFE_PRINTF("Failed to initialize data: %d\r\n", status);
        vTaskDelete(NULL);
    }

    status = VL53LX_SetDistanceMode(&dev, VL53LX_DISTANCEMODE_LONG);
    if (status != VL53LX_ERROR_NONE) {
        SAFE_PRINTF("Failed to set distance mode: %d\r\n", status);
        vTaskDelete(NULL);
    }

    status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&dev, 50000);
    if (status != VL53LX_ERROR_NONE) {
        SAFE_PRINTF("Failed to set timing budget: %d\r\n", status);
        vTaskDelete(NULL);
    }

    status = VL53LX_StartMeasurement(&dev);
    if (status != VL53LX_ERROR_NONE) {
        SAFE_PRINTF("Failed to start measurement: %d\r\n", status);
        vTaskDelete(NULL);
    }

    SAFE_PRINTF("Sensor ready. Type 'on' to start measuring.\r\n> ");

    /* ── Main measurement loop ── */
    for (;;)
    {
        if (measuring)
        {
            uint32_t elapsed_ms = 0;
            uint32_t limit_ms   = measure_limit_ms;

            while (measuring)
            {
                /* Check time limit */
                if (limit_ms > 0 && elapsed_ms >= limit_ms)
                {
                    measuring = false;
                    SAFE_PRINTF("\r\nMeasurement complete.\r\n> ");
                    break;
                }

                uint8_t ready = 0;
                status = VL53LX_GetMeasurementDataReady(&dev, &ready);
                if (status != VL53LX_ERROR_NONE) {
                    SAFE_PRINTF("GetMeasurementDataReady error: %d\r\n", status);
                    measuring = false;
                    break;
                }

                if (ready)
                {
                    VL53LX_MultiRangingData_t data;
                    status = VL53LX_GetMultiRangingData(&dev, &data);
                    if (status != VL53LX_ERROR_NONE) {
                        SAFE_PRINTF("GetMultiRangingData error: %d\r\n", status);
                        measuring = false;
                        break;
                    }

                    /* Same as original working code */
                    SAFE_PRINTF("Distance: %d mm\r\n",
                                data.RangeData[0].RangeMilliMeter);

                    VL53LX_ClearInterruptAndStartMeasurement(&dev);
                    elapsed_ms += 100;
                }

                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/*------------------------------------------------------------------
 * CLI Task
 *------------------------------------------------------------------*/
static void cli_task(void *pvParameters)
{
    char    input_buf[MAX_INPUT_LEN];
    char   *output_buf = FreeRTOS_CLIGetOutputBuffer();
    uint8_t input_idx  = 0;
    uint8_t c;

    for (;;)
    {
        cyhal_uart_getc(&cy_retarget_io_uart_obj, &c, portMAX_DELAY);

        if (c == '\r' || c == '\n')
        {
            SAFE_PRINTF("\r\n");

            if (input_idx > 0)
            {
                input_buf[input_idx] = '\0';
                input_idx = 0;

                BaseType_t more;
                do {
                    more = FreeRTOS_CLIProcessCommand(input_buf,
                                                      output_buf,
                                                      configCOMMAND_INT_MAX_OUTPUT_SIZE);
                    SAFE_PRINTF("%s", output_buf);
                } while (more != pdFALSE);
            }

            SAFE_PRINTF("> ");
        }
        else if ((c == '\b' || c == 127) && input_idx > 0)
        {
            input_idx--;
            xSemaphoreTake(print_mutex, portMAX_DELAY);
            printf("\b \b");
            fflush(stdout);
            xSemaphoreGive(print_mutex);
        }
        else if (input_idx < (MAX_INPUT_LEN - 1) && c >= 0x20)
        {
            input_buf[input_idx++] = (char)c;
            xSemaphoreTake(print_mutex, portMAX_DELAY);
            printf("%c", c);
            fflush(stdout);
            xSemaphoreGive(print_mutex);
        }
    }
}

/*------------------------------------------------------------------
 * main
 *------------------------------------------------------------------*/
int main(void)
{
    cy_rslt_t result;

    /* ── Board init ── */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    __enable_irq();

    /* ── UART init ── */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                        CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\nVL53L3CX + FreeRTOS CLI (PSoC6)\n");
    printf("Commands: on [seconds] | off | help\r\n");

    /* ── Create mutex ── */
    print_mutex = xSemaphoreCreateMutex();
    configASSERT(print_mutex != NULL);

    /* ── PWM init (P9_6, 1 kHz, start at 0%) ── */
    result = cyhal_pwm_init(&led_pwm, LED_PWM_PIN, NULL);
    if (result == CY_RSLT_SUCCESS)
    {
        cyhal_pwm_set_duty_cycle(&led_pwm, 0.0f, 1000);
        cyhal_pwm_start(&led_pwm);
    }

    /* ── LSM6DSM IMU init (Module 0: P10_0/P10_1, before scheduler) ── */
    lsm6dsm_platform_init();

    /* ── Register CLI commands ── */
    FreeRTOS_CLIRegisterCommand(&on_cmd);
    FreeRTOS_CLIRegisterCommand(&off_cmd);
    FreeRTOS_CLIRegisterCommand(&pwm_cmd);

    /* ── Create tasks ── */
    xTaskCreate(measure_task,
                "Measure",
                configMINIMAL_STACK_SIZE * 16,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(cli_task,
                "CLI",
                configMINIMAL_STACK_SIZE * 8,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(task_imu,
                "IMU",
                configMINIMAL_STACK_SIZE * 8,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    /* ── Start FreeRTOS scheduler ── */
    vTaskStartScheduler();

    for (;;) {}
}