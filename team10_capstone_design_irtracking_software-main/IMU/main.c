#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"


/* LSM6DSM IMU driver */
#include "imu.h"

#include <stdlib.h>
#include <string.h>

#define MAX_INPUT_LEN 64

/* ── Mutex to protect printf ── */
SemaphoreHandle_t print_mutex;

#define SAFE_PRINTF(...) do { \
    xSemaphoreTake(print_mutex, portMAX_DELAY); \
    printf(__VA_ARGS__); \
    fflush(stdout); \
    xSemaphoreGive(print_mutex); \
} while(0)

/*------------------------------------------------------------------
 * CLI command: "on <Ns>" / "off"
 *------------------------------------------------------------------*/
static BaseType_t imu_on_handler(char *pcWriteBuffer,
                                 size_t xWriteBufferLen,
                                 const char *pcCommandString)
{
    BaseType_t param_len = 0;
    const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);

    if (param == NULL || param_len <= 0)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: on <Ns> (example: on 10s)\r\n");
        return pdFALSE;
    }

    char tmp[16] = {0};
    if (param_len >= (BaseType_t)sizeof(tmp))
        param_len = (BaseType_t)sizeof(tmp) - 1;
    strncpy(tmp, param, (size_t)param_len);

    char *endptr = NULL;
    unsigned long seconds = strtoul(tmp, &endptr, 10);
    if ((endptr == tmp) || seconds == 0UL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Invalid duration. Use like: on 10s\r\n");
        return pdFALSE;
    }
    if ((*endptr != '\0') && !((*endptr == 's' || *endptr == 'S') && *(endptr + 1) == '\0'))
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Invalid unit. Use seconds, e.g. on 10s\r\n");
        return pdFALSE;
    }
    if (seconds > 3600UL)
    {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Duration too long. Max 3600s.\r\n");
        return pdFALSE;
    }

    imu_capture_start_seconds((uint32_t)seconds);
    snprintf(pcWriteBuffer, xWriteBufferLen, "IMU capture started for %lus\r\n", seconds);
    return pdFALSE;
}

static BaseType_t imu_off_handler(char *pcWriteBuffer,
                                  size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    (void)pcCommandString;
    imu_capture_stop();
    snprintf(pcWriteBuffer, xWriteBufferLen, "IMU capture stopped.\r\n");
    return pdFALSE;
}

static const CLI_Command_Definition_t imu_on_cmd =
{
    "on",
    "on <Ns>: Start IMU capture for N seconds (example: on 10s)\r\n",
    imu_on_handler,
    1
};

static const CLI_Command_Definition_t imu_off_cmd =
{
    "off",
    "off: Stop IMU capture immediately\r\n",
    imu_off_handler,
    0
};

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

    printf("\r\nIMU ready. Commands: on <Ns> | off\r\n");

    /* ── Create mutex ── */
    print_mutex = xSemaphoreCreateMutex();
    configASSERT(print_mutex != NULL);

    /* ── LSM6DSM I2C init done inside task_imu via lsm6dsm_init() ── */

    /* ── Register CLI commands ── */
    FreeRTOS_CLIRegisterCommand(&imu_on_cmd);
    FreeRTOS_CLIRegisterCommand(&imu_off_cmd);

    /* ── Create tasks ── */
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
