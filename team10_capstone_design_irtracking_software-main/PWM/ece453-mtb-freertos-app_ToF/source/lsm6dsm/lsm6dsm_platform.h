#ifndef __LSM6DSM_PLATFORM_H__
#define __LSM6DSM_PLATFORM_H__

/* PSoC6 HAL */
#include "cyhal.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ST driver */
#include "lsm6dsm_reg.h"

/* Project I2C layer (provides i2c_master_obj, i2c_init, Semaphore_I2C) */
#include "i2c.h"

/*
 * 7-bit I2C address for PSoC6 HAL (cyhal_i2c_master_write/read).
 * LSM6DSM_I2C_ADD_L = 0xD5 is the 8-bit format (SA0=0, R/W bit included).
 * Shift right by 1 to get the 7-bit form used by the HAL.
 */
#define LSM6DSM_I2C_ADDR_7BIT   0x6Au   /* SA0 pin = 0; use 0x6B if SA0 = 1 */

/*
 * print_mutex is defined in main.c (non-static so it can be shared here).
 * IMU_PRINTF wraps printf with the same mutex used by SAFE_PRINTF in main.c,
 * preventing interleaved output when both tasks print concurrently.
 */
extern SemaphoreHandle_t print_mutex;

#define IMU_PRINTF(...) do {                           \
    xSemaphoreTake(print_mutex, portMAX_DELAY);        \
    printf(__VA_ARGS__);                               \
    fflush(stdout);                                    \
    xSemaphoreGive(print_mutex);                       \
} while (0)

/* LSM6DSM driver context — populated by lsm6dsm_platform_init().
 * Pass &imu_ctx (or imu_ctx directly) to every lsm6dsm_*() API call. */
extern stmdev_ctx_t imu_ctx;

/*
 * lsm6dsm_platform_init()
 *   Call from main() BEFORE vTaskStartScheduler().
 *   Initialises I2C (Module 0: P10_0=SCL, P10_1=SDA, 100 kHz),
 *   wires the stmdev_ctx_t callbacks, verifies WHO_AM_I, performs
 *   software reset, then configures accel/gyro for basic operation.
 *   Returns CY_RSLT_SUCCESS on success, non-zero on any failure.
 */
cy_rslt_t lsm6dsm_platform_init(void);

/*
 * task_imu()
 *   FreeRTOS task entry point.  Polls accel and gyro data-ready flags
 *   every 10 ms and prints converted values via IMU_PRINTF.
 *   Create with xTaskCreate() after lsm6dsm_platform_init() succeeds.
 */
void task_imu(void *pvParameters);

#endif /* __LSM6DSM_PLATFORM_H__ */
