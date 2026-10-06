#include "lsm6dsm_platform.h"
#include <string.h>
#include <stdio.h>

/* Exported context — used by all lsm6dsm_*() API calls */
stmdev_ctx_t imu_ctx;

/* ── I2C write callback ─────────────────────────────────────────────────────
 * Required signature: int32_t fn(void *handle, uint8_t reg,
 *                                const uint8_t *data, uint16_t len)
 *
 * Builds a single I2C transaction: [reg_addr(1 byte), data[0..len-1]]
 * and calls cyhal_i2c_master_write with send_stop = true.
 *
 * Pattern from vl53lx_platform.c VL53LX_WriteMulti(), adapted for
 * 8-bit register address (VL53LX uses 16-bit, LSM6DSM uses 8-bit).
 * ────────────────────────────────────────────────────────────────────────── */
static int32_t psoc6_lsm6dsm_write(void *handle, uint8_t reg,
                                    const uint8_t *data, uint16_t len)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    cy_rslt_t rslt = cyhal_i2c_master_write(
        (cyhal_i2c_t *)handle,
        LSM6DSM_I2C_ADDR_7BIT,
        buf,
        (uint16_t)(len + 1),
        1000,
        true);   /* send_stop = true */

    return (rslt == CY_RSLT_SUCCESS) ? 0 : -1;
}

/* ── I2C read callback ──────────────────────────────────────────────────────
 * Required signature: int32_t fn(void *handle, uint8_t reg,
 *                                uint8_t *data, uint16_t len)
 *
 * Writes the 1-byte register address with send_stop = false (repeated start),
 * then reads len bytes.
 *
 * Pattern from vl53lx_platform.c VL53LX_ReadMulti(), same two-phase
 * write-then-read sequence, register address narrowed to 1 byte.
 * ────────────────────────────────────────────────────────────────────────── */
static int32_t psoc6_lsm6dsm_read(void *handle, uint8_t reg,
                                   uint8_t *data, uint16_t len)
{
    cy_rslt_t rslt;

    rslt = cyhal_i2c_master_write(
        (cyhal_i2c_t *)handle,
        LSM6DSM_I2C_ADDR_7BIT,
        &reg,
        1,
        1000,
        false);  /* send_stop = false → repeated start before read */

    if (rslt != CY_RSLT_SUCCESS)
        return -1;

    rslt = cyhal_i2c_master_read(
        (cyhal_i2c_t *)handle,
        LSM6DSM_I2C_ADDR_7BIT,
        data,
        (uint16_t)len,
        1000,
        true);   /* send_stop = true */

    return (rslt == CY_RSLT_SUCCESS) ? 0 : -1;
}

/* ── Millisecond delay callback ─────────────────────────────────────────────
 * FreeRTOS-aware: uses vTaskDelay when the scheduler is running so the
 * CPU is not wasted busy-waiting; falls back to HAL busy-wait before
 * the scheduler starts (e.g. during lsm6dsm_platform_init).
 *
 * Identical pattern to VL53LX_WaitMs() in vl53lx_platform.c:192.
 * ────────────────────────────────────────────────────────────────────────── */
static void psoc6_lsm6dsm_mdelay(uint32_t ms)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        vTaskDelay(pdMS_TO_TICKS(ms));
    else
        cyhal_system_delay_ms(ms);
}

/* ── Platform init ──────────────────────────────────────────────────────────
 * Call from main() BEFORE vTaskStartScheduler().
 *
 * I2C bus: Module 0 (P10_0 = SCL, P10_1 = SDA, 100 kHz).
 *   - Reuses i2c_init(MODULE_SITE_0) from source/app_hw/i2c.c.
 *   - i2c_master_obj and Semaphore_I2C are defined in i2c.c (extern via i2c.h).
 *   - Module 1 (P9_0/P9_1) is already occupied by the VL53L3CX ToF sensor.
 *
 * Sequence matches ST application notes for LSM6DSM startup:
 *   1. I2C init  2. Context wiring  3. WHO_AM_I  4. SW reset
 *   5. BDU       6. Accel config    7. Gyro config
 * ────────────────────────────────────────────────────────────────────────── */
cy_rslt_t lsm6dsm_platform_init(void)
{
    cy_rslt_t rslt;
    int32_t   ret;
    uint8_t   id      = 0;
    uint8_t   rst     = 0;
    uint32_t  timeout = 100;   /* max 100 ms for reset to clear */

    /* Step 1 — I2C bus on Module 0 (P10_0 = SCL, P10_1 = SDA, 100 kHz) */
    rslt = i2c_init(MODULE_SITE_0);
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("LSM6DSM: I2C init failed (0x%08lX)\r\n", (unsigned long)rslt);
        return rslt;
    }

    /* Step 2 — Wire context callbacks to PSoC6 implementations */
    imu_ctx.write_reg = psoc6_lsm6dsm_write;
    imu_ctx.read_reg  = psoc6_lsm6dsm_read;
    imu_ctx.mdelay    = psoc6_lsm6dsm_mdelay;
    imu_ctx.handle    = &i2c_master_obj;   /* extern from i2c.h */

    /* Step 3 — WHO_AM_I: verify the correct chip is reachable */
    ret = lsm6dsm_device_id_get(&imu_ctx, &id);
    if (ret != 0 || id != LSM6DSM_ID)
    {
        printf("LSM6DSM: WHO_AM_I failed (got 0x%02X, expected 0x%02X)\r\n",
               id, LSM6DSM_ID);
        return (cy_rslt_t)0xFFFF0001u;
    }
    printf("LSM6DSM: WHO_AM_I OK (0x%02X)\r\n", id);

    /* Step 4 — Software reset; poll until the bit self-clears */
    lsm6dsm_reset_set(&imu_ctx, PROPERTY_ENABLE);
    do {
        psoc6_lsm6dsm_mdelay(1);
        lsm6dsm_reset_get(&imu_ctx, &rst);
    } while (rst && --timeout);

    /* Step 5 — Block Data Update: prevents reading a mix of old and new bytes */
    lsm6dsm_block_data_update_set(&imu_ctx, PROPERTY_ENABLE);

    /* Steps 6-9 — Accel ±2 g @ 104 Hz, Gyro ±2000 dps @ 104 Hz */
    lsm6dsm_xl_full_scale_set(&imu_ctx, LSM6DSM_2g);
    lsm6dsm_gy_full_scale_set(&imu_ctx, LSM6DSM_2000dps);
    lsm6dsm_xl_data_rate_set(&imu_ctx,  LSM6DSM_XL_ODR_104Hz);
    lsm6dsm_gy_data_rate_set(&imu_ctx,  LSM6DSM_GY_ODR_104Hz);

    printf("LSM6DSM: Init complete (Accel +-2g, Gyro +-2000dps, ODR 104Hz)\r\n");
    return CY_RSLT_SUCCESS;
}

/* ── IMU FreeRTOS task ──────────────────────────────────────────────────────
 * Polls accel and gyro data-ready flags every 10 ms.
 * Converts raw LSBs using datasheet sensitivity:
 *   Accel ±2 g      →  0.061 mg / LSB
 *   Gyro  ±2000 dps → 70.0  mdps / LSB
 * Output is guarded by print_mutex (same mutex as SAFE_PRINTF in main.c).
 * ────────────────────────────────────────────────────────────────────────── */
void task_imu(void *pvParameters)
{
    (void)pvParameters;

    int16_t accel[3];
    int16_t gyro[3];
    uint8_t xl_rdy = 0;
    uint8_t gy_rdy = 0;

    for (;;)
    {
        lsm6dsm_xl_flag_data_ready_get(&imu_ctx, &xl_rdy);
        if (xl_rdy)
        {
            lsm6dsm_acceleration_raw_get(&imu_ctx, accel);
            IMU_PRINTF("Accel [mg]:  X=%6d  Y=%6d  Z=%6d\r\n",
                       (int)(accel[0] * 0.061f),
                       (int)(accel[1] * 0.061f),
                       (int)(accel[2] * 0.061f));
        }

        lsm6dsm_gy_flag_data_ready_get(&imu_ctx, &gy_rdy);
        if (gy_rdy)
        {
            lsm6dsm_angular_rate_raw_get(&imu_ctx, gyro);
            IMU_PRINTF("Gyro [mdps]: X=%6d  Y=%6d  Z=%6d\r\n",
                       (int)(gyro[0] * 70.0f),
                       (int)(gyro[1] * 70.0f),
                       (int)(gyro[2] * 70.0f));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
