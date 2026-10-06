// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "cyhal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "vl53lx_platform.h"
#include "vl53lx_platform_log.h"

#define trace_print(level, ...) \
    _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
    level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
    _LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
    VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

/* Global I2C object handle */
cyhal_i2c_t i2c_monarch_obj;


VL53LX_Error VL53LX_CommsInitialise(
    VL53LX_Dev_t *pdev,
    uint8_t       comms_type,
    uint16_t      comms_speed_khz)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;

    cyhal_i2c_cfg_t i2c_config =
    {
        CYHAL_I2C_MODE_MASTER,
        0,
        (uint32_t)comms_speed_khz * 1000U
    };

    cy_rslt_t rslt = cyhal_i2c_init(&i2c_monarch_obj, pdev->pin_sda, pdev->pin_scl, NULL);
    if (rslt != CY_RSLT_SUCCESS)
        return VL53LX_ERROR_UNDEFINED;

    rslt = cyhal_i2c_configure(&i2c_monarch_obj, &i2c_config);
    if (rslt != CY_RSLT_SUCCESS)
        return VL53LX_ERROR_UNDEFINED;

    pdev->i2c = &i2c_monarch_obj;

    return status;
}


VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WriteMulti(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata,
    uint32_t      count)
{
    uint8_t buffer[count + 2];

    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    memcpy(&buffer[2], pdata, count);

    cy_rslt_t rslt = cyhal_i2c_master_write(
        pdev->i2c,
        pdev->i2c_slave_address,
        buffer,
        (uint16_t)(count + 2),
        1000,
        true);

    if (rslt != CY_RSLT_SUCCESS)
        return VL53LX_ERROR_UNDEFINED;

    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_ReadMulti(
    VL53LX_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata,
    uint32_t      count)
{
    uint8_t index_buffer[2];
    index_buffer[0] = (uint8_t)(index >> 8);
    index_buffer[1] = (uint8_t)(index & 0xFF);

    cy_rslt_t rslt = cyhal_i2c_master_write(
        pdev->i2c,
        pdev->i2c_slave_address,
        index_buffer,
        2,
        1000,
        false);  /* no stop - repeated start for read */

    if (rslt != CY_RSLT_SUCCESS)
        return VL53LX_ERROR_UNDEFINED;

    rslt = cyhal_i2c_master_read(
        pdev->i2c,
        pdev->i2c_slave_address,
        pdata,
        (uint16_t)count,
        1000,
        true);

    if (rslt != CY_RSLT_SUCCESS)
        return VL53LX_ERROR_UNDEFINED;

    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t VL53LX_p_003)
{
    uint8_t buffer[1];
    buffer[0] = VL53LX_p_003;
    return VL53LX_WriteMulti(pdev, index, buffer, 1);
}


VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t VL53LX_p_003)
{
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
    buffer[1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
    return VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);
}


VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t VL53LX_p_003)
{
    uint8_t buffer[4];
    buffer[0] = (uint8_t) (VL53LX_p_003 >> 24);
    buffer[1] = (uint8_t)((VL53LX_p_003 & 0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((VL53LX_p_003 & 0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (VL53LX_p_003 & 0x000000FF);
    return VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);
}


VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata)
{
    uint8_t buffer[1];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 1);
    *pdata = buffer[0];
    return status;
}


VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *pdata)
{
    uint8_t buffer[2];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);
    *pdata = (uint16_t)(((uint16_t)buffer[0] << 8) + buffer[1]);
    return status;
}


VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *pdata)
{
    uint8_t buffer[4];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0] << 24)
           + ((uint32_t)buffer[1] << 16)
           + ((uint32_t)buffer[2] << 8)
           +  (uint32_t)buffer[3];
    return status;
}


VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us)
{
    cyhal_system_delay_us((uint32_t)wait_us);
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms)
{
    /* Use FreeRTOS delay if scheduler is running, otherwise use HAL */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
    else
        cyhal_system_delay_ms((uint32_t)wait_ms);

    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GpioInterruptDisable(void)
{
    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GetTickCount(
    VL53LX_Dev_t *pdev,
    uint32_t     *ptick_count_ms)
{
    (void)pdev;

    /* Works both before and after scheduler starts:
     * - Before: FreeRTOS tick counter is 0, Cy_SysTick gives real time
     * - After:  xTaskGetTickCount() is accurate (1 tick = 1ms per FreeRTOSConfig.h) */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        *ptick_count_ms = (uint32_t)xTaskGetTickCount();
    else
        *ptick_count_ms = (uint32_t)(Cy_SysLib_GetUniqueId() & 0xFFFFFFFF); /* fallback */

    return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t      timeout_ms,
    uint16_t      index,
    uint8_t       value,
    uint8_t       mask,
    uint32_t      poll_delay_ms)
{
    VL53LX_Error status         = VL53LX_ERROR_NONE;
    uint32_t     start_time_ms  = 0;
    uint32_t     current_time_ms = 0;
    uint8_t      byte_value     = 0;
    uint8_t      found          = 0;

    VL53LX_GetTickCount(pdev, &start_time_ms);
    pdev->new_data_ready_poll_duration_ms = 0;

    while ((status == VL53LX_ERROR_NONE) &&
           (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
           (found == 0))
    {
        status = VL53LX_WaitMs(pdev, (int32_t)poll_delay_ms);

        status = VL53LX_RdByte(pdev, index, &byte_value);

        if ((byte_value & mask) == value)
            found = 1;

        VL53LX_GetTickCount(pdev, &current_time_ms);
        pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
    }

    if (found == 0 && status == VL53LX_ERROR_NONE)
        status = VL53LX_ERROR_TIME_OUT;

    return status;
}