/*
 ******************************************************************************
 * @file    orientation_6d.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to detect 6D orientation from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI189V1
 * - NUCLEO_F411RE + STEVAL-MKI189V1
 * - DISCOVERY_SPC584B + STEVAL-MKI189V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

// #define STEVAL_MKI109V3  /* little endian */
// #define NUCLEO_F411RE    /* little endian */
// #define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static uint8_t lsm6dsm_i2c_addr = LSM6DSM_I2C_ADDR_L;
static volatile bool imu_capture_enabled = false;
static volatile TickType_t imu_capture_end_tick = 0;

/* Extern variables ----------------------------------------------------------*/
stmdev_ctx_t dev_ctx;

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

static bool lsm6dsm_probe_addr(uint8_t addr, uint8_t *whoami)
{
  uint8_t reg = LSM6DSM_WHO_AM_I;
  uint8_t val = 0;

  xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);
  cy_rslt_t rslt = cyhal_i2c_master_write(&i2c_master_obj, addr, &reg, 1, 10, false);
  if (rslt == CY_RSLT_SUCCESS)
  {
    rslt = cyhal_i2c_master_read(&i2c_master_obj, addr, &val, 1, 10, true);
  }
  xSemaphoreGive(Semaphore_I2C);

  if ((rslt == CY_RSLT_SUCCESS) && (val == LSM6DSM_ID))
  {
    if (whoami != NULL)
    {
      *whoami = val;
    }
    return true;
  }
  return false;
}

void lsm6dsm_init(void)
{
  /* Initialize mems driver interface */
  lsm6dsm_int1_route_t int_1_reg;
  
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;

  /* Init test platform */
  cy_rslt_t i2c_rslt = i2c_init(MODULE_SITE_1);
  (void)i2c_rslt;
  /* Wait sensor boot time (LSM6DSM datasheet: max 35ms) */
  platform_delay(35);

  /* Check device ID */
  whoamI = 0;
  if (lsm6dsm_probe_addr(LSM6DSM_I2C_ADDR_L, &whoamI))
  {
    lsm6dsm_i2c_addr = LSM6DSM_I2C_ADDR_L;
  }
  else if (lsm6dsm_probe_addr(LSM6DSM_I2C_ADDR_H, &whoamI))
  {
    lsm6dsm_i2c_addr = LSM6DSM_I2C_ADDR_H;
  }
  else
  {
    (void)platform_read(NULL, LSM6DSM_WHO_AM_I, &whoamI, 1);
  }

  if (whoamI != LSM6DSM_ID)
  {
    printf("IMU init failed.\r\n");
    while (1) {}
  }

  /* Restore default configuration */
  lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do
  {
    lsm6dsm_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Set XL Output Data Rate */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_416Hz);

  /* Set 2g full XL scale */
  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_2g);

  /* Set Gyro Output Data Rate */
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_416Hz);

  /* Set 2000 dps full Gyro scale */
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);

  /* Set threshold to 60 degrees */
  lsm6dsm_6d_threshold_set(&dev_ctx, LSM6DSM_DEG_60);

  /* LPF2 on 6D function selection */
  lsm6dsm_6d_feed_data_set(&dev_ctx, LSM6DSM_LPF2_FEED);

  /* Enable interrupt generation on 6D INT1 pin */
  lsm6dsm_pin_int1_route_get(&dev_ctx, &int_1_reg);

  int_1_reg.int1_6d = PROPERTY_ENABLE;
  lsm6dsm_pin_int1_route_set(&dev_ctx, int_1_reg);
  printf("IMU init OK.\r\n");
}

/* Main Example --------------------------------------------------------------*/
void lsm6dsm_orientation(void)
{
  /* Wait Events */
  while (1)
  {
    lsm6dsm_all_sources_t all_source;

    /* Check if 6D Orientation events */
    lsm6dsm_all_sources_get(&dev_ctx, &all_source);

    if (all_source.d6d_src.d6d_ia)
    {
      sprintf((char *)tx_buffer, "Orientation:  ");

      if (all_source.d6d_src.xh)
      {
        strcat((char *)tx_buffer, "XH");
      }

      if (all_source.d6d_src.xl)
      {
        strcat((char *)tx_buffer, "XL");
      }

      if (all_source.d6d_src.yh)
      {
        strcat((char *)tx_buffer, "YH");
      }

      if (all_source.d6d_src.yl)
      {
        strcat((char *)tx_buffer, "YL");
      }

      if (all_source.d6d_src.zh)
      {
        strcat((char *)tx_buffer, "ZH");
      }

      if (all_source.d6d_src.zl)
      {
        strcat((char *)tx_buffer, "ZL");
      }

      strcat((char *)tx_buffer, "\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      cyhal_system_delay_ms(50);
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  uint8_t *tx = malloc(len + 1);
  tx[0] = reg;
  memcpy(tx + 1, bufp, len);

  xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);
  cy_rslt_t rslt = cyhal_i2c_master_write(&i2c_master_obj,
                                          lsm6dsm_i2c_addr,
                                          tx, len + 1,
                                          10, true);
  xSemaphoreGive(Semaphore_I2C);
  free(tx);
  return (rslt == CY_RSLT_SUCCESS) ? 0 : -1;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

  /* Write register address, no STOP so bus stays active for repeated start */
  cy_rslt_t rslt = cyhal_i2c_master_write(&i2c_master_obj,
                                          lsm6dsm_i2c_addr,
                                          &reg, 1,
                                          10, false);
  if (rslt != CY_RSLT_SUCCESS)
  {
    xSemaphoreGive(Semaphore_I2C);
    return -1;
  }

  /* Read back the data */
  rslt = cyhal_i2c_master_read(&i2c_master_obj,
                               lsm6dsm_i2c_addr,
                               bufp, len,
                               10, true);
  xSemaphoreGive(Semaphore_I2C);
  return (rslt == CY_RSLT_SUCCESS) ? 0 : -1;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  printf("%s", tx_buffer);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  Cy_SysLib_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  i2c_init(MODULE_SITE_1);  /* P9_0=SCL, P9_1=SDA */
}

void imu_capture_start_seconds(uint32_t seconds)
{
  if (seconds == 0U)
  {
    imu_capture_enabled = false;
    return;
  }

  imu_capture_end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(seconds * 1000U);
  imu_capture_enabled = true;
}

void imu_capture_stop(void)
{
  imu_capture_enabled = false;
}

bool imu_capture_is_active(void)
{
  return imu_capture_enabled;
}

/* FreeRTOS task: init IMU then continuously read and print accel/gyro */
void task_imu(void *pvParameters)
{
  lsm6dsm_init();

  /* Verify ODR registers were written correctly */
  lsm6dsm_odr_xl_t odr_xl = LSM6DSM_XL_ODR_OFF;
  lsm6dsm_odr_g_t  odr_g  = LSM6DSM_GY_ODR_OFF;
  lsm6dsm_xl_data_rate_get(&dev_ctx, &odr_xl);
  lsm6dsm_gy_data_rate_get(&dev_ctx, &odr_g);
  printf("XL ODR readback=%d (expect 6), GY ODR readback=%d (expect 6)\r\n",
         (int)odr_xl, (int)odr_g);

  int16_t raw[3];
  float   accel_mg[3];
  float   gyro_mdps[3];

  for (;;)
  {
    if (!imu_capture_enabled)
    {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if ((int32_t)(xTaskGetTickCount() - imu_capture_end_tick) >= 0)
    {
      imu_capture_enabled = false;
      printf("IMU capture finished.\r\n");
      continue;
    }

    /* ODR=416Hz → new data every 2.4ms; 100ms poll guarantees data ready */
    lsm6dsm_acceleration_raw_get(&dev_ctx, raw);
    accel_mg[0] = lsm6dsm_from_fs2g_to_mg(raw[0]);
    accel_mg[1] = lsm6dsm_from_fs2g_to_mg(raw[1]);
    accel_mg[2] = lsm6dsm_from_fs2g_to_mg(raw[2]);
    printf("Accel [mg]: X=%.1f  Y=%.1f  Z=%.1f\r\n",
           accel_mg[0], accel_mg[1], accel_mg[2]);

    lsm6dsm_angular_rate_raw_get(&dev_ctx, raw);
    gyro_mdps[0] = lsm6dsm_from_fs2000dps_to_mdps(raw[0]);
    gyro_mdps[1] = lsm6dsm_from_fs2000dps_to_mdps(raw[1]);
    gyro_mdps[2] = lsm6dsm_from_fs2000dps_to_mdps(raw[2]);
    printf("Gyro [mdps]: X=%.1f  Y=%.1f  Z=%.1f\r\n",
           gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
