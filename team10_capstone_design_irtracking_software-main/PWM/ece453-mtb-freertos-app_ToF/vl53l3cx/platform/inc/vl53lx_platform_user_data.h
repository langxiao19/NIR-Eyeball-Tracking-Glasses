
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */



#ifndef _VL53LX_PLATFORM_USER_DATA_H_
#define _VL53LX_PLATFORM_USER_DATA_H_

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#include "vl53lx_def.h"
#include "cyhal.h"
#ifdef __cplusplus
extern "C"
{
#endif





typedef struct {
	cyhal_i2c_t *i2c;
// PSoC6 I2C handle
cyhal_gpio_t pin_sda; // PSoC6 I2C SDA pin
cyhal_gpio_t pin_scl; // PSoC6 I2C S
cyhal_gpio_t pin_xshut; // PSoC6 GPIO pin for VL53L4CX XSHUT

	VL53LX_DevData_t   Data;


	uint8_t   i2c_slave_address;

	uint8_t   comms_type;

	uint16_t  comms_speed_khz;


	uint32_t  new_data_ready_poll_duration_ms;

} VL53LX_Dev_t;



typedef VL53LX_Dev_t *VL53LX_DEV;



#define VL53LXDevDataGet(Dev, field) (Dev->Data.field)



#define VL53LXDevDataSet(Dev, field, VL53LX_p_003) ((Dev->Data.field) = (VL53LX_p_003))



#define VL53LXDevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)



#define VL53LXDevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)


#ifdef __cplusplus
}
#endif

#endif


