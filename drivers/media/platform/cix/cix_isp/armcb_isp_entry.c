/*
 * Copyright (c) 2021-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "armcb_isp.h"
#include "armcb_isp_driver.h"
#include "armcb_v4l2_config.h"
#include "armcb_sensor.h"
#include "actuator/armcb_actuator.h"
#include "bus/i2c/system_i2c.h"
#include "linux/kern_levels.h"
#include "linux/kernel.h"
#include "bus/spi/system_spi.h"
#include "armcb_camera_io_drv.h"
#include "cix_vi_hw.h"

typedef void* (*init_func)(void);
typedef void (*exit_func)(void);

struct armcb_ko_entry {
	init_func init;
	exit_func exit;
	void *priv;
	bool is_init;
};

struct armcb_ko_entry g_ko_entries[] = {
	// 1.init cfg driver(video0) firstly
	{ armcb_get_v4l2_cfg_driver_instance,    armcb_v4l2_cfg_driver_destroy,    NULL, false },
	// 2.init other v4l2 sub devices
	{ armcb_get_sensor_driver_instance,      armcb_sensor_driver_destroy,      NULL, false },
	{ armcb_get_isp_driver_instance,         armcb_isp_driver_destroy,         NULL, false },
	{ armcb_get_motor_driver_instance,       armcb_motor_driver_destroy,       NULL, false },
	{ armcb_get_cam_io_drv_instance,         armcb_cam_io_drv_destroy,         NULL, false },
	{ cix_vi_hw_instance,                    cix_vi_hw_destroy,                NULL, false },
};

static int __init armcb_isp_submodules_init(void)
{
	int i = 0;
	init_func fn;
	for (; i < ARRAY_SIZE(g_ko_entries); i++) {
		fn = g_ko_entries[i].init;
		if (fn && fn() != NULL) {
			g_ko_entries[i].is_init = true;
		}
	}
	return 0;
}

static void __exit armcb_isp_submodule_exit(void)
{
	int       i  = 0;
	exit_func fn = NULL;
#if ( LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 17, 0 ) )
	armcb_cam_instance_destroy();
#endif
	i = ARRAY_SIZE(g_ko_entries) - 1;

	for (; i >= 0; i--) {
		fn = g_ko_entries[i].exit;
		if (fn) {
			printk(KERN_ERR "destroy enter %d.\n", i);
			fn();
		}
	}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION( 4, 17, 0 ) )
	armcb_cam_instance_destroy();
#endif
	return;
}

module_init(armcb_isp_submodules_init);
module_exit(armcb_isp_submodule_exit);

MODULE_AUTHOR("Armchina Inc.");
MODULE_DESCRIPTION("Armchina isp ko entry driver");
MODULE_LICENSE("GPL v2");
