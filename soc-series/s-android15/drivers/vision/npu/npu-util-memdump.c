/*
 * Samsung Exynos SoC series NPU driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <soc/samsung/exynos/debug-snapshot.h>
#include "include/npu-common.h"
#include "npu-debug.h"
#include "npu-util-memdump.h"
#include "npu-util-regs.h"
#include "include/npu-memory.h"
#include "npu-scheduler.h"
#include "npu-ver.h"
#include "dsp-dhcp.h"


void npu_util_dump_handle_nrespone(struct npu_system *system)
{
	int i = 0;
	struct npu_device *device = NULL;

	device = container_of(system, struct npu_device, system);

	for (i = 0; i < 5; i++)
		npu_soc_status_report(&device->system);

#if IS_ENABLED(CONFIG_SOC_S5E9955)
	{
		union dsp_dhcp_pwr_ctl pwr_ctl;

		pwr_ctl.value = device->system.dhcp_table->DNC_PWR_CTRL;

		if (pwr_ctl.npu_pm)
			npu_cmd_map_dump(&device->system, "gnpucmdqpc");
		if (pwr_ctl.dsp_pm)
			npu_cmd_map_dump(&device->system, "dspcmdqpc");

		npu_read_hw_reg(npu_get_io_area(system, "sfrmbox1"), 0x2000, 0xFFFFFFFF, 0);
	}
#endif

	npu_ver_dump(device);
	fw_will_note(FW_LOGSIZE);
	npu_memory_dump(&device->system.memory);
#if IS_ENABLED(CONFIG_SOC_S5E9955)
	npu_log_fw_governor(device);
#endif
	session_fault_listener();

	/* trigger a wdreset to analyse s2d dumps in this case */
	dbg_snapshot_expire_watchdog();
}

int npu_util_dump_handle_error_k(struct npu_device *device)
{
	int ret = 0;
	int i = 0;

	for (i = 0; i < 5; i++)
		npu_soc_status_report(&device->system);

#if IS_ENABLED(CONFIG_SOC_S5E9955)
	union dsp_dhcp_pwr_ctl pwr_ctl;

	pwr_ctl.value = device->system.dhcp_table->DNC_PWR_CTRL;

	if (pwr_ctl.npu_pm) {
		npu_dump("GNPU 0/1 PC Value\n");
		npu_cmd_map_dump(&device->system, "gnpucmdqpc");
	}
	if (pwr_ctl.dsp_pm) {
		npu_dump("DSP PC Value\n");
		npu_cmd_map_dump(&device->system, "dspcmdqpc");
	}

	npu_read_hw_reg(npu_get_io_area(&device->system, "sfrmbox1"), 0x2000, 0xFFFFFFFF, 0);
#endif

	npu_ver_dump(device);
	fw_will_note(FW_LOGSIZE);
	npu_memory_dump(&device->system.memory);
#if IS_ENABLED(CONFIG_SOC_S5E9955)
	npu_log_fw_governor(device);
#endif
	session_fault_listener();

	/* trigger a wdreset to analyse s2d dumps in this case */
	dbg_snapshot_expire_watchdog();

	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////
// Lifecycle functions

int npu_util_memdump_probe(struct npu_system *system)
{
	return 0;
}

int npu_util_memdump_open(struct npu_system *system)
{
	/* NOP */
	return 0;
}

int npu_util_memdump_close(struct npu_system *system)
{
	/* NOP */
	return 0;
}
