// SPDX-License-Identifier: GPL-2.0-only
/*
 * @file sgpu_pm_monitor.c
 * Copyright (C) 2024 Samsung Electronics
 */

#include "amdgpu.h"
#include "sgpu_pm_monitor.h"
#include "amdgpu_trace.h"
#include "sgpu_sysfs.h"

void sgpu_pm_monitor_init(struct amdgpu_device *adev)
{
	struct sgpu_pm_monitor *monitor = &adev->pm_monitor;

	spin_lock_init(&monitor->lock);

	monitor->pm_stats.power_status = SGPU_POWER_STATE_ON;
	monitor->pm_stats.last_update =  ktime_get_ns();

	memset(&monitor->base_stats, 0, sizeof(struct sgpu_pm_stats));
	monitor->monitor_enable = false;
}

static void sgpu_pm_monitor_calculate(struct amdgpu_device *adev)
{
	struct sgpu_pm_stats *stats = &adev->pm_monitor.pm_stats;
	uint64_t cur_time, diff_time;

	cur_time = ktime_get_ns();
	diff_time = cur_time - stats->last_update;
	stats->state_time[stats->power_status] += diff_time;
	stats->last_update = cur_time;
}

void sgpu_pm_monitor_update(struct amdgpu_device *adev, u32 new_status)
{
	struct sgpu_pm_monitor *monitor = &adev->pm_monitor;
	unsigned long flags;

	spin_lock_irqsave(&monitor->lock, flags);

	sgpu_pm_monitor_calculate(adev);
	monitor->pm_stats.power_status = new_status;
	trace_sgpu_pm_monitor_update(new_status);

	spin_unlock_irqrestore(&monitor->lock, flags);
}

void sgpu_pm_monitor_get_stats(struct amdgpu_device *adev,
				struct sgpu_pm_stats *pm_stats)
{
	struct sgpu_pm_monitor *monitor = &adev->pm_monitor;
	unsigned long flags;

	spin_lock_irqsave(&monitor->lock, flags);

	sgpu_pm_monitor_calculate(adev);
	memcpy(pm_stats, &monitor->pm_stats, sizeof(struct sgpu_pm_stats));

	spin_unlock_irqrestore(&monitor->lock, flags);
}

uint64_t sgpu_pm_monitor_get_idle_time(struct amdgpu_device *adev)
{
	struct sgpu_pm_stats pm_stats;

	sgpu_pm_monitor_get_stats(adev, &pm_stats);

	return pm_stats.state_time[SGPU_POWER_STATE_IFPO]
		+ pm_stats.state_time[SGPU_POWER_STATE_SUSPEND];
}

static int sgpu_pm_monitor_start(struct amdgpu_device *adev)
{
	struct sgpu_pm_monitor *monitor = &adev->pm_monitor;

	if (monitor->monitor_enable)
		return -EINVAL;

	sgpu_pm_monitor_get_stats(adev, &monitor->base_stats);

	monitor->monitor_enable = true;

	return 0;
}

static int sgpu_pm_monitor_end(struct amdgpu_device *adev)
{
	struct sgpu_pm_monitor *monitor = &adev->pm_monitor;
	struct sgpu_pm_stats *base_stat = &monitor->base_stats;
	struct sgpu_pm_stats stat;
	int i;

	if (!monitor->monitor_enable)
		return -EINVAL;

	sgpu_pm_monitor_get_stats(adev, &stat);

	for (i = 0; i < SGPU_POWER_STATE_NR; ++i)
		monitor->state_duration[i] = stat.state_time[i]
					     - base_stat->state_time[i];
	monitor->monitor_duration = stat.last_update - base_stat->last_update;

	monitor->monitor_enable = false;

	return 0;
}

static ssize_t sgpu_pm_monitor_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	struct amdgpu_device *adev = kobj_to_adev(kobj);
	u32 enable;
	int ret;

	ret = kstrtou32(buf, 0, &enable);
	if (ret)
		return ret;

	if (!!enable)
		ret = sgpu_pm_monitor_start(adev);
	else
		ret = sgpu_pm_monitor_end(adev);

	if (ret)
		return ret;

	return count;
}

static ssize_t sgpu_pm_monitor_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct amdgpu_device *adev = kobj_to_adev(kobj);
	struct sgpu_pm_monitor *monitor = &adev->pm_monitor;
	uint64_t ratio[SGPU_POWER_STATE_NR] = {0, };
	char *power_states[SGPU_POWER_STATE_NR] = {"Active",
							"IFPO power off",
							"Device suspend"};
	ssize_t count = 0;
	int i;

	count += scnprintf(&buf[count], PAGE_SIZE - count,
				"Last measured PM monitoring data\n");

	if (monitor->monitor_duration) {
		for (i = 0; i < SGPU_POWER_STATE_NR; ++i)
			ratio[i] = (monitor->state_duration[i] * 100)
					/ monitor->monitor_duration;
	}

	for (i = 0; i < SGPU_POWER_STATE_NR; ++i)
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"%16s : %llu%% (%lluus)\n", power_states[i],
				ratio[i], monitor->state_duration[i] / 1000);

	count += scnprintf(&buf[count], PAGE_SIZE - count,
				"Monitor duration : %lluus\n",
				monitor->monitor_duration / 1000);

	return count;
}
struct kobj_attribute attr_gpu_pm_monitor = __ATTR_RW(sgpu_pm_monitor);
