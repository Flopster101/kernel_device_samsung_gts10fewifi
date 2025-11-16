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
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <soc/samsung/exynos/exynos-soc.h>

#include "npu-log.h"
#include "npu-profile-v2.h"
#include "npu-device.h"
#include "include/npu-common.h"
#include "npu-session.h"
#include "npu-system.h"
#include "npu-scheduler.h"
#include "npu-util-common.h"
#include "npu-hw-device.h"
#include "npu-dvfs.h"
#include "dsp-dhcp.h"
#include "npu-protodrv.h"
#if IS_ENABLED(CONFIG_NPU_CHECK_PRECISION)
#include "npu-precision.h"
#endif
#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
#include "dsp-kernel.h"
#include "dl/dsp-dl-engine.h"
#include "dl/dsp-common.h"
#endif
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
#include "npu-governor.h"
#endif

#ifdef NPU_LOG_TAG
#undef NPU_LOG_TAG
#endif
#define NPU_LOG_TAG		"npu-sess"

//#define SYSMMU_FAULT_TEST
#define KPI_FRAME_MAX_BUFFER	(16)
#define KPI_FRAME_PRIORITY	(200)
static struct npu_frame kpi_frame[KPI_FRAME_MAX_BUFFER];
const struct npu_session_ops npu_session_ops;
#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
static u32 dl_unique_id;
#endif

static void __npu_session_ion_sync_for_device(struct npu_memory_buffer *pbuf,
						enum dma_data_direction dir)
{
	if (dir == DMA_TO_DEVICE)
		dma_sync_sg_for_device(pbuf->attachment->dev,
			pbuf->sgt->sgl, pbuf->sgt->orig_nents, DMA_TO_DEVICE);
	else // dir == DMA_FROM_DEVICE
		dma_sync_sg_for_cpu(pbuf->attachment->dev,
			pbuf->sgt->sgl, pbuf->sgt->orig_nents, DMA_FROM_DEVICE);
}

void npu_session_ion_sync_for_device(struct npu_memory_buffer *pbuf, enum dma_data_direction dir)
{
	if (likely(pbuf->vaddr))
		__npu_session_ion_sync_for_device(pbuf, dir);
}

npu_errno_t chk_nw_result_no_error(struct npu_session *session)
{
	return session->nw_result.result_code;
}

int npu_session_save_result(struct npu_session *session, struct nw_result nw_result)
{
	session->nw_result = nw_result;
	wake_up(&session->wq);
	return 0;
}

void npu_session_queue_done(struct npu_queue *queue, struct npu_queue_list *inclist, struct npu_queue_list *otclist, unsigned long flag)
{
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	u32 buff_cnt, freq_index;

	buff_cnt = otclist->containers[0].count;
	freq_index = npu_update_cmdq_progress_from_done(queue);
	npu_queue_done(queue, inclist, otclist, flag);
	npu_update_frequency_from_done(queue, freq_index);
#else
	u32 buff_cnt;

	buff_cnt = otclist->containers[0].count;
	npu_queue_done(queue, inclist, otclist, flag);
#endif
}

static save_result_func get_notify_func(const nw_cmd_e nw_cmd)
{
	if (unlikely(nw_cmd == NPU_NW_CMD_CLEAR_CB))
		return NULL;
	else
		return npu_session_save_result;
}

int npu_session_put_nw_req(struct npu_session *session, nw_cmd_e nw_cmd)
{
	int ret = 0;

	struct npu_nw req = {
		.uid = session->uid,
		.bound_id = session->sched_param.bound_id,
		.priority = session->sched_param.priority,
		.deadline = session->sched_param.deadline,
		.preference = session->preference,
		.session = session,
		.cmd = nw_cmd,
		.ncp_addr = session->ncp_info.ncp_addr,
		.magic_tail = NPU_NW_MAGIC_TAIL,
	};

	req.notify_func = get_notify_func(nw_cmd);

	ret = npu_ncp_mgmt_put(&req);
	if (unlikely(!ret)) {
		npu_uerr("npu_ncp_mgmt_put failed(%d)", session, session->hids, ret);
		return ret;
	}
	return ret;
}

#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
static inline struct npu_system *get_session_system(struct npu_session *session)
{
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	struct npu_system *system;

	WARN_ON(!session);
	vctx = &session->vctx;
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);
	system = &device->system;
	return system;
}
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
static struct imb_size_control *imb_size_ctl_ref;

static inline u32 __calc_imb_instance_n(struct npu_session *session, bool free)
{
	u32 idx, largest = 0, second_largest = 0;
	struct npu_session *session_ref;
	struct npu_sessionmgr *sessionmgr = session->cookie;

	WARN_ON(!sessionmgr);
	mutex_lock(session->global_lock);
	for (idx = 0; idx < NPU_MAX_SESSION; idx++) {
		if (sessionmgr->session[idx]) {
			session_ref = sessionmgr->session[idx];

			if (!session_ref->IMB_mem_buf)
				continue;

			if (free && (session_ref == session))
				continue;

			if (session_ref->is_instance_1)
				continue;

			npu_uinfo("[INSTANCE N] idx : %d - imb_size : %ld\n",
					session_ref, session_ref->hids, idx, session_ref->IMB_mem_buf->ncp_max_size);
			if (session_ref->IMB_mem_buf->ncp_max_size > largest) {
				second_largest = largest;
				largest = session_ref->IMB_mem_buf->ncp_max_size;
			} else if (session_ref->IMB_mem_buf->ncp_max_size > second_largest) {
				second_largest = session_ref->IMB_mem_buf->ncp_max_size;
			}
		}
	}
	mutex_unlock(session->global_lock);

	npu_uinfo("[INSTANCE_N] largest : %u, second_largest : %u\n", session, session->hids, largest, second_largest);
	return largest + second_largest;
}

static inline u32 __calc_imb_instance_1(struct npu_session *session, bool free)
{
	u32 idx, largest = 0;
	struct npu_session *session_ref;
	struct npu_sessionmgr *sessionmgr = session->cookie;

	WARN_ON(!sessionmgr);
	mutex_lock(session->global_lock);
	for (idx = 0; idx < NPU_MAX_SESSION; idx++) {
		if (sessionmgr->session[idx]) {
			session_ref = sessionmgr->session[idx];

			if (!session_ref->IMB_mem_buf)
				continue;

			if (free && (session_ref == session))
				continue;

			if (!session_ref->is_instance_1)
				continue;

			npu_uinfo("[INSTANCE 1] idx : %d - imb_size : %ld\n",
					session_ref, session_ref->hids, idx, session_ref->IMB_mem_buf->ncp_max_size);
			if (session_ref->IMB_mem_buf->ncp_max_size > largest)
				largest = session_ref->IMB_mem_buf->ncp_max_size;
		}
	}
	mutex_unlock(session->global_lock);

	npu_uinfo("[INSTANCE_1] largest : %u\n", session, session->hids, largest);
	return largest;
}

#define MAX(a, b) ((a) > (b) ? (a) : (b))
static inline u32 __calc_imb_req_chunk_v2(struct npu_session *session, bool free)
{
	u32 req_chunk_cnt, req_size;

	req_size = MAX(__calc_imb_instance_1(session, free), __calc_imb_instance_n(session, free));
	req_size = ALIGN(req_size, NPU_IMB_GRANULARITY);
	req_chunk_cnt = req_size / NPU_IMB_CHUNK_SIZE;
	if (unlikely(req_chunk_cnt > NPU_IMB_CHUNK_MAX_NUM))
		req_chunk_cnt = NPU_IMB_CHUNK_MAX_NUM;

	return req_chunk_cnt;
}

static inline u32 __calc_imb_req_chunk(struct npu_session *session,
					struct npu_memory_buffer *IMB_mem_buf, bool free)
{
	return __calc_imb_req_chunk_v2(session, free);
}

static int __npu_update_imb_size_save_result(struct npu_session *dummy_session, struct nw_result nw_result)
{
	int ret = 0;

	WARN_ON(!imb_size_ctl_ref);

	imb_size_ctl_ref->result_code = nw_result.result_code;
	imb_size_ctl_ref->fw_size = nw_result.nw.result_value;
	wake_up(&imb_size_ctl_ref->waitq);

	return ret;
}

static int __npu_update_imb_size(struct npu_session *session, u32 new_chunk_cnt)
{
	int ret_req;
	struct npu_system *system = get_session_system(session);
	struct npu_nw req = {
		.uid = session->uid,
		.cmd = NPU_NW_CMD_IMB_SIZE,
		.notify_func = __npu_update_imb_size_save_result,
		.param0 = new_chunk_cnt * NPU_IMB_CHUNK_SIZE,
		.session = session,
	};

	WARN_ON(!system);
	imb_size_ctl_ref = &system->imb_size_ctl;
	imb_size_ctl_ref->result_code = NPU_SYSTEM_JUST_STARTED;

	/* queue the message for sending */
	ret_req = npu_ncp_mgmt_put(&req);
	if (unlikely(!ret_req)) {
		npu_uerr("IMB: npu_ncp_mgmt_put failed(%d)\n", session, session->hids, ret_req);
		return -EWOULDBLOCK;
	}

	/* wait for response from FW */
	wait_event(imb_size_ctl_ref->waitq, imb_size_ctl_ref->result_code != NPU_SYSTEM_JUST_STARTED);

	if (unlikely(imb_size_ctl_ref->result_code != NPU_ERR_NO_ERROR)) {
		/* firmware failed or did not accept the change in IMB size */
		npu_uerr("IMB: failure response for IMB_SIZE message, result code = %d\n",
			 session, session->hids, imb_size_ctl_ref->result_code);
	}

	return imb_size_ctl_ref->result_code;
}

static int __fw_sync_CMD_IMB_SIZE(struct npu_session *session, u32 new_chunk_cnt)
{
	int ret;
	u32 fw_chunk_cnt;
	struct npu_system *system = get_session_system(session);

	WARN_ON(!system);
	ret = __npu_update_imb_size(session, new_chunk_cnt);
	if (likely(!ret)) {
		fw_chunk_cnt = system->imb_size_ctl.fw_size / NPU_IMB_CHUNK_SIZE;
		if (fw_chunk_cnt == new_chunk_cnt) {
			npu_udbg("IMB: new size 0x%X was accepted by FW\n", session, session->hids, fw_chunk_cnt * NPU_IMB_CHUNK_SIZE);
		} else {
			npu_udbg("IMB: new size 0x%X will be updated. currently, FW uses IMB size 0x%X\n",
				 session, session->hids, new_chunk_cnt * NPU_IMB_CHUNK_SIZE, fw_chunk_cnt * NPU_IMB_CHUNK_SIZE);
		}
	} else {
		npu_uerr("IMB: sending IMB_SIZE command to FW failed\n", session, session->hids);
	}

	return ret;
}

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
#define	NPU_IMB_ASYNC_THRESHOLD		(0x1000000)

static void npu_imb_async_work(struct work_struct *work)
{
	int ret = 0;
	u32 i = 0;
	u32 address_vector_offset;
	struct address_vector *av;
	char *ncp_vaddr;
	u32 IMB_cnt;
	u32 addr_offset = 0;
	dma_addr_t dAddr;
	u32 prev_chunk_cnt;
	struct npu_session *session;
	struct npu_system *system;
	struct npu_memory_buffer *IMB_mem_buf;
	struct addr_info **imb_av;
	u32 req_chunk_cnt;

	session = container_of(work, struct npu_session, imb_async_work.work);
	IMB_cnt = session->IMB_cnt;
	IMB_mem_buf = session->IMB_mem_buf;
	imb_av = session->imb_av;
	system = get_session_system(session);
	session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_START;
	ncp_vaddr = (char *)session->ncp_hdr_buf->vaddr;
	address_vector_offset = session->address_vector_offset;

	av = (struct address_vector *)(ncp_vaddr + address_vector_offset);

	npu_uinfo("Start async alloc for IMB\n", session, session->hids);

	WARN_ON(!IMB_mem_buf);

	if (session->hids == NPU_HWDEV_ID_NPU) {
		mutex_lock(&system->imb_alloc_data.imb_alloc_lock);
		req_chunk_cnt = __calc_imb_req_chunk(session, IMB_mem_buf, false);

		prev_chunk_cnt = system->imb_alloc_data.alloc_chunk_cnt;
		for (i = system->imb_alloc_data.alloc_chunk_cnt; i < req_chunk_cnt; i++) {
			dAddr = system->imb_alloc_data.chunk_imb->paddr + (i * NPU_IMB_CHUNK_SIZE);
			system->imb_alloc_data.chunk[i].size = NPU_IMB_CHUNK_SIZE;
			ret = npu_memory_alloc_from_heap(system->pdev, &(system->imb_alloc_data.chunk[i]), dAddr,
						&system->memory, "IMB", 0);
			if (unlikely(ret)) {
				npu_uerr("IMB: npu_memory_alloc_from_heap failed, err: %d\n", session, session->hids, ret);
				mutex_unlock(&system->imb_alloc_data.imb_alloc_lock);
				session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_ERROR;
				ret = -EFAULT;
				goto p_err;
			}
			system->imb_alloc_data.alloc_chunk_cnt++;
			npu_udbg("IMB: successfully allocated chunk = %u with size = 0x%X\n", session, session->hids, i, NPU_IMB_CHUNK_SIZE);
		}

		/* in allocation case we're not interested in FW size acceptance */
		__fw_sync_CMD_IMB_SIZE(session, system->imb_alloc_data.alloc_chunk_cnt);

		if (prev_chunk_cnt != system->imb_alloc_data.alloc_chunk_cnt) {
			npu_udbg("IMB: system total size 0x%X -> 0x%X\n", session, session->hids,
				 prev_chunk_cnt * NPU_IMB_CHUNK_SIZE,
				 system->imb_alloc_data.alloc_chunk_cnt * NPU_IMB_CHUNK_SIZE);
		}

		IMB_mem_buf->daddr = system->imb_alloc_data.chunk_imb->paddr;
		mutex_unlock(&system->imb_alloc_data.imb_alloc_lock);
	} else {
		ret = npu_memory_alloc(session->memory, IMB_mem_buf, npu_get_configs(NPU_PBHA_HINT_00));
		if (unlikely(ret)) {
			npu_uerr("IMB: npu_memory_alloc is fail(%d).\n", session, session->hids, ret);
			goto p_err;
		}
	}

	for (i = 0; i < IMB_cnt; i++) {
		(av + (*imb_av + i)->av_index)->m_addr = IMB_mem_buf->daddr + addr_offset;
		(session->IMB_info + i)->daddr = IMB_mem_buf->daddr + addr_offset;
		(session->IMB_info + i)->vaddr = ((void *)((char *)(IMB_mem_buf->vaddr)) + addr_offset);
		(session->IMB_info + i)->size = (*imb_av + i)->size;
		addr_offset += ALIGN((*imb_av + i)->size, (u32)NPU_IMB_ALIGN);
		npu_udbg("IMB: (IMB_mem_buf + %d)->vaddr(%pad), daddr(%pad), size(%zu)\n",
			 session, session->hids, i, (session->IMB_info + i)->vaddr, &(session->IMB_info + i)->daddr,
			 (session->IMB_info + i)->size);
	}

	npu_session_ion_sync_for_device(IMB_mem_buf, DMA_TO_DEVICE);
	session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_DONE;
	npu_uinfo("End async alloc for IMB\n", session, session->hids);
p_err:
	wake_up(&session->imb_wq);
	if (likely(imb_av)) {
		kfree(imb_av);
		session->imb_av = NULL;
	}
	return;
}

static int npu_imb_alloc_async(struct npu_session *session,
		struct addr_info **IMB_av, struct npu_memory_buffer *IMB_mem_buf)
{
	int ret = 0;

	session->imb_av = (struct addr_info **)kzalloc(sizeof(struct addr_info *) * session->IMB_cnt, GFP_KERNEL);
	if (unlikely(!session->imb_av)) {
		npu_uerr("failed in a_IMB_av(ENOMEM)\n", session, session->hids);
		return -ENOMEM;
	}

	memcpy(session->imb_av, IMB_av, sizeof(struct addr_info *) * session->IMB_cnt);
	session->IMB_mem_buf = IMB_mem_buf;
	queue_delayed_work(session->imb_async_wq, &session->imb_async_work, msecs_to_jiffies(0));

	return ret;
}

int npu_session_imb_async_init(struct npu_session *session)
{
	int ret = 0;
	struct npu_device *device;
	struct npu_sessionmgr *sess_mgr;

	sess_mgr = (struct npu_sessionmgr *)(session->cookie);
	device = container_of(sess_mgr, struct npu_device, sessionmgr);
	INIT_DELAYED_WORK(&session->imb_async_work, npu_imb_async_work);

	session->imb_async_wq = create_singlethread_workqueue(dev_name(device->dev));
	if (!session->imb_async_wq) {
		npu_uerr("fail to create workqueue -> system->imb_async_wq\n", session, session->hids);
		ret = -EFAULT;
		goto err_probe;
	}

	init_waitqueue_head(&(session->imb_wq));
err_probe:
	return ret;
}
#endif
/* full - is used while release graph only, otherwise it's dequeue with guaranteed fw syncing */
static void __free_imb_mem_buf(struct npu_session *session, bool full)
{
	int ret;
	u32 fw_chunk, alloc_chunk, max_next_req_chunk = 0;
	struct npu_system *system = get_session_system(session);
	struct npu_sessionmgr *sessionmgr = session->cookie;

	WARN_ON(!sessionmgr);
	if (likely(session->IMB_mem_buf)) {
		mutex_lock(&system->imb_alloc_data.imb_alloc_lock);

		max_next_req_chunk = __calc_imb_req_chunk_v2(session, true);

		if (max_next_req_chunk < system->imb_alloc_data.alloc_chunk_cnt) {
			if (!full || max_next_req_chunk) {
				/* if deque or there are other non-closed sessions with buffers,
				 * we should sync it with FW
				 */
				ret = __fw_sync_CMD_IMB_SIZE(session, max_next_req_chunk);
				if (likely(!ret)) {
					/* deallocate depending on the memory that fw is using */
					fw_chunk = system->imb_size_ctl.fw_size / NPU_IMB_CHUNK_SIZE;
					alloc_chunk = (fw_chunk >= max_next_req_chunk) ? fw_chunk : max_next_req_chunk;
					__free_imb_chunk(alloc_chunk, system, &system->imb_alloc_data);
				}
			} else {
				/* if we release graph and there no any sessions with buffers,
				 * deallocate it anyway without FW syncing
				 */
				npu_udbg("IMB: buffer full free w/o FW syncing\n", session, session->hids);
				__free_imb_chunk(0, system, &system->imb_alloc_data);
			}
		}
		mutex_unlock(&system->imb_alloc_data.imb_alloc_lock);
	}
}
#endif
#endif

int npu_session_put_frame_req(
	struct npu_session *session, struct npu_queue *queue,	struct npu_queue_list *incl, struct npu_queue_list *otcl,
	frame_cmd_e frame_cmd, struct av_info *IFM_info, struct av_info *OFM_info)
{
	int ret = 0;

	struct npu_frame frame = {
		.uid = session->uid,
		.frame_id = incl->id,
		.npu_req_id = 0,	/* Determined in manager */
		.result_code = 0,
		.session = session,
		.cmd = frame_cmd,
		.priority = session->sched_param.priority,   /* Read priority passed by app */
		.src_queue = queue,
		.input = incl,
		.output = otcl,
		.magic_tail = NPU_FRAME_MAGIC_TAIL,
		.mbox_process_dat = session->mbox_process_dat,
		.IFM_info = IFM_info,
		.OFM_info = OFM_info,
	};
	struct npu_frame *f;

	if (is_kpi_mode_enabled(false)) {
		f = &(kpi_frame[incl->index % KPI_FRAME_MAX_BUFFER]);
		f->uid = session->uid;
		f->frame_id = incl->id;
		f->npu_req_id = 0;       /* Determined in manager */
		f->result_code = 0;
		f->session = session;
		f->cmd = frame_cmd;
		/* setting high priority for KPI frames even though app may not have set */
		f->priority = KPI_FRAME_PRIORITY;
		f->src_queue = queue;
		f->input = incl;
		f->output = otcl;
		f->magic_tail = NPU_FRAME_MAGIC_TAIL;
		f->mbox_process_dat = session->mbox_process_dat;
		f->IFM_info = IFM_info;
		f->OFM_info = OFM_info;

		npu_dbg("Sending kpi frames to mailbox : uid %d, index %d/%d, frame id %d, frame %p\n",
				session->uid, incl->index, otcl->index, incl->id, f);
		session->current_frame = f;
		ret = kpi_frame_mbox_put(f);
		if (unlikely(!ret)) {
			npu_uerr("fail(%d) in kpi_frame_mobx_put\n",
					session, session->hids, ret);
			ret = -EFAULT;
			goto p_err;
		}
	} else {
		npu_dbg("Sending frames to mailbox : uid %d, index %d/%d, frame id %d\n",
				session->uid, incl->index, otcl->index, incl->id);
		session->current_frame = &frame;

		mutex_lock(session->global_lock);
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
		npu_update_frequency_from_queue(session, 1);
		ret = proto_drv_handler_frame_requested(&frame);

		if (unlikely(ret)) {
			npu_uerr("fail(%d) in npu_buffer_q_put\n",
					session, session->hids, ret);
			mutex_unlock(session->global_lock);
			npu_update_frequency_from_queue(session, 1);
			npu_revert_cmdq_list(session);
			ret = -EFAULT;
			goto p_err;
		}
#else
		ret = proto_drv_handler_frame_requested(&frame);

		if (unlikely(ret)) {
			npu_uerr("fail(%d) in npu_buffer_q_put\n", session, session->hids, ret);
			mutex_unlock(session->global_lock);
			ret = -EFAULT;
			goto p_err;
		}
#endif
		mutex_unlock(session->global_lock);
	}

	npu_udbg("success\n", session, session->hids);

	return 0;
p_err:
	return ret;
}

int add_ion_mem(struct npu_session *session, struct npu_memory_buffer *mem_buf, u32 memory_type)
{
	switch (memory_type) {
	case MEMORY_TYPE_NCP:
		session->ncp_mem_buf = mem_buf;
		break;
	case MEMORY_TYPE_NCP_HDR:
		session->ncp_hdr_buf = mem_buf;
		break;
	case MEMORY_TYPE_IN_FMAP:
		session->IOFM_mem_buf = mem_buf;
		break;
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	case MEMORY_TYPE_IM_FMAP:
		session->IMB_mem_buf = mem_buf;
		break;
#endif
	case MEMORY_TYPE_WEIGHT:
		session->weight_mem_buf = mem_buf;
		break;
	default:
		break;
	}
	return 0;
}

#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
static inline void __release_imb_mem_buf(struct npu_session *session)
{
	struct npu_memory_buffer *ion_mem_buf = session->IMB_mem_buf;

	if (likely(ion_mem_buf)) {
#ifdef CONFIG_NPU_IMB_ASYNC_ALLOC
		session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_SKIP;
#endif
	if (session->hids == NPU_HWDEV_ID_NPU) {
		/* deallocate all memory chunks */
		__free_imb_mem_buf(session, true);
	}	else {
		if (ion_mem_buf->daddr)
			npu_memory_free(session->memory, ion_mem_buf);
	}

		mutex_lock(session->global_lock);
		session->IMB_mem_buf = NULL;
		mutex_unlock(session->global_lock);

		kfree(ion_mem_buf);
	}
}
#endif

void __release_graph_ion(struct npu_session *session)
{
	struct npu_memory *memory;
	struct npu_memory_buffer *ion_mem_buf;

	memory = session->memory;
	ion_mem_buf = session->ncp_mem_buf;
	session->ncp_mem_buf = NULL;
	session->ncp_mem_buf_offset = 0;

	if (unlikely(ion_mem_buf)) {
		npu_memory_unmap(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

	ion_mem_buf = session->ncp_hdr_buf;
	session->ncp_hdr_buf = NULL;
	if (unlikely(ion_mem_buf)) {
		npu_memory_free(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

	ion_mem_buf = session->IOFM_mem_buf;
	session->IOFM_mem_buf = NULL;
	if (unlikely(ion_mem_buf)) {
		npu_memory_free(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	__release_imb_mem_buf(session);
#endif
}

int npu_session_NW_CMD_UNLOAD(struct npu_session *session)
{
	/* check npu_device emergency error */
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	nw_cmd_e nw_cmd = NPU_NW_CMD_UNLOAD;

	vctx = &(session->vctx);
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

	if (unlikely(npu_device_is_emergency_err(device)))
		npu_udbg("NPU DEVICE IS EMERGENCY ERROR\n", session, session->hids);

	/* Post unload command */
	npu_udbg("sending UNLOAD command.\n", session, session->hids);
	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

	if (session->ss_state & BIT(NPU_SESSION_STATE_PROFILE))
		npu_profile_iter_reupdate(0, 0);

	if (session->ncp_payload) {
		npu_memory_free(session->memory, session->ncp_payload);
		kfree(session->ncp_payload);
		session->ncp_payload = NULL;
	}

	return 0;
}

int npu_session_close(struct npu_session *session)
{
	int ret = 0;
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	struct npu_sessionmgr *sessionmgr;
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	struct vs4l_param param;
#endif

	proto_req_frames_free(session->uid);

#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	param.target = NPU_S_PARAM_QOS_MIF;
	param.offset = NPU_QOS_DEFAULT_VALUE;
	npu_qos_param_handler(session, &param);
#endif
	npu_scheduler_unregister_session(session);
	session->ss_state |= BIT(NPU_SESSION_STATE_CLOSE);
	vctx = &(session->vctx);
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);
	sessionmgr = session->cookie;
	if (unlikely(!sessionmgr)) {
		npu_uerr("Failed to get sessionmgr\n", session, session->hids);
		ret = -EINVAL;
		goto p_err;
	}

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	if (session->imb_async_wq) {
		destroy_workqueue(session->imb_async_wq);
		session->imb_async_wq = NULL;
	}
	session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_SKIP;
#endif

#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	if (session->check_core) {
		if (session->is_control_dsp)
			sessionmgr->dsp_flag--;

		if (session->is_control_npu)
			sessionmgr->npu_flag--;

		session->check_core = false;
	}
#endif

#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
	mutex_lock(session->global_lock);
	if (session->hids & NPU_HWDEV_ID_DSP)
		dsp_graph_remove_kernel(device, session);

	mutex_unlock(session->global_lock);
#endif

	ret = npu_session_undo_close(session);
	if (ret)
		goto p_err;

	npu_sessionmgr_unset_unifiedID(session);

	ret = npu_session_undo_open(session);
	if (ret)
		goto p_err;
	return ret;
p_err:
	npu_err("fail(%d)\n", ret);
	return ret;
}

static void npu_session_process_register(struct npu_session *session)
{
	struct task_struct *parent;

	session->comm[0] = '\0';
	session->p_comm[0] = '\0';

	session->pid = task_pid_nr(current);

	strncpy(session->comm, current->comm, TASK_COMM_LEN);

	if (current->parent != NULL) {
		parent = current->parent;
		session->p_pid = task_pid_nr(parent);
		strncpy(session->p_comm, parent->comm, TASK_COMM_LEN);
	}

	npu_uinfo("pid(%d), current(%s), parent pid(%d), parent(%s)\n",
		session, session->hids, session->pid, session->comm, session->p_pid, session->p_comm);
}

int npu_session_open(struct npu_session **session, void *cookie, void *memory)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	int i = 0;
	struct npu_sessionmgr *sessionmgr;
#endif

	*session = kzalloc(sizeof(struct npu_session), GFP_KERNEL);
	if (unlikely(*session == NULL)) {
		npu_err("fail in kzalloc\n");
		ret = -ENOMEM;
		return ret;
	}

	(*session)->ss_state |= BIT(NPU_SESSION_STATE_OPEN);

	INIT_LIST_HEAD(&(*session)->active_session);
	ret = npu_sessionmgr_regID(cookie, *session);
	if (unlikely(ret)) {
		npu_err("fail(%d) in npu_sessionmgr_regID\n", ret);
		npu_session_register_undo_cb(*session, npu_session_undo_open);
		goto p_err;
	}

	(*session)->ss_state |= BIT(NPU_SESSION_STATE_REGISTER);

	init_waitqueue_head(&((*session)->wq));

	(*session)->cookie = cookie;
	(*session)->memory = memory;

	(*session)->IMB_info = NULL;

	(*session)->IFM_info.addr_info = NULL;
	(*session)->IFM_info.state = SS_BUF_STATE_UNPREPARE;
	(*session)->OFM_info.addr_info = NULL;
	(*session)->OFM_info.state = SS_BUF_STATE_UNPREPARE;
	(*session)->ncp_mem_buf = NULL;
	(*session)->weight_mem_buf = NULL;
	(*session)->shared_mem_buf = NULL;
	(*session)->ncp_mem_buf_offset = 0;
	(*session)->IOFM_mem_buf = NULL;
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	(*session)->IMB_mem_buf = NULL;
#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	(*session)->imb_av = NULL;
#endif
#endif

	(*session)->qbuf_IOFM_idx = -1;
	(*session)->dqbuf_IOFM_idx = -1;
	(*session)->hids = NPU_HWDEV_ID_NONE;
	(*session)->unified_id = 0;
	(*session)->unified_op_id = 0;

	/* Set default scheduling parameters */
	(*session)->sched_param.bound_id = NPU_BOUND_UNBOUND;
	(*session)->sched_param.priority = NPU_PRIORITY_MID_PRIO; /* Mid priority */
	(*session)->sched_param.deadline = NPU_MAX_DEADLINE;
	(*session)->preference = CMD_LOAD_FLAG_IMB_PREFERENCE2;

	npu_session_process_register(*session);

#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
	INIT_LIST_HEAD(&(*session)->kernel_list);
	(*session)->kernel_count = 0;
	(*session)->dl_unique_id = 0;
	(*session)->str_manager.count = 0;
	(*session)->str_manager.strings = NULL;
#endif

#if IS_ENABLED(CONFIG_NPU_USE_IFD)
	(*session)->dvfs_info = NULL;
#endif

	(*session)->last_qbuf_arrival = npu_get_time_us();
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	(*session)->arrival_interval = NPU_SCHEDULER_DEFAULT_REQUESTED_TPF;
	sessionmgr = cookie;
	(*session)->model_hash_node = 0;
	(*session)->tune_npu_level = 0;
	(*session)->earliest_qbuf_time = 0;
	(*session)->prev_npu_level = 0;
	(*session)->next_mif_level = 0;
	(*session)->tpf = NPU_SCHEDULER_DEFAULT_TPF;
	(*session)->tpf_requested = false;
	(*session)->is_control_dsp = false;
	(*session)->is_control_npu = true;

	for (i = 0; i < 5; i++) {
		(*session)->predict_error[i] = 0;
	}
	(*session)->predict_error_idx = 0x0;
#endif
	(*session)->is_instance_1 = false;

	ret = npu_scheduler_register_session(*session);
	if (unlikely(ret)) {
		npu_err("fail(%d) in register_session\n", ret);
		npu_session_register_undo_cb(*session, npu_session_undo_open);
		goto p_err;
	}

	npu_session_imb_async_init(*session);

	ret = proto_req_frames_alloc((*session)->uid);
	if (unlikely(ret)) {
		npu_err("fail(%d) in proto_req_frames_alloc\n", ret);
		goto p_err;
	}

	return ret;
p_err:
	npu_session_execute_undo_cb(*session);
	return ret;
}

int _undo_s_graph_each_state(struct npu_session *session)
{
	struct npu_memory *memory;
	struct npu_memory_buffer *ion_mem_buf;
	struct addr_info *addr_info;

	memory = session->memory;

	if (session->ss_state <= BIT(NPU_SESSION_STATE_GRAPH_ION_MAP))
		goto graph_ion_unmap;
	if (session->ss_state <= BIT(NPU_SESSION_STATE_WGT_KALLOC))
		goto wgt_kfree;
	if (session->ss_state <= BIT(NPU_SESSION_STATE_IOFM_KALLOC))
		goto iofm_kfree;
	if (session->ss_state <= BIT(NPU_SESSION_STATE_IOFM_ION_ALLOC))
		goto iofm_ion_unmap;
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	if (session->ss_state <= BIT(NPU_SESSION_STATE_IMB_ION_ALLOC))
		goto imb_ion_unmap;

imb_ion_unmap:
	addr_info = session->IMB_info;
	session->IMB_info = NULL;
	if (likely(addr_info))
		kfree(addr_info);

	__release_imb_mem_buf(session);
#endif
iofm_ion_unmap:
	ion_mem_buf = session->IOFM_mem_buf;
	session->IOFM_mem_buf = NULL;
	if (likely(ion_mem_buf)) {
		npu_memory_free(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

iofm_kfree:
	addr_info = session->IFM_info.addr_info;
	session->IFM_info.addr_info = NULL;
	if (likely(addr_info))
		kfree(addr_info);

	addr_info = session->OFM_info.addr_info;
	session->OFM_info.addr_info = NULL;
	if (likely(addr_info))
		kfree(addr_info);

wgt_kfree:
	addr_info = session->WGT_info;
	session->WGT_info = NULL;
	if (likely(addr_info))
		kfree(addr_info);

graph_ion_unmap:
	ion_mem_buf = session->ncp_mem_buf;
	session->ncp_mem_buf = NULL;
	if (likely(ion_mem_buf)) {
		npu_memory_unmap(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

	ion_mem_buf = session->weight_mem_buf;
	session->weight_mem_buf = NULL;
	if (likely(ion_mem_buf)) {
		if (ion_mem_buf->reserved != VS4L_MEMORY_DMABUF_DYNAMIC)
			npu_memory_unmap(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

	ion_mem_buf = session->ncp_hdr_buf;
	session->ncp_hdr_buf = NULL;
	if (likely(ion_mem_buf)) {
		npu_memory_free(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

	ion_mem_buf = session->shared_mem_buf;
	session->shared_mem_buf = NULL;
	if (ion_mem_buf) {
		npu_memory_unmap(memory, ion_mem_buf);
		kfree(ion_mem_buf);
	}

	return 0;
}

int _undo_s_format_each_state(struct npu_session *session)
{
	int ret = 0;

	if (unlikely(!session)) {
		npu_err("Failed to get session\n");
		return -EINVAL;
	}

	if (unlikely(session->ss_state < BIT(NPU_SESSION_STATE_IMB_ION_ALLOC))) {
		ret = -EINVAL;
		goto p_err;
	}

	if (session->ss_state <= BIT(NPU_SESSION_STATE_FORMAT_OT))
		goto init_format;

init_format:

p_err:
	return ret;
}

int _undo_streamoff_each_state(struct npu_session *session)
{
	int ret = 0;

	if (unlikely(!session)) {
		npu_err("Failed to get session\n");
		ret = -EINVAL;
		goto p_err;
	}

	if (unlikely(session->ss_state < BIT(NPU_SESSION_STATE_START))) {
		ret = -EINVAL;
		goto p_err;
	}

	if (session->ss_state <= BIT(NPU_SESSION_STATE_STOP))
		goto release_streamoff;

release_streamoff:

p_err:
	return ret;
}

int _undo_close_each_state(struct npu_session *session)
{
	int ret = 0;

	if (unlikely(session->ss_state < BIT(NPU_SESSION_STATE_STOP))) {
		ret = -EINVAL;
		goto p_err;
	}

	if (session->ss_state <= BIT(NPU_SESSION_STATE_CLOSE))
		goto session_close;

session_close:

p_err:
	return ret;
}

bool EVER_FIND_FM(u32 *FM_cnt, struct addr_info *FM_av, u32 address_vector_index)
{
	bool ret = false;
	u32 i;

	for (i = 0; i < (*FM_cnt); i++) {
		if ((FM_av + i)->av_index == address_vector_index) {
			ret = true;
			break;
		}
	}
	return ret;
}

void __set_unique_id(struct npu_session *session, struct drv_usr_share *usr_data)
{
	u32 uid = session->uid;
	usr_data->id = uid;
}

int __set_unified_id(struct npu_session *session, struct drv_usr_share *usr_data)
{
	int ret = 0;

	session->unified_op_id = usr_data->unified_op_id;
	ret = npu_sessionmgr_set_unifiedID(session);
	return ret;
}

#if IS_ENABLED(CONFIG_SOC_S5E9955) || IS_ENABLED(CONFIG_NPU_GOVERNOR)
static u32 npu_get_hash_name_key(const char *model_name,
		unsigned int computational_workload,
		unsigned int io_workload)
{
	u32 key = 0;
	char c = 0;
	int i = 0;

	key |= ((u32)computational_workload) << 16;
	key |= ((u32)io_workload) & 0x0000FFFF;

	while ((c = *model_name++)) {
		key |= ((u32)c) << ((8 * i++) & 31);
	}

	return key;
}
#endif

#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
bool is_matching_ncp_for_hash(struct npu_model_info_hash *h, struct ncp_header *ncp)
{
	return (!strcmp(h->model_name, ncp->model_name)) &&
		(h->computational_workload == ncp->computational_workload) &&
		(h->io_workload == ncp->io_workload);
}
#endif

int update_ncp_info(struct npu_session *session, struct npu_memory_buffer *model_mem_buf, u32 memory_type)
{
	int ret = 0;

	if (!session || !model_mem_buf) {
		npu_err("invalid session or model_mem_buf\n");
		return -EINVAL;
	}

	switch (memory_type) {
	case MEMORY_TYPE_NCP:
		{
			if (session->ncp_mem_buf_offset > model_mem_buf->dma_buf->size) {
				npu_uerr("ncp_offset(%lu) is larger than buffer size(%zu).\n",
					session, session->hids, session->ncp_mem_buf_offset, model_mem_buf->dma_buf->size);
				ret = -EINVAL;
				goto p_err;
			}

			session->ncp_info.ncp_addr.size = model_mem_buf->size;
			session->ncp_info.ncp_addr.vaddr = model_mem_buf->vaddr + session->ncp_mem_buf_offset;
			session->ncp_info.ncp_addr.daddr = model_mem_buf->daddr + session->ncp_mem_buf_offset;

			npu_udbg("ncp offset (%lu) + ncp_ion_map(0x%pad) vaddr(0x%pK)\n",
			session, session->hids, session->ncp_mem_buf_offset,
			&model_mem_buf->daddr, model_mem_buf->vaddr);
			break;
		}
	case MEMORY_TYPE_WEIGHT:
		{
			break;
		}
	default:
		npu_uerr("can't support memory type\n", session, session->hids);
		ret = -EINVAL;
		break;
	}

p_err:
	return ret;
}

#ifndef CONFIG_NPU_KUNIT_TEST
void npu_session_dump(struct npu_session *session)
{
	size_t IFM_cnt;
	size_t OFM_cnt;
	size_t IMB_cnt;
	size_t WGT_cnt;
	size_t idx1 = 0;

	struct av_info *IFM_info;
	struct av_info *OFM_info;
	struct addr_info *IFM_addr;
	struct addr_info *OFM_addr;
	struct addr_info *IMB_addr;
	struct addr_info *WGT_addr;

	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;

	if (!session)
		return;

	IFM_cnt = session->IFM_cnt;
	OFM_cnt = session->OFM_cnt;
	IMB_cnt = session->IMB_cnt;
	WGT_cnt = session->WGT_cnt;

	vctx = &session->vctx;
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

	npu_dump("npu session id = %d, state = %lu\n",session->uid, session->ss_state);
	npu_dump("npu session unified id = %d, unified op id = %lld, hw id = %d\n",
		session->unified_id, session->unified_op_id, session->hids);
	npu_dump("- index type vaddr                daddr              size\n");
	npu_dump("------------ session IFM/OFM memory ------------\n");

	IFM_info = &session->IFM_info;
	OFM_info = &session->OFM_info;
	IFM_addr = IFM_info->addr_info;
	OFM_addr = OFM_info->addr_info;

	if (IFM_addr) {
		for (idx1 = 0; idx1 < IFM_cnt; idx1++) {
			npu_dump("- %zu      %d   %pK   %pad   %zu\n", idx1,
				 MEMORY_TYPE_IN_FMAP, (IFM_addr + idx1)->vaddr,
			 &((IFM_addr + idx1)->daddr), (IFM_addr + idx1)->size);
		}
	}

	if (OFM_addr) {
		for (idx1 = 0; idx1 < OFM_cnt; idx1++) {
			npu_dump("- %zu      %d   %pK   %pad   %zu\n", idx1,
				 MEMORY_TYPE_OT_FMAP, (OFM_addr + idx1)->vaddr,
				 &((OFM_addr + idx1)->daddr), (OFM_addr + idx1)->size);
		}
	}

	if (session->hids & NPU_HWDEV_ID_NPU) {
		npu_dump("------------ session IMB/WGT memory ------------\n");
		IMB_addr = session->IMB_info;
		WGT_addr = session->WGT_info;
		if (IMB_addr) {
			for (idx1 = 0; idx1 < IMB_cnt; idx1++) {
				npu_dump("- %zu      %d   %pK   %pad   %zu\n", idx1,
					 MEMORY_TYPE_IM_FMAP, (IMB_addr + idx1)->vaddr,
					 &((IMB_addr + idx1)->daddr), (IMB_addr + idx1)->size);
			}
		}

		if (WGT_addr) {
			for (idx1 = 0; idx1 < WGT_cnt; idx1++) {
				npu_dump("- %zu      %d   %pK   %pad   %zu\n", idx1,
					 MEMORY_TYPE_WEIGHT, (WGT_addr + idx1)->vaddr,
					 &((WGT_addr + idx1)->daddr), (WGT_addr + idx1)->size);
			}
		}
	}

	npu_dump("--------- session NCP/IOFM/IMB memory ----------\n");
	npu_memory_buffer_dump(session->ncp_mem_buf);
	npu_memory_buffer_dump(session->ncp_hdr_buf);
	npu_memory_buffer_dump(session->IOFM_mem_buf);
	npu_dump("------------------------------------------------\n");
}
#endif

static inline u32 __get_dl_unique_id(void)
{
	u32 id = 0;

#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
	id = ++dl_unique_id;
	if (dl_unique_id == U32_MAX) {
		dl_unique_id = 0;
	}
#endif

	return id;
}

int __shared_ion_map(struct npu_session *session, struct vs4l_graph *info)
{
	int ret = 0;
	struct npu_memory_buffer *shared_mem_buf = NULL;
	struct npu_system *system = get_session_system(session);
	struct dhcp_table *dhcp_table = system->dhcp_table;

	shared_mem_buf = kzalloc(sizeof(struct npu_memory_buffer), GFP_KERNEL);
	if (shared_mem_buf == NULL) {
		npu_uerr("fail in npu_ion_map kzalloc\n", session, session->hids);
		ret = -ENOMEM;
		goto p_err;
	}

	shared_mem_buf->m.fd = info->priority;
	shared_mem_buf->size = info->size;
	ret = npu_memory_map(session->memory, shared_mem_buf, npu_get_configs(NPU_PBHA_HINT_11),
								session->lazy_unmap_disable, FALSE);
	if (ret < 0) {
		npu_uerr("fail in npu_memory_map\n", session, session->hids);
		kfree(shared_mem_buf);
		goto p_err;
	}

	dhcp_table->PDCL_MBOX_ADDR = shared_mem_buf->daddr;
	session->shared_mem_buf = shared_mem_buf;

p_err:
	return ret;
}

static struct npu_memory_buffer *
copy_ncp_header(struct npu_session *session, struct npu_memory_buffer *ncp_mem_buf)
{
	struct npu_memory_buffer *buffer;
	struct ncp_header *src_hdr = ncp_mem_buf->vaddr + session->ncp_mem_buf_offset;

	buffer = npu_memory_copy(session->memory, ncp_mem_buf,
				session->ncp_mem_buf_offset, src_hdr->hdr_size);
	if (IS_ERR_OR_NULL(buffer))
		return buffer;

	add_ion_mem(session, buffer, MEMORY_TYPE_NCP_HDR);

	return buffer;
}

static int model_ion_map(struct npu_session *session, struct drv_usr_share *usr_data, u32 memory_type)
{
	int ret = 0;
	struct npu_memory_buffer *model_mem_buf = NULL, *ncp_hdr_buf = NULL;

	model_mem_buf = kzalloc(sizeof(struct npu_memory_buffer), GFP_KERNEL);
	if (model_mem_buf == NULL) {
		npu_uerr("fail in npu_ion_map kzalloc\n", session, session->hids);
		return -ENOMEM;
	}

	switch (memory_type) {
	case MEMORY_TYPE_NCP:
		{
			model_mem_buf->m.fd = usr_data->ncp_fd;
			model_mem_buf->size = usr_data->ncp_size;
			model_mem_buf->ncp = true;

			session->ncp_mem_buf_offset = usr_data->ncp_offset;

			if (session->hids == NPU_HWDEV_ID_NPU)
				strcpy(model_mem_buf->name, "NCP");
			else
				strcpy(model_mem_buf->name, "UCGO");

			break;
		}
	case MEMORY_TYPE_WEIGHT:
		{
			model_mem_buf->m.fd = usr_data->ncp_fd;
			model_mem_buf->size = usr_data->ncp_size;
			model_mem_buf->reserved = usr_data->reserved1;
			session->weight_mem_buf_offset = usr_data->ncp_offset;

			break;
		}
	default:
		npu_uerr("can't support memory type\n", session, session->hids);
		ret = -EINVAL;
		goto err_free;
	}

	if (model_mem_buf->m.fd <= 0) {
		ret = -EBADF;
		goto err_free;
	}

	if (usr_data->reserved1 != VS4L_MEMORY_DMABUF_DYNAMIC) {
		ret = npu_memory_map(session->memory, model_mem_buf, npu_get_configs(NPU_PBHA_HINT_11), session->lazy_unmap_disable, TRUE);
		if (unlikely(ret)) {
			npu_err("npu_memory_map is fail(%d).\n", ret);
			goto err_free;
		}
	}

	ret = update_ncp_info(session, model_mem_buf, memory_type);
	if (ret) {
		npu_uerr("fail(%d).\n", session, session->hids, ret);
		goto err_unmap;
	}

	ret = add_ion_mem(session, model_mem_buf, memory_type);
	if (ret) {
		npu_uerr("fail(%d).\n", session, session->hids, ret);
		goto err_unmap;
	}

	if (memory_type == MEMORY_TYPE_NCP) {
		ncp_hdr_buf = copy_ncp_header(session, model_mem_buf);
		if (IS_ERR_OR_NULL(ncp_hdr_buf))
			return PTR_ERR(ncp_hdr_buf); /* free buffer from __release_graph_ion */
#if IS_ENABLED(CONFIG_SOC_S5E8855)
		ret = npu_util_validate_user_ncp(session, ncp_hdr_buf->vaddr, model_mem_buf->size);
		if (ret)
			return ret; /* free buffer from __release_graph_ion */
#endif
	}

	return ret;
err_unmap:
	npu_memory_unmap(session->memory, model_mem_buf);
err_free:
	kfree(model_mem_buf);
	return ret;
}

int __get_session_info(struct npu_session *session, struct vs4l_graph *info)
{
	int ret = 0;
	size_t fd_size = 0;
	struct npu_vertex *vertex;
	struct npu_device *device;
	struct drv_usr_share *usr_data = NULL;

	vertex = session->vctx.vertex;
	device = container_of(vertex, struct npu_device, vertex);

	// To be removed
	if (info->size == 0)
		info->size++;

	if (device->sched->mode == NPU_PERF_MODE_NPU_BOOST_PRUNE)
		fd_size = 1;
	else
		fd_size = info->size;

	usr_data = kzalloc(fd_size * sizeof(struct drv_usr_share), GFP_KERNEL);
	if (unlikely(!usr_data)) {
		npu_uerr("Not enough Memory\n", session, session->hids);
		ret = -ENOMEM;
		return ret;
	}
	ret = copy_from_user((void *)usr_data, (void *)info->addr, fd_size * sizeof(struct drv_usr_share));
	if (ret) {
		npu_uerr("copy_from_user failed(%d)\n", session, session->hids, ret);
		goto p_err;
	}
	__set_unique_id(session, usr_data);

	ret = __set_unified_id(session, usr_data);
	if (ret < 0) {
		npu_uerr("__set_unified_id failed(%d)\n", session, session->hids, ret);
		goto p_err;
	}

	ret = copy_to_user((void *)info->addr, (void *)usr_data, fd_size * sizeof(struct drv_usr_share));
	if (ret) {
		npu_uerr("copy_to_user failed(%d)\n", session, session->hids, ret);
		goto p_err;
	}
	ret = model_ion_map(session, usr_data, MEMORY_TYPE_NCP);
	if (unlikely(ret)) {
		npu_uerr("model_ion_map is fail(%d) - MEMORY_TYPE_NCP\n", session, session->hids, ret);
		goto p_err;
	}

	if (fd_size > 1) {
		ret = model_ion_map(session, &usr_data[1], MEMORY_TYPE_WEIGHT);
		if (unlikely(ret)) {
			npu_uerr("model_ion_map is fail(%d) - MEMORY_TYPE_WEIGHT\n", session, session->hids, ret);
			goto p_err;
		}
	}

	session->ss_state |= BIT(NPU_SESSION_STATE_GRAPH_ION_MAP);
p_err:
	if (usr_data)
		kfree(usr_data);
	return ret;
}

int __pilot_parsing_ncp(struct npu_session *session, u32 *IFM_cnt, u32 *OFM_cnt, u32 *IMB_cnt, u32 *WGT_cnt)
{
	int ret = 0;
	u32 i = 0;
	char *ncp_vaddr;
	u32 memory_vector_cnt;
	u32 memory_vector_offset;
	struct memory_vector *mv;
	struct ncp_header *ncp;
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	struct npu_model_info_hash *h;
	struct npu_sessionmgr *sessionmgr;
	u32 key;

	sessionmgr = session->cookie;
#endif
	ncp_vaddr = (char *)session->ncp_hdr_buf->vaddr;

	ncp = (struct ncp_header *)ncp_vaddr;

#if (NCP_VERSION <= 27)
	if (ncp->thread_vector_cnt == 2)
		session->is_instance_1 = true;

	npu_uinfo("session is instance %s\n", session, session->hids, session->is_instance_1 ? "1" : "N");

	memory_vector_offset = ncp->memory_vector_offset;
	if (unlikely(memory_vector_offset > session->ncp_mem_buf->size)) {
		npu_uerr("memory vector offset(0x%x) > max size(0x%x) ;out of bounds\n",
				session, session->hids, (u32)memory_vector_offset, (u32)session->ncp_mem_buf->size);
		return -EFAULT;
	}

	memory_vector_cnt = ncp->memory_vector_cnt;
#else
	if (ncp->vector_list[NCP_VECTOR_TYPE_THREAD].cnt == 2)
		session->is_instance_1 = true;

	npu_uinfo("session is instance %s\n", session, session->hids, session->is_instance_1 ? "1" : "N");

	memory_vector_offset = ncp->vector_list[NCP_VECTOR_TYPE_MEMORY].offset;
	if (unlikely(memory_vector_offset > session->ncp_mem_buf->size)) {
		npu_uerr("memory vector offset(0x%x) > max size(0x%x) ;out of bounds\n",
				session, session->hids, (u32)memory_vector_offset, (u32)session->ncp_mem_buf->size);
		return -EFAULT;
	}

	memory_vector_cnt = ncp->vector_list[NCP_VECTOR_TYPE_MEMORY].cnt;
#endif
	if (unlikely(((memory_vector_cnt * sizeof(struct memory_vector)) + memory_vector_offset) >  session->ncp_mem_buf->size)) {
		npu_uerr("memory vector count(0x%x) seems abnormal ;out of bounds\n", session, session->hids, memory_vector_cnt);
		return -EFAULT;
	}
	session->memory_vector_offset = memory_vector_offset;
	session->memory_vector_cnt = memory_vector_cnt;

	mv = (struct memory_vector *)(ncp_vaddr + memory_vector_offset);

	for (i = 0; i < memory_vector_cnt; i++) {
		u32 memory_type = (mv + i)->type;

		switch (memory_type) {
		case MEMORY_TYPE_IN_FMAP:
			{
				(*IFM_cnt)++;
				break;
			}
		case MEMORY_TYPE_OT_FMAP:
			{
				(*OFM_cnt)++;
				break;
			}
		case MEMORY_TYPE_IM_FMAP:
			{
				(*IMB_cnt)++;
				break;
			}
#if NCP_VERSION <= 27
		case MEMORY_TYPE_WMASK:
#else
		case MEMORY_TYPE_CDATA:
		case MEMORY_TYPE_RODATA:
		case MEMORY_TYPE_CUCODE:
			break;
#endif
		case MEMORY_TYPE_WEIGHT:
			{
				(*WGT_cnt)++;
				break;
			}
		}
	}

#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	npu_governor_make_ncp_valid(ncp);
	key = npu_get_hash_name_key(ncp->model_name, ncp->computational_workload, ncp->io_workload);
	mutex_lock(&sessionmgr->model_lock);
	hash_for_each_possible(sessionmgr->model_info_hash_head, h, hlist, key) {
		if (is_matching_ncp_for_hash(h, ncp)) {
			session->model_hash_node = h;
			break;
		}
	}
	mutex_unlock(&sessionmgr->model_lock);
	if (session->model_hash_node == 0) {
		session->model_hash_node = kcalloc(1, sizeof(struct npu_model_info_hash), GFP_KERNEL);
		if (!session->model_hash_node) {
			npu_uerr("Failed to alloc session->model_hash_node\n", session, session->hids);
			return -ENOMEM;
		}

		session->model_hash_node->alpha = MAX_ALPHA;
		session->model_hash_node->beta = MAX_BETA;
		session->check_core = false;
		session->model_hash_node->miss_cnt = 0;
		session->model_hash_node->computational_workload = ncp->computational_workload;
		session->model_hash_node->io_workload = ncp->io_workload;
		strcpy(session->model_hash_node->model_name, ncp->model_name);
		init_workload_params(session);

		mutex_lock(&sessionmgr->model_lock);
		hash_add(sessionmgr->model_info_hash_head, &session->model_hash_node->hlist, key);
		mutex_unlock(&sessionmgr->model_lock);
	}
#endif

#if IS_ENABLED(CONFIG_SOC_S5E9955)
	{
		struct npu_device *device;
		struct npu_vertex *vertex;

		vertex = session->vctx.vertex;
		device = container_of(vertex, struct npu_device, vertex);

		if (device->sched->mode != NPU_PERF_MODE_NPU_BOOST &&
				device->sched->mode != NPU_PERF_MODE_NPU_BOOST_PRUNE) {
			session->key = npu_get_hash_name_key(ncp->model_name, ncp->computational_workload, ncp->io_workload);
			session->computational_workload = ncp->computational_workload;
			session->io_workload = ncp->io_workload;
#if NCP_VERSION <= 27
			session->featuremapdata_type = ncp->featuremapdata_type;
#else
			session->featuremapdata_type = ncp->operation_group;
#endif
			strncpy(session->model_name, ncp->model_name, NCP_MODEL_NAME_LEN);
#if IS_ENABLED(CONFIG_NPU_CHECK_PRECISION)
			npu_precision_active_info_hash_add(session);
#endif
		}
	}
#else
	strncpy(session->model_name, ncp->model_name, NCP_MODEL_NAME_LEN);
#endif

	return ret;
}

int parsing_ncp_memory_vector(struct npu_session *session,
	struct addr_info **IFM_av, struct addr_info **OFM_av,
	struct addr_info **IMB_av, struct addr_info **WGT_av)
{
	int i = 0;

	u32 IFM_cnt = 0;
	u32 OFM_cnt = 0;
	u32 IMB_cnt = 0;
	u32 WGT_cnt = 0;

	u32 address_vector_offset;
	u32 address_vector_cnt;
	u32 memory_vector_offset;
	u32 memory_vector_cnt;
	u32 weight_offset = 0;
	u32 weight_size;
	u32 body_offset;

	u32 memory_type = 0;
	u32 address_vector_index = 0;

#if NCP_VERSION > 27 && IS_ENABLED(CONFIG_SOC_S5E8855)
	u32 memory_offset = 0;
	u32 memory_size;
#endif

	struct ncp_header *ncp;
	struct address_vector *av;
	struct address_vector *av_wgt;
	struct memory_vector *mv;

	char *ncp_vaddr;
	dma_addr_t ncp_daddr;

	ncp_vaddr = (char *)session->ncp_hdr_buf->vaddr;

	ncp = (struct ncp_header *)ncp_vaddr;

#if IS_ENABLED(CONFIG_SOC_S5E9955)
	// to be removed
	if (session->hids == NPU_HWDEV_ID_NPU)
		ncp->device_id = 0 << 4;
	else
		ncp->device_id = 1 << 4;
#endif

	if (ncp->model_name[0] != '\0') {
		strncpy(session->ncp_hdr_buf->name, ncp->model_name, sizeof(session->ncp_hdr_buf->name) - 1);
		strncpy(session->ncp_mem_buf->name, ncp->model_name, sizeof(session->ncp_mem_buf->name) - 1);
	}

#if NCP_VERSION <= 27
	address_vector_offset = ncp->address_vector_offset;
#else
	address_vector_offset = ncp->vector_list[NCP_VECTOR_TYPE_ADDRESS].offset;
#endif
	if (unlikely(address_vector_offset > session->ncp_mem_buf->size)) {
		npu_uerr("address vector offset(0x%x) > max size(0x%x) ;out of bounds\n",
				session, session->hids, address_vector_offset, (u32)session->ncp_mem_buf->size);
		return -EFAULT;
	}

#if NCP_VERSION <= 27
	address_vector_cnt = ncp->address_vector_cnt;
#else
	address_vector_cnt = ncp->vector_list[NCP_VECTOR_TYPE_ADDRESS].cnt;
#endif
	if (unlikely(((address_vector_cnt * sizeof(struct address_vector)) + address_vector_offset) >
									session->ncp_mem_buf->size)) {
		npu_uerr("address vector count(0x%x) seems abnormal ;out of bounds\n", session, session->hids, address_vector_cnt);
		return -EFAULT;
	}

	session->address_vector_offset = address_vector_offset;
	session->address_vector_cnt = address_vector_cnt;

	session->ncp_info.address_vector_cnt = address_vector_cnt;

	memory_vector_offset = session->memory_vector_offset;
	memory_vector_cnt = session->memory_vector_cnt;
#if NCP_VERSION <= 27
	ncp->unique_id |= session->unified_id << 16;
	ncp->fence_flag = 1 << session->uid;
#else
	ncp->nnc_id |= (session->unified_id << 16);
#endif

	mv = (struct memory_vector *)(ncp_vaddr + memory_vector_offset);
	av = (struct address_vector *)(ncp_vaddr + address_vector_offset);

	ncp_daddr = session->ncp_mem_buf->daddr;
	ncp_daddr += session->ncp_mem_buf_offset;

	for (i = 0; i < memory_vector_cnt; i++) {
		memory_type = (mv + i)->type;

		switch (memory_type) {
		case MEMORY_TYPE_IN_FMAP:
			{
				if (IFM_cnt >= session->IFM_cnt) {
					npu_uerr("IFM_cnt(%d) should not exceed size of allocated array(%d)\n",
							session, session->hids, IFM_cnt, session->IFM_cnt);
					return -EFAULT;
				}
				address_vector_index = (mv + i)->address_vector_index;
				if (likely(!EVER_FIND_FM(&IFM_cnt, *IFM_av, address_vector_index))) {
					(*IFM_av + IFM_cnt)->av_index = address_vector_index;
					if (unlikely(((address_vector_index * sizeof(struct address_vector))
						+ address_vector_offset) > session->ncp_mem_buf->size) ||
						unlikely(address_vector_index >= address_vector_cnt)) {
						npu_uerr("address_vector_index(%d) should not exceed max addr vec count(%d)\n",
								session, session->hids, address_vector_index, address_vector_cnt);
						return -EFAULT;
					}
#if IS_ENABLED(CONFIG_SOC_S5E8855)
					(*IFM_av + IFM_cnt)->size = (av + address_vector_index)->size;
					(*IFM_av + IFM_cnt)->pixel_format = (mv + i)->pixel_format;
					(*IFM_av + IFM_cnt)->width = (mv + i)->width;
					(*IFM_av + IFM_cnt)->height = (mv + i)->height;
					(*IFM_av + IFM_cnt)->channels = (mv + i)->channels;
					(mv + i)->wstride = 0;
					(*IFM_av + IFM_cnt)->stride = (mv + i)->wstride;
					npu_udbg("(IFM_av + %u)->index = %u\n", session, session->hids,
						IFM_cnt, (*IFM_av + IFM_cnt)->av_index);
					npu_udbg("[session] IFM, index(%u)\n", session, session->hids, (*IFM_av + IFM_cnt)->av_index);
					npu_udbg("[session] IFM, size(%zu)\n", session, session->hids, (*IFM_av + IFM_cnt)->size);
					npu_udbg("[session] IFM, pixel_format(%u)\n", session, session->hids, (*IFM_av + IFM_cnt)->pixel_format);
					npu_udbg("[session] IFM, width(%u)\n", session, session->hids, (*IFM_av + IFM_cnt)->width);
					npu_udbg("[session] IFM, height(%u)\n", session, session->hids,(*IFM_av + IFM_cnt)->height);
					npu_udbg("[session] IFM, channels(%u)\n", session, session->hids,(*IFM_av + IFM_cnt)->channels);
					npu_udbg("[session] IFM, wstride(%u)\n", session, session->hids,(*IFM_av + IFM_cnt)->stride);
#endif
					IFM_cnt++;
				}
				break;
			}
		case MEMORY_TYPE_OT_FMAP:
			{
				if (OFM_cnt >= session->OFM_cnt) {
					npu_uerr("OFM_cnt(%d) should not exceed size of allocated array(%d)\n",
							session, session->hids, OFM_cnt, session->OFM_cnt);
					return -EFAULT;
				}
				address_vector_index = (mv + i)->address_vector_index;
				if (likely(!EVER_FIND_FM(&OFM_cnt, *OFM_av, address_vector_index))) {
					(*OFM_av + OFM_cnt)->av_index = address_vector_index;
					if (unlikely(((address_vector_index * sizeof(struct address_vector))
						+ address_vector_offset) > session->ncp_mem_buf->size) ||
						unlikely(address_vector_index >= address_vector_cnt)) {
						npu_err("address_vector_index(%d) should not exceed max addr vec count(%d)\n",
								address_vector_index, address_vector_cnt);
						return -EFAULT;
					}
#if IS_ENABLED(CONFIG_SOC_S5E8855)
					(*OFM_av + OFM_cnt)->size = (av + address_vector_index)->size;
					(*OFM_av + OFM_cnt)->pixel_format = (mv + i)->pixel_format;
					(*OFM_av + OFM_cnt)->width = (mv + i)->width;
					(*OFM_av + OFM_cnt)->height = (mv + i)->height;
					(*OFM_av + OFM_cnt)->channels = (mv + i)->channels;
					(mv + i)->wstride = 0;
					(*OFM_av + OFM_cnt)->stride = (mv + i)->wstride;
					npu_udbg("(OFM_av + %u)->index = %u\n", session, session->hids, OFM_cnt, (*OFM_av + OFM_cnt)->av_index);
					npu_udbg("OFM, index(%d)\n", session, session->hids, (*OFM_av + OFM_cnt)->av_index);
					npu_udbg("[session] OFM, size(%zu)\n", session, session->hids, (*OFM_av + OFM_cnt)->size);
					npu_udbg("[session] OFM, pixel_format(%u)\n", session, session->hids, (*OFM_av + OFM_cnt)->pixel_format);
					npu_udbg("[session] OFM, width(%u)\n", session, session->hids, (*OFM_av + OFM_cnt)->width);
					npu_udbg("[session] OFM, height(%u)\n", session, session->hids, (*OFM_av + OFM_cnt)->height);
					npu_udbg("[session] OFM, channels(%u)\n", session, session->hids,(*OFM_av + OFM_cnt)->channels);
					npu_udbg("[session] OFM, wstride(%u)\n", session, session->hids,(*OFM_av + OFM_cnt)->stride);
#endif
					OFM_cnt++;
				}
				break;
			}
		case MEMORY_TYPE_IM_FMAP:
			{
				if (IMB_cnt >= session->IMB_cnt) {
					npu_uerr("IMB: IMB_cnt(%d) should not exceed size of allocated array(%d)\n",
							session, session->hids, IMB_cnt, session->IMB_cnt);
					return -EFAULT;
				}
				address_vector_index = (mv + i)->address_vector_index;
				if (likely(!EVER_FIND_FM(&IMB_cnt, *IMB_av, address_vector_index))) {
					(*IMB_av + IMB_cnt)->av_index = address_vector_index;
					if (unlikely(((address_vector_index * sizeof(struct address_vector))
						+ address_vector_offset) > session->ncp_mem_buf->size) ||
						unlikely(address_vector_index >= address_vector_cnt)) {
						npu_uerr("IMB: address_vector_index(%d) should not exceed max addr vec count(%d)\n",
								session, session->hids, address_vector_index, address_vector_cnt);
						return -EFAULT;
					}
					(*IMB_av + IMB_cnt)->size = (av + address_vector_index)->size;
					(*IMB_av + IMB_cnt)->pixel_format = (mv + i)->pixel_format;
					(*IMB_av + IMB_cnt)->width = (mv + i)->width;
					(*IMB_av + IMB_cnt)->height = (mv + i)->height;
					(*IMB_av + IMB_cnt)->channels = (mv + i)->channels;
					(mv + i)->wstride = 0;
					(*IMB_av + IMB_cnt)->stride = (mv + i)->wstride;
					npu_udbg("[session] IMB, (*IMB_av + %u)->index = %x\n",
							session, session->hids, IMB_cnt, (*IMB_av + IMB_cnt)->av_index);
					npu_udbg("[session] IMB, size(%zu)\n", session, session->hids, (*IMB_av + IMB_cnt)->size);
					IMB_cnt++;
				}
				break;
			}
#if IS_ENABLED(CONFIG_SOC_S5E8855)
#if NCP_VERSION <= 27
		case MEMORY_TYPE_WMASK:
#else
		case MEMORY_TYPE_CDATA:
		case MEMORY_TYPE_RODATA:
		case MEMORY_TYPE_CUCODE:
			{
				// update address vector, m_addr with ncp_alloc_daddr + offset
				address_vector_index = (mv + i)->address_vector_index;

				if (unlikely(((address_vector_index * sizeof(struct address_vector))
						+ address_vector_offset) > session->ncp_mem_buf->size) ||
						unlikely(address_vector_index >= address_vector_cnt)) {
					npu_uerr("address_vector_index(%d) should not exceed max addr vec count(%d)\n",
							session, session->hids, address_vector_index, address_vector_cnt);
					return -EFAULT;
				}
				memory_offset = (av + address_vector_index)->m_addr;
				if (unlikely(memory_offset > (u32)session->ncp_mem_buf->size)) {
					npu_uerr("memory_offset is invalid, offset(0x%x), ncp_daddr(0x%x)\n",
						session, session->hids, (u32)memory_offset, (u32)session->ncp_mem_buf->size);
					return -EINVAL;
				}

				(av + address_vector_index)->m_addr = ncp_daddr + ncp->body_offset + memory_offset;

				memory_size = (av + address_vector_index)->size;
					if (unlikely((memory_offset + memory_size) < memory_size)) {
						npu_uerr("memory_offset(0x%x) + weight size (0x%x) seems to be overflow.\n",
								session, session->hids, memory_offset, memory_size);
						return -ERANGE;
					}
					if (unlikely((memory_offset + memory_size) > (u32)session->ncp_mem_buf->size)) {
						npu_uerr("memory_offset(0x%x) + weight size (0x%x) seems to go beyond ncp size(0x%x)\n",
								session, session->hids, memory_offset, memory_size, (u32)session->ncp_mem_buf->size);
						return -EFAULT;
					}

				break;
			}
#endif
#endif
		case MEMORY_TYPE_WEIGHT:
			{
				ncp_vaddr = (char *)session->ncp_mem_buf->vaddr;
				ncp_vaddr += session->ncp_mem_buf_offset;

				av_wgt = (struct address_vector *)(ncp_vaddr + address_vector_offset);

				if (WGT_cnt >= session->WGT_cnt) {
					npu_uerr("WGT_cnt(%d) should not exceed size of allocated array(%d)\n",
							session, session->hids, WGT_cnt, session->WGT_cnt);
					return -EFAULT;
				}
				// update address vector, m_addr with ncp_alloc_daddr + offset
				address_vector_index = (mv + i)->address_vector_index;
				session->wgt_av_index = address_vector_index;
				if (unlikely(((address_vector_index * sizeof(struct address_vector))
						+ address_vector_offset) > session->ncp_mem_buf->size) ||
						unlikely(address_vector_index >= address_vector_cnt)) {
					npu_uerr("address_vector_index(%d) should not exceed max addr vec count(%d)\n",
							session, session->hids, address_vector_index, address_vector_cnt);
					return -EFAULT;
				}
				weight_offset = (av + address_vector_index)->m_addr;
				if (unlikely(weight_offset > (u32)session->ncp_mem_buf->size)) {
					npu_uerr("weight_offset is invalid, offset(0x%x), ncp_daddr(0x%x)\n",
						session, session->hids, (u32)weight_offset, (u32)session->ncp_mem_buf->size);
					return -EINVAL;
				}

				if (session->weight_mem_buf != NULL &&
						session->weight_mem_buf->reserved == VS4L_MEMORY_DMABUF_DYNAMIC) {
						break;
				}

				if (weight_offset == 0) {
					if (!session->weight_mem_buf || session->weight_mem_buf->m.fd <= 0)
						return -EFAULT;
					if ((av + address_vector_index)->size > session->weight_mem_buf->size)
						return -EFAULT;

					(av + address_vector_index)->m_addr = session->weight_mem_buf->daddr + session->weight_mem_buf_offset;
					(av_wgt + address_vector_index)->m_addr = session->weight_mem_buf->daddr + session->weight_mem_buf_offset;

					(*WGT_av + WGT_cnt)->size = session->weight_mem_buf->size;
					(*WGT_av + WGT_cnt)->daddr = session->weight_mem_buf->daddr + session->weight_mem_buf_offset;
					(*WGT_av + WGT_cnt)->vaddr = session->weight_mem_buf->vaddr + session->weight_mem_buf_offset;
					(*WGT_av + WGT_cnt)->memory_type = memory_type;
				} else {
#if NCP_VERSION < 27
					body_offset = 0;
#else
					body_offset = ncp->body_offset;
					(av_wgt + address_vector_index)->m_addr = ncp_daddr + body_offset + weight_offset;
#endif

					(av + address_vector_index)->m_addr = ncp_daddr + body_offset + weight_offset;


					(*WGT_av + WGT_cnt)->av_index = address_vector_index;
					weight_size = (av + address_vector_index)->size;
					if (unlikely((weight_offset + weight_size) < weight_size)) {
						npu_uerr("weight_offset(0x%x) + weight size (0x%x) seems to be overflow.\n",
								session, session->hids, weight_offset, weight_size);
						return -ERANGE;
					}
					if (unlikely((weight_offset + weight_size) > (u32)session->ncp_mem_buf->size)) {
						npu_uerr("weight_offset(0x%x) + weight size (0x%x) seems to go beyond ncp size(0x%x)\n",
								session, session->hids, weight_offset, weight_size, (u32)session->ncp_mem_buf->size);
						return -EFAULT;
					}

					(*WGT_av + WGT_cnt)->size = weight_size;
					(*WGT_av + WGT_cnt)->daddr =  ncp_daddr + body_offset + weight_offset;
					(*WGT_av + WGT_cnt)->vaddr =  ncp_vaddr + body_offset + weight_offset;
					(*WGT_av + WGT_cnt)->memory_type = memory_type;
				}
				WGT_cnt++;
				break;
			}
		default:
			break;
		}
	}
	session->IOFM_cnt = IFM_cnt + OFM_cnt;

	return 0;
}

#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
int parsing_ncp_kernel_vector(struct npu_session *session)
{
	int ret = 0;
	int i = 0;

	struct ncp_header *ncp;
	char *hdr_vaddr;
	char *ncp_vaddr;

	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	struct dsp_dl_lib_file gkt;
	struct kernel_vector *dsp_kernel_bin_vector;
	struct descriptor_vector *dsp_kernel_desc_vector;

	u32 kernel_desc_vector_offset;
	u32 kernel_desc_vector_cnt;
	u32 kernel_bin_vector_offset;
	u32 kernel_bin_vector_cnt;
	u32 body_offset;

	hdr_vaddr = (char *)session->ncp_hdr_buf->vaddr;
	ncp_vaddr = (char *)session->ncp_mem_buf->vaddr + session->ncp_mem_buf_offset;
	ncp = (struct ncp_header *)hdr_vaddr;

	vctx = &session->vctx;
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

	kernel_desc_vector_offset = ncp->vector_list[NCP_VECTOR_TYPE_DESCRIPTOR].offset;
	if (kernel_desc_vector_offset > session->ncp_mem_buf->size) {
		npu_uerr("kernel desc vector offset(0x%x) > max size(0x%x), out of bounds\n",
				session, session->hids, kernel_desc_vector_offset, (u32)session->ncp_mem_buf->size);
		return -EFAULT;
	}
	kernel_desc_vector_cnt = ncp->vector_list[NCP_VECTOR_TYPE_DESCRIPTOR].cnt;

	kernel_bin_vector_offset = ncp->vector_list[NCP_VECTOR_TYPE_KERNEL].offset;
	if (kernel_bin_vector_offset > session->ncp_mem_buf->size) {
		npu_uerr("kernel bin vector offset(0x%x) > max size(0x%x), out of bounds\n",
				session, session->hids, kernel_bin_vector_offset, (u32)session->ncp_mem_buf->size);
		return -EFAULT;
	}
	kernel_bin_vector_cnt = ncp->vector_list[NCP_VECTOR_TYPE_KERNEL].cnt;

	if (!kernel_desc_vector_cnt || !kernel_bin_vector_cnt) {
		npu_uerr("please check descriptor vector(%d) & kernel vector(%d)\n",
					session, session->hids, kernel_desc_vector_cnt, kernel_bin_vector_cnt);
		return -EINVAL;
	}

	mutex_lock(session->global_lock);
	dsp_kernel_desc_vector = (struct descriptor_vector *)(ncp_vaddr + kernel_desc_vector_offset);
	session->dl_unique_id = __get_dl_unique_id();
#if NCP_VERSION < 27
	body_offset = 0;
#else
	body_offset = ncp->body_offset;
#endif

	for (i = 0; i < kernel_desc_vector_cnt; i++) {
		gkt.mem = ncp_vaddr + body_offset + (dsp_kernel_desc_vector + i)->offset;
		gkt.size = (dsp_kernel_desc_vector + i)->size;
		ret = dsp_dl_add_gkt(&gkt, &session->str_manager, session->dl_unique_id);
		if (ret) {
			mutex_unlock(session->global_lock);
			npu_uerr("failed to parse gkt.xml(%d).\n", session, session->hids, ret);
			return ret;
		}
	}

	dsp_kernel_bin_vector = (struct kernel_vector *)(ncp_vaddr + kernel_bin_vector_offset);
	session->kernel_count = kernel_bin_vector_cnt;
	session->dl_libs = kcalloc(session->kernel_count, sizeof(*session->dl_libs),
			GFP_KERNEL);
	if (!session->dl_libs) {
		mutex_unlock(session->global_lock);
		npu_uerr("Failed to alloc dl_libs(%u)\n", session, session->hids, session->kernel_count);
		return -ENOMEM;
	}

	for (i = 0; i < kernel_bin_vector_cnt; i++) {
		if (i >= MAX_USER_KERNEL) {
			mutex_unlock(session->global_lock);
			npu_uerr("invalid number of user kernels(%d).\n", session, session->hids, i);
			return -EINVAL;
		}

		session->kernel_elf[i] = ncp_vaddr + body_offset + (dsp_kernel_bin_vector + i)->offset;
		session->kernel_elf_size[i] = (dsp_kernel_bin_vector + i)->size;

		ret = dsp_graph_add_kernel(device, session,
			(dsp_kernel_bin_vector + i)->kernel_string, session->dl_unique_id, i);
		if (ret) {
			mutex_unlock(session->global_lock);
			npu_uerr("Failed(%d) to add user kernel\n", session, session->hids,ret);
			return ret;
		}
	}

	ret = dsp_kernel_load(&device->kmgr, session->dl_libs, session->kernel_count);
	if (ret) {
		npu_uerr("Failed(%d) to user kernel load\n", session, session->hids, ret);
		return ret;
	}

	session->kernel_loaded = true;

	mutex_unlock(session->global_lock);

	return ret;
}
#endif

int second_parsing_ncp(struct npu_session *session,
	struct addr_info **IFM_av, struct addr_info **OFM_av,
	struct addr_info **IMB_av, struct addr_info **WGT_av)
{
	int ret = 0;

	ret = parsing_ncp_memory_vector(session, IFM_av,OFM_av, IMB_av, WGT_av);
	if (ret) {
		npu_uerr("fail in parsing_ncp_memory_vector(%d)\n", session, session->hids, ret);
		goto p_err;
	}

#if IS_ENABLED(CONFIG_DSP_USE_VS4L)
	if (session->hids == NPU_HWDEV_ID_DSP) {
		ret = parsing_ncp_kernel_vector(session);
		if (ret) {
			npu_uerr("fail in parsing_ncp_kernel_vector(%d)\n", session, session->hids, ret);
			goto p_err;
		}
	}
#endif

p_err:
	return ret;
}

int __ion_alloc_IOFM(struct npu_session *session, struct addr_info **IFM_av, struct addr_info **OFM_av)
{
	int ret = 0;
	u32 i = 0, j = 0, k = 0;
	struct npu_memory_buffer *IOFM_mem_buf;
	struct address_vector *vaddr;

	u32 IFM_cnt = session->IFM_cnt;
	u32 IOFM_cnt = session->IOFM_cnt;

	IOFM_mem_buf = kzalloc(sizeof(struct npu_memory_buffer), GFP_KERNEL);
	if (unlikely(!IOFM_mem_buf)) {
		npu_uerr("failed in (ENOMEM)\n", session, session->hids);
		ret = -ENOMEM;
		return ret;
	}
	npu_udbg("(0x%pK)\n", session, session->hids, IOFM_mem_buf);

	IOFM_mem_buf->size = NPU_FRAME_MAX_CONTAINERLIST * NPU_FRAME_MAX_BUFFER * IOFM_cnt * sizeof(struct address_vector);
	strcpy(IOFM_mem_buf->name, "IOFM");
	ret = npu_memory_alloc(session->memory, IOFM_mem_buf, npu_get_configs(NPU_PBHA_HINT_00));
	if (unlikely(ret)) {
		npu_uerr("npu_memory_alloc is fail(%d).\n", session, session->hids, ret);
		goto p_err;
	}
	npu_udbg(" (IOFM_mem_buf + %d)->vaddr(0x%pK), daddr(0x%pad), size(%zu)\n",
		session, session->hids, 0, (IOFM_mem_buf + 0)->vaddr, &(IOFM_mem_buf + 0)->daddr, (IOFM_mem_buf + 0)->size);

	vaddr = IOFM_mem_buf->vaddr;
	for (i = 0; i < IOFM_cnt; i++) {
		for (k = 0; k < NPU_FRAME_MAX_CONTAINERLIST; k++) {
			for (j = 0; j < NPU_FRAME_MAX_BUFFER; j++) {
				if (i < IFM_cnt) {
					(vaddr + (k*NPU_FRAME_MAX_BUFFER + j)*IOFM_cnt + i)->index = (*IFM_av + i)->av_index;
					(vaddr + (k*NPU_FRAME_MAX_BUFFER + j)*IOFM_cnt + i)->size = (*IFM_av + i)->size;
				} else {
					(vaddr + (k*NPU_FRAME_MAX_BUFFER + j)*IOFM_cnt + i)->index = (*OFM_av + (i - IFM_cnt))->av_index;
					(vaddr + (k*NPU_FRAME_MAX_BUFFER + j)*IOFM_cnt + i)->size = (*OFM_av + (i - IFM_cnt))->size;
				}
			}
		}
	}
	ret = add_ion_mem(session, IOFM_mem_buf, MEMORY_TYPE_IN_FMAP);
	return ret;
p_err:
	npu_memory_free(session->memory, IOFM_mem_buf);
	if (IOFM_mem_buf)
		kfree(IOFM_mem_buf);
	return ret;
}

#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
int prepare_IMB_info(struct npu_session *session, struct addr_info *IMB_av)
{
	int ret = 0;
	u32 i = 0;
	u32 IMB_cnt = session->IMB_cnt;
	struct npu_memory_buffer *IMB_mem_buf = NULL;

	IMB_mem_buf = kcalloc(1, sizeof(struct npu_memory_buffer), GFP_KERNEL);
	if (unlikely(!IMB_mem_buf)) {
		npu_uerr("IMB: IMB_mem_buf alloc is fail\n", session, session->hids);
		ret = -ENOMEM;
		goto p_err;
	}
	strcpy(IMB_mem_buf->name, "IMB");

	session->IMB_info = IMB_av;

	for (i = 0; i < IMB_cnt; i++) {
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
		IMB_mem_buf->ncp_max_size = max(IMB_mem_buf->ncp_max_size, ALIGN((IMB_av + i)->size, NPU_IMB_ALIGN));
#endif
		IMB_mem_buf->size += ALIGN((IMB_av + i)->size, NPU_IMB_ALIGN);
	}
	session->IMB_size = IMB_mem_buf->size;
	session->IMB_mem_buf = IMB_mem_buf;

#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	if (session->IMB_size > (NPU_IMB_CHUNK_SIZE * NPU_IMB_CHUNK_MAX_NUM)) {
		npu_uerr("IMB_size(%zu) is larger than %u\n",
			session, session->hids, session->IMB_size, NPU_IMB_CHUNK_SIZE * NPU_IMB_CHUNK_MAX_NUM);
		ret = -ENOMEM;
		goto p_err;
	}
#endif

	return ret;

p_err:
	if (likely(IMB_mem_buf))
		kfree(IMB_mem_buf);
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
int __ion_alloc_IMB(struct npu_session *session, struct addr_info **IMB_av, struct npu_memory_buffer *IMB_mem_buf)
{
	int ret = 0;
	u32 i = 0;
	u32 address_vector_offset;
	struct address_vector *av;
	char *ncp_vaddr;
	u32 IMB_cnt = session->IMB_cnt;
	u32 addr_offset = 0;
	struct npu_system *system = get_session_system(session);
	u32 req_chunk_cnt = __calc_imb_req_chunk(session, IMB_mem_buf, false);

	ncp_vaddr = (char *)session->ncp_hdr_buf->vaddr;
	address_vector_offset = session->address_vector_offset;

	av = (struct address_vector *)(ncp_vaddr + address_vector_offset);

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	if (session->IMB_size >= NPU_IMB_ASYNC_THRESHOLD) {
		npu_udbg("Async IMB allocation starts\n", session, session->hids);
		ret = npu_imb_alloc_async(session, IMB_av, IMB_mem_buf);
		if (unlikely(ret)) {
			session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_ERROR;
			goto p_err;
		} else {
			 npu_udbg("Async IMB allocation ends\n", session, session->hids);
			 return ret;
		}
	} else {
		if (session->imb_async_wq) {
			destroy_workqueue(session->imb_async_wq);
			session->imb_async_wq = NULL;
		}
	}
#endif
	npu_udbg("Sync IMB allocation starts\n", session, session->hids);
	if (session->hids == NPU_HWDEV_ID_NPU) {
		mutex_lock(&system->imb_alloc_data.imb_alloc_lock);
		ret = __alloc_imb_chunk(IMB_mem_buf, system,
					&system->imb_alloc_data, req_chunk_cnt);
		if (unlikely(ret)) {
			npu_uerr("IMB: __alloc_imb_chunk is fail(%d)\n", session, session->hids, ret);
			mutex_unlock(&system->imb_alloc_data.imb_alloc_lock);
			goto p_err;
		}
		/* in allocation case we're not interested in FW size acceptance */
		__fw_sync_CMD_IMB_SIZE(session, system->imb_alloc_data.alloc_chunk_cnt);
		mutex_unlock(&system->imb_alloc_data.imb_alloc_lock);
	} else {
		ret = npu_memory_alloc(session->memory, IMB_mem_buf, npu_get_configs(NPU_PBHA_HINT_00));
		if (unlikely(ret)) {
			npu_uerr("IMB: npu_memory_alloc is fail(%d).\n", session, session->hids, ret);
			goto p_err;
		}
	}

	for (i = 0; i < IMB_cnt; i++) {
		(av + (*IMB_av + i)->av_index)->m_addr = IMB_mem_buf->daddr + addr_offset;
		(session->IMB_info + i)->daddr = IMB_mem_buf->daddr + addr_offset;
		(session->IMB_info + i)->vaddr = ((void *)((char *)(IMB_mem_buf->vaddr)) + addr_offset);
		(session->IMB_info + i)->size = (*IMB_av + i)->size;
		addr_offset += ALIGN((*IMB_av + i)->size, (u32)NPU_IMB_ALIGN);
		npu_udbg("IMB: (IMB_mem_buf + %d)->vaddr(%pad), daddr(%pad), size(%zu)\n",
			 session, session->hids, i, (session->IMB_info + i)->vaddr, &(session->IMB_info + i)->daddr,
			 (session->IMB_info + i)->size);
	}

	npu_session_ion_sync_for_device(IMB_mem_buf, DMA_TO_DEVICE);
	return ret;

p_err:
#if !IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	npu_memory_free(session->memory, IMB_mem_buf);
#endif
#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	if (session->imb_async_wq) {
		destroy_workqueue(session->imb_async_wq);
		session->imb_async_wq = NULL;
	}
#endif

	kfree(IMB_mem_buf);
	session->IMB_mem_buf = NULL;

	return ret;
}
#endif

int __config_session_info(struct npu_session *session)
{
	int ret = 0;

	struct addr_info *IFM_av = NULL;
	struct addr_info *OFM_av = NULL;
	struct addr_info *IMB_av = NULL;
	struct addr_info *WGT_av = NULL;

	ret = __pilot_parsing_ncp(session, &session->IFM_cnt, &session->OFM_cnt, &session->IMB_cnt, &session->WGT_cnt);
	if (ret) {
		npu_uerr("failed in __pilot_parsing_ncp(%d)\n", session, session->hids, ret);
		goto p_err;
	}

	IFM_av = kcalloc(session->IFM_cnt, sizeof(struct addr_info), GFP_KERNEL);
	OFM_av = kcalloc(session->OFM_cnt, sizeof(struct addr_info), GFP_KERNEL);
	IMB_av = kcalloc(session->IMB_cnt, sizeof(struct addr_info), GFP_KERNEL);
	WGT_av = kcalloc(session->WGT_cnt, sizeof(struct addr_info), GFP_KERNEL);

	session->WGT_info = WGT_av;
	session->ss_state |= BIT(NPU_SESSION_STATE_WGT_KALLOC);

	if (unlikely(!IFM_av || !OFM_av || !IMB_av || !WGT_av)) {
		npu_uerr("failed in (ENOMEM)\n", session, session->hids);
		ret = -ENOMEM;
		goto p_err;
	}

	ret = second_parsing_ncp(session, &IFM_av, &OFM_av, &IMB_av, &WGT_av);
	if (unlikely(ret)) {
		npu_uerr("fail(%d) in second_parsing_ncp\n", session, session->hids, ret);
		goto p_err;
	}

	session->IFM_info.addr_info = IFM_av;
	session->OFM_info.addr_info = OFM_av;
	session->ss_state |= BIT(NPU_SESSION_STATE_IOFM_KALLOC);

	ret = __ion_alloc_IOFM(session, &IFM_av, &OFM_av);
	if (unlikely(ret)) {
		npu_uerr("fail(%d) in __ion_alloc_IOFM\n", session, session->hids, ret);
		goto p_err;
	}
	session->ss_state |= BIT(NPU_SESSION_STATE_IOFM_ION_ALLOC);
#if IS_ENABLED(CONFIG_NPU_USE_IMB_ALLOCATOR)
	if (session->IMB_cnt != 0) {
		ret = prepare_IMB_info(session, IMB_av);
		if (unlikely(ret)) {
			npu_uerr("IMB: fail(%d) in prepare_IMB_info\n", session, session->hids, ret);
			goto p_err;
		}
		npu_udbg("IMB: buffer summary size = 0x%zX\n", session, session->hids, session->IMB_size);
		npu_udbg("IMB: NCP max size = 0x%zX\n", session, session->hids, session->IMB_mem_buf->ncp_max_size);
		npu_udbg("IMB: granularity size = 0x%X\n", session, session->hids, NPU_IMB_GRANULARITY);

		if (session->IMB_mem_buf->size != 0) {
			ret = __ion_alloc_IMB(session, &IMB_av, session->IMB_mem_buf);
			if (unlikely(ret)) {
				npu_uerr("IMB: fail(%d) in ion alloc IMB\n", session, session->hids, ret);
				goto p_err;
			}
		}
		session->ss_state |= BIT(NPU_SESSION_STATE_IMB_ION_ALLOC);
	}
#endif
	npu_session_ion_sync_for_device(session->ncp_hdr_buf, DMA_TO_DEVICE);

	return ret;
p_err:
	if (likely(IFM_av)) {
		kfree(IFM_av);
		session->IFM_info.addr_info = NULL;
	}

	if (likely(OFM_av)) {
		kfree(OFM_av);
		session->OFM_info.addr_info = NULL;
	}

	if (likely(WGT_av)) {
		kfree(WGT_av);
		session->WGT_info = NULL;
	}

	if (likely(IMB_av)) {
		kfree(IMB_av);
		session->IMB_info = NULL;
	}

	return ret;
}

int npu_session_s_graph(struct npu_session *session, struct vs4l_graph *info)
{
	int ret = 0;
	struct npu_vertex *vertex;
	struct npu_device *device;

	vertex = session->vctx.vertex;
	device = container_of(vertex, struct npu_device, vertex);

	if (unlikely(!session)) {
		npu_err("Failed to get npu_session\n");
		return -EINVAL;
	}

	if (unlikely(!info)) {
		npu_uerr("Failed to get vs4l_graph\n", session, session->hids);
		return -EINVAL;
	}

	ret = __get_session_info(session, info);
	if (unlikely(ret)) {
		npu_uerr("invalid(%d) in __get_session_info\n", session, session->hids, ret);
		goto p_err;
	}

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_SKIP;
#endif

	ret = __config_session_info(session);
	if (unlikely(ret)) {
		npu_uerr("invalid in __config_session_info\n", session, session->hids);
		goto p_err;
	}

	if (device->sched->mode == NPU_PERF_MODE_NPU_BOOST_PRUNE && session->hids == NPU_HWDEV_ID_NPU) {
		ret = __shared_ion_map(session, info);
		if (unlikely(ret)) {
			npu_uerr("__shared_ion_map is fail(%d)\n", session, session->hids, ret);
			goto p_err;
		}
		info->reserved1 = session->IOFM_mem_buf->daddr;
	}

	return ret;

p_err:
	npu_uerr("Clean-up buffers for graph\n", session, session->hids);
	__release_graph_ion(session);

	return ret;
}

int npu_session_start(struct npu_queue *queue)
{
	int ret = 0;
	struct npu_vertex_ctx *vctx;
	struct npu_session *session;

	if (unlikely(!queue)) {
		npu_err("Failed to get queue\n");
		return -EINVAL;
	}

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	if (unlikely(!vctx)) {
		npu_uerr("Failed to get npu_vertex_ctx\n", session, session->hids);
		return -EINVAL;
	}

	if (unlikely(!session)) {
		npu_err("Failed to get npu_session\n");
		return -EINVAL;
	}

	session->ss_state |= BIT(NPU_SESSION_STATE_START);
	return ret;
}

int npu_session_streamoff(struct npu_queue *queue)
{

	if (unlikely(!queue)) {
		npu_err("Failed to get queue\n");
		return -EINVAL;
	}

	return 0;
}

int npu_session_NW_CMD_STREAMOFF(struct npu_session *session)
{
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	nw_cmd_e nw_cmd = NPU_NW_CMD_STREAMOFF;

	/* check npu_device emergency error */
	vctx = &session->vctx;
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

	npu_udbg("sending STREAM OFF command.\n", session, session->hids);
	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

	if (npu_device_is_emergency_err(device)) {
		npu_udbg("NPU DEVICE IS EMERGENCY ERROR !\n", session, session->hids);
		npu_udbg("sending CLEAR_CB command.\n", session, session->hids);
		npu_session_put_nw_req(session, NPU_NW_CMD_CLEAR_CB);
		/* Clear CB has no notify function */
	}

	return 0;
}

int npu_session_stop(struct npu_queue *queue)
{
	struct npu_session *session;
	struct npu_vertex_ctx *vctx;

	if (unlikely(!queue)) {
		npu_err("Failed to get queue\n");
		return -EINVAL;
	}

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	session->ss_state |= BIT(NPU_SESSION_STATE_STOP);

	return 0;
}

int npu_session_format(struct npu_queue *queue, struct vs4l_format_bundle *fbundle)
{
	int ret = 0;
	struct npu_session *session;
	struct npu_vertex_ctx *vctx;
#if IS_ENABLED(CONFIG_SOC_S5E8855)
	u32 i = 0, j = 0;
	struct vs4l_format_list *flist;

	struct address_vector *av = NULL;
	struct memory_vector *mv = NULL;

	char *ncp_vaddr;
	struct vs4l_format *formats;
	struct addr_info *FM_av;
	u32 address_vector_offset;
	u32 address_vector_cnt;

	u32 memory_vector_offset;
	u32 memory_vector_cnt;

	u32 cal_size;
	u32 bpp;
	u32 channels;
	u32 width;
	u32 height;

	u32 FM_cnt;

	if (unlikely(!queue)) {
		npu_err("Failed to get npu_queue\n");
		return -EINVAL;
	}

	if (unlikely(!fbundle)) {
		npu_err("Failed to get vs4l_format_bundle\n");
		return -EINVAL;
	}
#endif
	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

#if IS_ENABLED(CONFIG_SOC_S5E8855)
	if (unlikely(!vctx)) {
		npu_uerr("Failed to get npu_vertex_ctx\n", session, session->hids);
		return -EINVAL;
	}

	ncp_vaddr = (char *)session->ncp_hdr_buf->vaddr;

	address_vector_offset = session->address_vector_offset;
	address_vector_cnt = session->address_vector_cnt;

	memory_vector_offset = session->memory_vector_offset;
	memory_vector_cnt = session->memory_vector_cnt;

	if (memory_vector_cnt && memory_vector_offset)
		mv = (struct memory_vector *)(ncp_vaddr + memory_vector_offset);
	if (address_vector_cnt && address_vector_offset)
		av = (struct address_vector *)(ncp_vaddr + address_vector_offset);

	for (i = 0; i < 2; i++) {
		flist = fbundle->m[i].flist;
		formats = flist->formats;

		if (flist->count <= 0)
			continue;

		if (flist->count > NPU_FRAME_MAX_CONTAINERLIST) {
			npu_uerr("flist->count(%d) cannot be greater to NPU_FRAME_MAX_CONTAINERLIST(%d)\n", session, session->hids,
				flist->count, NPU_FRAME_MAX_CONTAINERLIST);
			ret = -EINVAL;
			goto p_err;
		}

		if (flist->direction == VS4L_DIRECTION_IN) {
			FM_av = session->IFM_info.addr_info;
			FM_cnt = session->IFM_cnt;
		} else {
			FM_av = session->OFM_info.addr_info;
			FM_cnt = session->OFM_cnt;
		}

		npu_udbg("flist %d, FM_cnt %d\n", session, session->hids, flist->count, FM_cnt);
		if (FM_cnt > flist->count || FM_cnt + FM_SHARED_NUM < flist->count) {
			npu_uerr("FM_cnt(%d) is not same as flist_cnt(%d)\n", session, session->hids, FM_cnt, flist->count);
			ret = -EINVAL;
			goto p_err;
		}

		for (j = 0; j < FM_cnt; j++) {
			if (mv) {
				(mv + (FM_av + j)->av_index)->wstride = (formats + j)->stride;
				(mv + (FM_av + j)->av_index)->cstride = (formats + j)->cstride;
			}

			bpp = (formats + j)->pixel_format;
			channels = (formats + j)->channels;
			width = (formats + j)->width;
			height = (formats + j)->height;
#if (NCP_VERSION <= 27)
			cal_size = (bpp / 8) * channels * width * height;
#else
			cal_size = channels * width * height;
#endif

			if (flist->direction == VS4L_DIRECTION_IN)
				queue->insize[j] = cal_size;
			else
				queue->otsize[j] = cal_size;

			npu_udbg("dir(%d), av_index(%d)\n", session, session->hids, flist->direction, (FM_av + j)->av_index);
			npu_udbg("width(%u), height(%u), stride(%u)\n", session, session->hids, width, height, (formats + j)->stride);
			npu_udbg("cstride(%u), channels(%u), pixel_format(%u)\n", session, session->hids, (formats + j)->cstride, channels, bpp);
			npu_udbg("dir(%d), av_size(%zu), cal_size(%u)\n", session, session->hids, flist->direction, (FM_av + j)->size, cal_size);
#if !IS_ENABLED(SYSMMU_FAULT_TEST)
			if ((FM_av + j)->size > cal_size) {
				if ((FM_av + j)->size % cal_size) {
					npu_udbg("in_size(%zu), cal_size(%u) invalid\n", session, session->hids, (FM_av + j)->size, cal_size);
					ret = NPU_ERR_DRIVER(NPU_ERR_SIZE_NOT_MATCH);
					goto p_err;
				}
			}
#endif
		}
	}
#endif
	session->ss_state |= BIT(NPU_SESSION_STATE_FORMAT_IN);
	session->ss_state |= BIT(NPU_SESSION_STATE_FORMAT_OT);

#if IS_ENABLED(CONFIG_SOC_S5E8855)
p_err:
#endif
	return ret;
}

int npu_session_NW_CMD_POWER_NOTIFY(struct npu_session *session, bool on)
{
	int ret = 0;
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	struct npu_hw_device *hdev;
	nw_cmd_e nw_cmd = NPU_NW_CMD_POWER_CTL;

	hdev = npu_get_hdev_by_id(session->hids);
	if (!hdev) {
		npu_uerr("no hwdevice found\n", session, session->hids);
		return -EINVAL;
	}

	mutex_lock(session->global_lock);

	vctx = &(session->vctx);
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	if (session->imb_async_wq && !on) {
		cancel_delayed_work_sync(&session->imb_async_work);
	}
#endif

	if (atomic_read(&hdev->init_cnt.refcount) > 1) {
		mutex_unlock(session->global_lock);
		return ret;
	}
	dsp_dhcp_update_pwr_status(device, session->hids, on);

	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

	mutex_unlock(session->global_lock);

	return ret;
}

#ifndef CONFIG_NPU_KUNIT_TEST
int npu_session_restore_cnt(struct npu_session *session)
{
	int ret = 0;
	struct npu_sessionmgr *sessionmgr;
	struct npu_hw_device *hdev_npu = npu_get_hdev_by_id(NPU_HWDEV_ID_NPU);

	mutex_lock(session->global_lock);

	sessionmgr = session->cookie;
	atomic_xchg(&hdev_npu->init_cnt.refcount, sessionmgr->npu_hw_cnt_saved);
	atomic_xchg(&hdev_npu->init_cnt.refcount, sessionmgr->npu_hw_cnt_saved);

	mutex_unlock(session->global_lock);

	return ret;
}

int npu_session_NW_CMD_RESUME(struct npu_session *session)
{
	int ret = 0;
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	nw_cmd_e nw_cmd = NPU_NW_CMD_POWER_CTL;

	if (!session) {
		npu_err("invalid session\n");
		return -EINVAL;
	}

	mutex_lock(session->global_lock);

	vctx = &(session->vctx);
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

	dsp_dhcp_update_pwr_status(device, session->hids, true);
	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

	mutex_unlock(session->global_lock);
	return ret;
}

int npu_session_NW_CMD_SUSPEND(struct npu_session *session)
{
	int ret = 0;
	int cnt = 0;

	struct npu_sessionmgr *sessionmgr;
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	struct npu_hw_device *hdev_npu = npu_get_hdev_by_id(NPU_HWDEV_ID_NPU);
	struct npu_hw_device *hdev_dsp = npu_get_hdev_by_id(NPU_HWDEV_ID_DSP);
	nw_cmd_e nw_cmd = NPU_NW_CMD_POWER_CTL;

	if (!session) {
		npu_err("invalid session\n");
		return -EINVAL;
	}

	mutex_lock(session->global_lock);

	sessionmgr = session->cookie;
	vctx = &(session->vctx);
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);

	sessionmgr->npu_hw_cnt_saved = 0;
	sessionmgr->npu_hw_cnt_saved = atomic_xchg(&hdev_npu->init_cnt.refcount, sessionmgr->npu_hw_cnt_saved);
	sessionmgr->dsp_hw_cnt_saved = 0;
	sessionmgr->dsp_hw_cnt_saved = atomic_xchg(&hdev_dsp->init_cnt.refcount, sessionmgr->dsp_hw_cnt_saved);

	dsp_dhcp_update_pwr_status(device, session->hids, false);
	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

	/* Waiting for FW to update the flag */
#define BOOT_TYPE_WARM_BOOT (0xb0080c0d)
	ret = -EFAULT;
	do {
		if (device->system.mbox_hdr->boottype == BOOT_TYPE_WARM_BOOT) {
			ret = 0;
			break;
		}
		msleep(1);
	} while (cnt++ < 18); /* Maximum 100 retries */
	if (!ret) {
		sessionmgr->npu_hw_cnt_saved = 0;
		sessionmgr->npu_hw_cnt_saved = atomic_xchg(&hdev_npu->init_cnt.refcount, sessionmgr->npu_hw_cnt_saved);
		sessionmgr->dsp_hw_cnt_saved = 0;
		sessionmgr->dsp_hw_cnt_saved = atomic_xchg(&hdev_dsp->init_cnt.refcount, sessionmgr->dsp_hw_cnt_saved);
	}

	mutex_unlock(session->global_lock);

	return ret;
}
#endif

static struct npu_memory_buffer *alloc_npu_load_payload(struct npu_session *session)
{
	struct npu_memory_buffer *buffer;
	int ret;

	buffer = kzalloc(sizeof(struct npu_memory_buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	/* 2 payloads.
	 * - User NCP
	 * - Kernel copy and updated NCP header.
	 */
	buffer->size = sizeof(struct cmd_load_payload) * 2;

	ret = npu_memory_alloc(session->memory, buffer, npu_get_configs(NPU_PBHA_HINT_00));
	if (ret) {
		npu_uerr("failed to allocate payload buffer(%d)\n", session, session->hids, ret);
		goto err_free_buffer;
	}

	return buffer;
err_free_buffer:
	kfree(buffer);
	return ERR_PTR(ret);
}

int npu_session_NW_CMD_LOAD(struct npu_session *session)
{
	int ret = 0;
	nw_cmd_e nw_cmd = NPU_NW_CMD_LOAD;

	if (unlikely(!session)) {
		npu_err("invalid session\n");
		ret = -EINVAL;
		return ret;
	}


	session->ncp_payload = alloc_npu_load_payload(session);
	if (IS_ERR_OR_NULL(session->ncp_payload)) {
		ret = PTR_ERR(session->ncp_payload);
		session->ncp_payload = NULL;
		return ret;
	}

	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);
	return ret;
}

#ifndef CONFIG_NPU_KUNIT_TEST
void npu_session_restart(void)
{
	npu_buffer_q_restart();
}
#endif

int npu_session_put_cancel_req(
	struct npu_session *session, struct npu_queue *queue,	struct npu_queue_list *incl, struct npu_queue_list *otcl,
	frame_cmd_e frame_cmd)
{
	int ret = 0;
	struct npu_sessionmgr *sess_mgr;
	struct npu_frame frame = {
		.uid = session->uid,
		.frame_id = incl->id,
		.result_code = 0,
		.session = session,
		.cmd = frame_cmd,
		.src_queue = queue,
		.input = incl,
		.output = otcl,
		.IFM_info = NULL,
		.OFM_info = NULL,
	};

	sess_mgr = session->cookie;
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	mutex_lock(&sess_mgr->cancel_lock);
#endif
	ret = proto_drv_handler_frame_requested(&frame);
	if (unlikely(ret)) {
			npu_uerr("fail(%d) in proto_drv_handler_frame_requested\n", session, session->hids, ret);
			ret = -EFAULT;
	}
#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	mutex_unlock(&sess_mgr->cancel_lock);
#endif

	return ret;
}

int npu_session_queue_cancel(struct npu_queue *queue, struct npu_queue_list *incl, struct npu_queue_list *otcl)
{
	int ret = 0;
	struct npu_session *session;
	struct npu_vertex_ctx *vctx;
	frame_cmd_e frame_cmd = NPU_FRAME_CMD_Q_CANCEL;

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	mutex_lock(session->global_lock);

	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	ret = npu_session_put_cancel_req(session, queue, incl, otcl, frame_cmd);
	if (unlikely(ret))
		goto p_err;
	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

p_err:
	mutex_unlock(session->global_lock);
	return ret;
}

int npu_session_queue(struct npu_queue *queue, struct npu_queue_list *incl, struct npu_queue_list *otcl)
{
	int ret = 0, i = 0;
	struct npu_session *session;
	struct npu_vertex_ctx *vctx;
	struct mbox_process_dat *mbox_process_dat;
	struct av_info *IFM_info;
	struct av_info *OFM_info;

	char *ncp_vaddr;
	char *ncp_vaddr_copy;

	struct address_vector *av;
	struct address_vector *av_copy;
	s64 now = 0;

	frame_cmd_e frame_cmd = NPU_FRAME_CMD_Q;

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	now = npu_get_time_us();
	ncp_vaddr = (char *)session->ncp_mem_buf->vaddr + session->ncp_mem_buf_offset;
	ncp_vaddr_copy = (char *)session->ncp_hdr_buf->vaddr;

#if IS_ENABLED(CONFIG_NPU_GOVERNOR)
	/* 40% weightage to old value; and 60% to new computation */
	session->arrival_interval = ((4 * session->arrival_interval) + (6 * (now - session->last_qbuf_arrival)))/10;
	if (!session->tpf_requested) {
		session->tpf = (session->arrival_interval * 70)/100;
	}
#endif

	session->last_qbuf_arrival = now;
	session->qbuf_IOFM_idx = incl->index;

	IFM_info = &session->IFM_info;
	OFM_info = &session->OFM_info;
	IFM_info->IOFM_buf_idx = incl->index;
	OFM_info->IOFM_buf_idx = otcl->index;

	npu_udbg("uid %d, in id %d, idx %d, out id %d, idx %d\n", session, session->hids,
			session->uid, incl->id, incl->index, otcl->id, otcl->index);

	mbox_process_dat = &session->mbox_process_dat;
	mbox_process_dat->address_vector_cnt = session->IOFM_cnt;
	mbox_process_dat->address_vector_start_daddr = (session->IOFM_mem_buf->daddr)
				+ ((session->qbuf_IOFM_idx * NPU_FRAME_MAX_BUFFER) * session->IOFM_cnt) * (sizeof(struct address_vector));

	av = (struct address_vector *)(ncp_vaddr + session->address_vector_offset);
	av_copy = (struct address_vector *)(ncp_vaddr_copy + session->address_vector_offset);

	if (session->weight_mem_buf != NULL &&
				session->weight_mem_buf->reserved == VS4L_MEMORY_DMABUF_DYNAMIC) {
		ret = npu_memory_map(session->memory, session->weight_mem_buf, npu_get_configs(NPU_PBHA_HINT_11), session->lazy_unmap_disable, FALSE);
		if (unlikely(ret)) {
			npu_err("npu_memory_map is fail(%d).\n", ret);
			goto p_err;
		}

		for (i = 0; i < session->WGT_cnt; i++) {
			(av + session->wgt_av_index)->m_addr = session->weight_mem_buf->daddr;
			(av_copy + session->wgt_av_index)->m_addr = session->weight_mem_buf->daddr;
		}
	}

	ret = npu_session_put_frame_req(session, queue, incl, otcl, frame_cmd, IFM_info, OFM_info);
	if (unlikely(ret))
		goto p_err;

	IFM_info->state = SS_BUF_STATE_QUEUE;
	OFM_info->state = SS_BUF_STATE_QUEUE;

	return 0;
p_err:
	return ret;
}

int npu_session_deque(struct npu_queue *queue)
{
	int ret = 0;
	struct npu_session *session;
	struct npu_vertex_ctx *vctx;
	struct av_info *FM_info;

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);


	if (session->weight_mem_buf != NULL &&
				session->weight_mem_buf->reserved == VS4L_MEMORY_DMABUF_DYNAMIC)
		npu_memory_unmap(session->memory, session->weight_mem_buf);

	FM_info = &session->IFM_info;
	FM_info->state = SS_BUF_STATE_DEQUEUE;

	FM_info = &session->OFM_info;
	FM_info->state = SS_BUF_STATE_DEQUEUE;

	npu_udbg("success\n", session, session->hids);
	return ret;
}

int npu_session_update_iofm(struct npu_queue *queue, struct npu_queue_list *clist)
{
	int ret = 0;

	struct npu_vertex_ctx *vctx;
	struct npu_session *session;
	struct nq_buffer *buffer;
	struct av_info *FM_info;
	struct nq_container *container;
	struct address_vector *IOFM_av;
	struct npu_memory_buffer *IOFM_mem_buf;

	u32 FM_cnt, count;
	u32 i, j;
	u32 buf_cnt;
	u32 clindex;

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	clindex = clist->index;
	IOFM_mem_buf = session->IOFM_mem_buf;
	IOFM_av = (struct address_vector *)(IOFM_mem_buf->vaddr);

	count = clist->count;
	for (i = 0; i < count ; i++) {
		container = &clist->containers[i];
		buf_cnt = clist->containers[i].count;

		for (j = 0; j < buf_cnt; j++) {
			npu_udbg("i %d/%d, j %d/%d\n", session, session->hids, i, count, j, buf_cnt);
			buffer = &container->buffers[j];
			if (clist->direction == VS4L_DIRECTION_IN) {
				FM_info = &session->IFM_info;
				FM_cnt = session->IFM_cnt;
				(IOFM_av + ((clindex * NPU_FRAME_MAX_BUFFER + j) * session->IOFM_cnt) + i)->m_addr = buffer->daddr + buffer->roi.offset;
				(IOFM_av + ((clindex * NPU_FRAME_MAX_BUFFER + j) * session->IOFM_cnt) + i)->size = buffer->dma_buf->size;
			} else {
				FM_info = &session->OFM_info;
				FM_cnt = session->OFM_cnt;
				(IOFM_av + ((clindex * NPU_FRAME_MAX_BUFFER + j) * session->IOFM_cnt) + i + session->IFM_cnt)->m_addr = buffer->daddr + buffer->roi.offset;
				(IOFM_av + ((clindex * NPU_FRAME_MAX_BUFFER + j) * session->IOFM_cnt) + i + session->IFM_cnt)->size = buffer->dma_buf->size;
			}

			npu_udbg("clist %d, FM_cnt %d\n", session, session->hids, count, FM_cnt);
			if (FM_cnt > count || FM_cnt + FM_SHARED_NUM < count) {
				npu_uerr("dir(%d), FM_cnt(%u) != count(%u)\n", session, session->hids, clist->direction, FM_cnt, count);
				ret = -EINVAL;
				goto p_err;
			}
		}
		FM_info->state = SS_BUF_STATE_PREPARE;
	}

	return ret;

p_err:
	return ret;
}

int npu_session_prepare(struct npu_queue *queue, struct npu_queue_list *clist)
{
	int ret = 0;
	struct npu_vertex_ctx *vctx;
	struct npu_session *session;

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	ret = npu_session_update_iofm(queue, clist);
	if (ret) {
		npu_uerr("fail(%d) in npu_session_update_iofm\n", session, session->hids, ret);
		goto p_err;
	}

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	if (session->IMB_size >= NPU_IMB_ASYNC_THRESHOLD) {
		npu_udbg("wait async allocation result:0x%x\n",
				session, session->hids, session->imb_async_result_code);
		wait_event(session->imb_wq,
			(session->imb_async_result_code == NPU_IMB_ASYNC_RESULT_DONE) ||
			(session->imb_async_result_code == NPU_IMB_ASYNC_RESULT_ERROR));

		if (session->imb_async_wq) {
			destroy_workqueue(session->imb_async_wq);
			session->imb_async_wq = NULL;
		}

		if (session->imb_async_result_code == NPU_IMB_ASYNC_RESULT_ERROR) {
			npu_uerr("IMB: fail in ion alloc IMB async\n", session, session->hids);
			ret = -EINVAL;
			goto p_err;
		}

		npu_udbg("wait async allocation done result:0x%x\n",
				session, session->hids, session->imb_async_result_code);
	}
#endif
p_err:
	return ret;
}

int npu_session_unprepare(struct npu_queue *queue, struct npu_queue_list *clist)
{
	int ret = 0;
	struct npu_vertex_ctx *vctx;
	struct npu_session *session;

	struct av_info *FM_info;

	vctx = container_of(queue, struct npu_vertex_ctx, queue);
	session = container_of(vctx, struct npu_session, vctx);

	if (clist->direction == VS4L_DIRECTION_IN)
		FM_info = &session->IFM_info;
	else
		FM_info = &session->OFM_info;

	FM_info->state = SS_BUF_STATE_UNPREPARE;

	return ret;
}

#if !IS_ENABLED(CONFIG_DEBUG_FS)
static npu_s_param_ret fw_test_s_param_handler(struct npu_session *sess, struct vs4l_param *param)
{
	/* No OP */
	return S_PARAM_NOMB;
}
#endif

/* S_PARAM handler list definition - Chain of responsibility pattern */
typedef npu_s_param_ret(*fn_s_param_handler)(struct npu_session *, struct vs4l_param *);
static fn_s_param_handler s_param_handlers[] = {
	NULL,	/* NULL termination is required to denote EOL */
	npu_scheduler_param_handler,
	npu_qos_param_handler,
	fw_test_s_param_handler,
#if !IS_ENABLED(CONFIG_NPU_BRINGUP_NOTDONE)
	npu_preference_param_handler,
#else
		NULL,
#endif
};

int npu_session_param(struct npu_session *session, struct vs4l_param_list *plist)
{
	npu_s_param_ret		ret = 0;
	struct vs4l_param	*param;
	u32			i, target;
	fn_s_param_handler	*p_fn;

	if (unlikely(!session)) {
		npu_err("Failed to get npu_session\n");
		return -EINVAL;
	}

	/* Search over each set param request over handlers */
	for (i = 0; i < plist->count; i++) {
		param = &plist->params[i];
		if (unlikely(!param)) {
			npu_uerr("Failed to get vs4l_param %d\n", session, session->hids, i);
			return -EINVAL;
		}

		target = param->target;
		npu_uinfo("Try set param [%u/%u] - target: [0x%x] - offset: [%d]\n",
			 session, session->hids, i+1, plist->count, target, param->offset);

		p_fn = s_param_handlers;
		if ((target >= NPU_S_PARAM_PERF_MODE) &&
				(target < NPU_S_PARAM_SCH_PARAM_END)) {
			p_fn += 1;
		} else if ((target >= NPU_S_PARAM_QOS_NPU) &&
				(target < NPU_S_PARAM_QOS_PARAM_END)) {
			p_fn += 2;
		} else if ((target >= NPU_S_PARAM_FW_UTC_LOAD) &&
				(target < NPU_S_PARAM_FW_PARAM_END)) {
			p_fn += 3;
		} else if (target == NPU_S_PARAM_PREF_MODE) {
			p_fn += 4;
		} else {
			/* Continue process others */
			npu_uwarn("No handler defined for target [0x%x]."
				"Skipping.\n", session, session->hids, param->target);
			continue;
		}

		ret = (*p_fn)(session, param);
		if (ret != S_PARAM_NOMB)
			npu_udbg("Handled by handler at [%pK](%d)\n",
						session, session->hids, *p_fn, ret);
	}

	if (ret == S_PARAM_ERROR)
		return ret;

	return 0;
}

int npu_session_nw_sched_param(struct npu_session *session, struct vs4l_sched_param *param)
{
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;

	int		retval = 0;
	u32		priority;
	u32		bound_id;
	u32		max_npu_core;

	vctx = &(session->vctx);
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);
	if (unlikely(!device)) {
		npu_uerr("Failed to get npu_device\n", session, session->hids);
		return -EINVAL;
	}

	priority = param->priority;
	bound_id = param->bound_id;
	max_npu_core = session->sched_param.max_npu_core;

	session->sched_param.deadline = param->deadline;
	/* Check priority range */
	if (priority <= NPU_PRIORITY_MAX_VAL)
		session->sched_param.priority = priority;
	else
		retval = -EINVAL;

	/* Check boundness to the core */
	if ((bound_id < max_npu_core) || (bound_id == NPU_BOUND_UNBOUND))
		session->sched_param.bound_id = bound_id;
	else
		retval = -EINVAL;

	/* Set the default value if anything is wrong */
	if (retval != 0) {
		session->sched_param.priority = NPU_PRIORITY_MID_PRIO;
		session->sched_param.bound_id = NPU_BOUND_UNBOUND;
		session->sched_param.deadline = NPU_MAX_DEADLINE;
	}

	npu_udbg("r[%d], p[%u], b[%u], c[%u]\n",
			session, session->hids, retval, priority, bound_id, max_npu_core);

	return retval;
}

int npu_session_register_undo_cb(struct npu_session *session, session_cb cb)
{
	session->undo_cb = cb;

	return 0;
}

int npu_session_execute_undo_cb(struct npu_session *session)
{

	if (session->undo_cb)
		session->undo_cb(session);

	return 0;
}

int npu_session_undo_open(struct npu_session *session)
{
	int ret = 0;

	if (session->ss_state < BIT(NPU_SESSION_STATE_OPEN)) {
		ret = -EINVAL;
		goto p_err;
	}

	if (session->ss_state <= BIT(NPU_SESSION_STATE_OPEN))
		goto session_free;

	npu_sessionmgr_unregID(session->cookie, session);

session_free:
	kfree(session);
	session = NULL;
p_err:
	return ret;
}

int npu_session_undo_s_graph(struct npu_session *session)
{
	int ret = 0;

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
	if (session->imb_async_wq) {
		destroy_workqueue(session->imb_async_wq);
		session->imb_async_wq = NULL;
	}
#endif

	if (!_undo_s_graph_each_state(session)) {
		npu_session_register_undo_cb(session, NULL);
		goto next_cb;
	}
	ret = -EINVAL;
	return ret;
next_cb:
	npu_session_execute_undo_cb(session);
	return ret;
}

int npu_session_undo_close(struct npu_session *session)
{
	int ret = 0;

	if (!_undo_close_each_state(session)) {
		npu_session_register_undo_cb(session, npu_session_undo_s_graph);
		goto next_cb;
	}
	ret = -EINVAL;
	return ret;
next_cb:
	npu_session_execute_undo_cb(session);
	return ret;
}

int npu_session_flush(struct npu_session *session)
{
	int ret = 0;

	if (!session) {
		npu_err("Failed to get npu_session\n");
		return -EINVAL;
	} else {
		struct vs4l_param	param;

		param.target = NPU_S_PARAM_PERF_MODE;
		param.offset = NPU_PERF_MODE_NORMAL;

#if IS_ENABLED(CONFIG_NPU_IMB_ASYNC_ALLOC)
		if (session->imb_async_wq) {
			cancel_delayed_work_sync(&session->imb_async_work);
			destroy_workqueue(session->imb_async_wq);
			session->imb_async_wq = NULL;
		}
		session->imb_async_result_code = NPU_IMB_ASYNC_RESULT_SKIP;
#endif

		ret = npu_scheduler_param_handler(session, &param);
		if (ret == S_PARAM_ERROR)
			npu_uerr("fail(%d) in npu_session_param\n", session, session->hids, ret);

		_undo_s_graph_each_state(session);
	}

	return 0;
}

int npu_session_profile_ready(struct npu_session *session, struct vs4l_profile_ready *pread)
{
	int ret = 0;
	struct npu_memory_buffer *profile_mem_buf = NULL;
	struct npu_memory_buffer *profile_mem_sub_buf = NULL;
	struct dhcp_table *dhcp_table;
	struct npu_vertex_ctx *vctx;
	struct npu_vertex *vertex;
	struct npu_device *device;
	int uid;

	vctx = &session->vctx;
	vertex = vctx->vertex;
	device = container_of(vertex, struct npu_device, vertex);
	dhcp_table = device->system.dhcp_table;
	uid = session->uid;

	profile_mem_sub_buf = kzalloc(sizeof(struct npu_memory_buffer), GFP_KERNEL);
	if (profile_mem_sub_buf == NULL) {
		npu_uerr("fail in npu_ion_map kzalloc\n", session, session->hids);
		ret = -ENOMEM;
		goto p_err;
	}

	profile_mem_sub_buf->size = sizeof(int) * 4194304;
	ret = npu_memory_alloc(session->memory, profile_mem_sub_buf, npu_get_configs(NPU_PBHA_HINT_00));
	if (unlikely(ret)) {
		npu_uerr("npu_memory_alloc is fail(%d).\n", session, session->hids, ret);
		goto p_err;
	}

	dhcp_table->PROF_MODE = pread->mode;
	dhcp_table->PROF_INFO[session->uid].PROFILE_MAX_ITER = pread->max_iter;
	dhcp_table->PROF_INFO[session->uid].PROFILE_SIZE = profile_mem_sub_buf->size;
	dhcp_table->PROF_INFO[session->uid].PROFILE_IOVA = profile_mem_sub_buf->daddr;

	if (pread->fd > 0) {
		if (pread->offset >= pread->size) {
			npu_uerr("please check profile offset & size\n", session, session->hids);
			ret = -EINVAL;
			goto p_err;
		}

		profile_mem_buf = kzalloc(sizeof(struct npu_memory_buffer), GFP_KERNEL);
		if (profile_mem_buf == NULL) {
			npu_uerr("fail in npu_ion_map kzalloc\n", session, session->hids);
			ret = -ENOMEM;
			goto p_err;
		}

		profile_mem_buf->m.fd = pread->fd;
		profile_mem_buf->size = pread->size;

		ret = npu_memory_map(session->memory, profile_mem_buf, npu_get_configs(NPU_PBHA_HINT_11),
										session->lazy_unmap_disable, TRUE);
		if (unlikely(ret)) {
			npu_uerr("npu_memory_map is fail(%d).\n", session, session->hids, ret);
			kfree(profile_mem_buf);
			goto p_err;
		}

		memset(profile_mem_buf->vaddr, 0, profile_mem_buf->size);

		dhcp_table->PROF_MAIN[session->uid].PROFILE_MAX_ITER = pread->max_iter;
		dhcp_table->PROF_MAIN[session->uid].PROFILE_SIZE = pread->size - pread->offset;
		dhcp_table->PROF_MAIN[session->uid].PROFILE_IOVA = profile_mem_buf->daddr + pread->offset;

		if (device->sched->mode == NPU_PERF_MODE_NPU_BOOST_PRUNE)
			uid = 0;
	}

	npu_profile_init(profile_mem_buf, profile_mem_sub_buf, pread, device->sched->mode, uid);
	session->ss_state |= BIT(NPU_SESSION_STATE_PROFILE);

p_err:
	return ret;
}

int npu_session_buf_votf(struct npu_session *session, struct vs4l_buf_votf *buf_votf)
{
	int ret = 0;
	nw_cmd_e nw_cmd = NPU_NW_CMD_BUF_VOTF;

	if (!session) {
		npu_err("invalid session\n");
		ret = -EINVAL;
		return ret;
	}

	session->votf_prop = buf_votf->prop;

	mutex_lock(session->global_lock);
	session->nw_result.result_code = NPU_NW_JUST_STARTED;
	npu_session_put_nw_req(session, nw_cmd);

	wait_event(session->wq, session->nw_result.result_code != NPU_NW_JUST_STARTED);

	mutex_unlock(session->global_lock);

	return ret;
}
