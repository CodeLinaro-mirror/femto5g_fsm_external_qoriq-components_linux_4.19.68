/* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __FSM_DP_MEM_H__
#define __FSM_DP_MEM_H__

#include <linux/types.h>
#include <linux/fsm_dp_ioctl.h>

struct fsm_dp_drv;

struct fsm_dp_mem_loc {
	void *base;		/* virtual address */
	void *page_base;	/* page aligned base address */
	unsigned int page_off;	/* offset to the page */
	size_t size;		/* size of memory chunk */
	dma_addr_t addr;	/* physical address */
	unsigned int cookie;	/* mmap cookie */
	bool dma_mapped;
	dma_addr_t dma_addr;	/* dma addr */
	enum dma_data_direction direction;
};

struct fsm_dp_mem {
	struct fsm_dp_mem_loc loc;		/* location */
	unsigned int buf_cnt;		/* buffer counter */
	unsigned int buf_sz;		/* buffer size */
	unsigned int buf_headroom_sz;	/* headroom unused for now */
	unsigned int buf_overhead_sz;	/* buffer overhead size */
};

struct fsm_dp_ring_opstats {
	unsigned long read_ok;
	unsigned long read_empty;
	unsigned long cons_head_updt_retry;
	unsigned long cons_tail_no_updt;
	unsigned long cons_tail_updt_backoff;
	unsigned long cons_tail_updt;
	unsigned long cons_tail_updt_stop;

	unsigned long write_ok;
	unsigned long write_full;
	unsigned long prod_head_updt_retry;
	unsigned long prod_tail_no_updt;
	unsigned long prod_tail_updt_backoff;
	unsigned long prod_tail_updt;
	unsigned long prod_tail_updt_stop;
};

struct fsm_dp_ring {
	struct fsm_dp_mem_loc loc;	/* location */
	unsigned int size;		/* size of ring(power of 2) */
	fsm_dp_ring_index_t *cons_head;	/* consumer index header */
	fsm_dp_ring_index_t *cons_tail;	/* consumer index tail */
	fsm_dp_ring_index_t *prod_head;	/* producer index header */
	fsm_dp_ring_index_t *prod_tail;	/* producer index tail */
	fsm_dp_ring_element_t *element;	/* ring element */
	struct fsm_dp_ring_opstats opstats;
};

struct fsm_dp_mempool_stats {
	unsigned long buf_put;
	unsigned long buf_get;
	unsigned long invalid_buf_put;
	unsigned long invalid_buf_get;
	unsigned long buf_put_err;
	unsigned long buf_get_err;
};

#define FSM_DP_MEMPOOL_SIG 0xdeadbeef
#define FSM_DP_MEMPOOL_SIG_BAD 0xbeefdead

struct fsm_dp_mempool {
	unsigned int signature;
	struct fsm_dp_drv *drv;
	enum fsm_dp_mem_type type;
	struct fsm_dp_ring ring;
	struct fsm_dp_mem mem;
	atomic_t ref;
	atomic_t out_xmit;
	struct fsm_dp_mempool_stats stats;
	char *dummy_buf;
};


struct fsm_dp_mempool *fsm_dp_mempool_alloc(
	struct fsm_dp_drv *pdrv,
	enum fsm_dp_mem_type type,
	unsigned int buf_sz,
	unsigned int buf_cnt,
	bool may_dma_map);

void fsm_dp_mempool_free(struct fsm_dp_mempool *mempool);

int fsm_dp_mempool_get_cfg(
	struct fsm_dp_mempool *mempool,
	struct fsm_dp_mempool_cfg *cfg);

int fsm_dp_mempool_put_buf(struct fsm_dp_mempool *mempool, void *vaddr);
void *fsm_dp_mempool_get_buf(struct fsm_dp_mempool *mempool);

static inline bool fsm_dp_mempool_hold(struct fsm_dp_mempool *mempool)
{
	bool ret = false;

	if (!mempool)
		return ret;
	smp_mb__before_atomic();
	if (atomic_inc_not_zero(&mempool->ref))
		ret = true;
	smp_mb__after_atomic();
	return ret;
}

static inline void __fsm_dp_mempool_hold(struct fsm_dp_mempool *mempool)
{
	atomic_inc(&mempool->ref);
}

static inline void fsm_dp_mempool_put(struct fsm_dp_mempool *mempool)
{
	if (mempool && atomic_dec_and_test(&mempool->ref))
		fsm_dp_mempool_free(mempool);
}

int fsm_dp_ring_init(
	struct fsm_dp_ring *ring,
	unsigned int ringsz,
	unsigned int mmap_cookie);

void fsm_dp_ring_cleanup(struct fsm_dp_ring *ring);

int fsm_dp_ring_read(struct fsm_dp_ring *ring, fsm_dp_ring_element_data_t *element_data,
		unsigned int *flag);

int fsm_dp_ring_write(struct fsm_dp_ring *ring, fsm_dp_ring_element_data_t element_data,
		unsigned int flag);

bool fsm_dp_ring_is_empty(struct fsm_dp_ring *ring);

int fsm_dp_ring_get_cfg(struct fsm_dp_ring *ring, struct fsm_dp_ring_cfg *cfg);

struct fsm_dp_mempool *fsm_dp_find_mempool(
	struct fsm_dp_drv *drv,
	void *addr,
	bool tx);

int fsm_dp_mempool_dma_map(
	struct fsm_dp_drv *pdrv,
	struct fsm_dp_mempool *mpool,
	enum fsm_dp_mem_type type);

/* inline */
static __always_inline bool __ulong_in_range(
	unsigned long v,
	unsigned long start,
	unsigned long end)
{
	return (v >= start && v < end);
}

static __always_inline bool ulong_in_range(
	unsigned long v,
	unsigned long start,
	size_t size)
{
	return __ulong_in_range(v, start, start+size-1);
}

static __always_inline bool __vaddr_in_range(void *addr, void *start, void *end)
{
	return __ulong_in_range((unsigned long)addr,
				(unsigned long)start,
				(unsigned long)end);
}

static __always_inline bool vaddr_in_range(void *addr, void *start, size_t size)
{
	return __vaddr_in_range(addr, start, (char *)start + size - 1);
}

static __always_inline bool __vaddr_in_vma_range(
	void __user *vaddr_start,
	void __user *vaddr_end,
	struct vm_area_struct *vma)
{
	unsigned long start = (unsigned long)vaddr_start;
	unsigned long end = (unsigned long)vaddr_end;

	return (start >= vma->vm_start && end < vma->vm_end);
}

static __always_inline bool vaddr_in_vma_range(
	void __user *vaddr,
	size_t len,
	struct vm_area_struct *vma)
{
	return __vaddr_in_vma_range(vaddr,
				    (void __user *)((char *)vaddr + len - 1),
				    vma);
}

static __always_inline unsigned long vaddr_offset(void *addr, void *base)
{
	return (unsigned long)addr - (unsigned long)base;
}

static __always_inline unsigned long fsm_dp_mem_loc_mmap_size(
	struct fsm_dp_mem_loc *loc)
{
	return (loc->size + loc->page_off);
}

static inline unsigned int calc_ring_size(unsigned int elements)
{
	unsigned int size = 1, shift = 0;

	for (shift = 0; (shift < (sizeof(unsigned int) * 8 - 1)); shift++) {
		if (size >= elements)
			return size;
		size <<= 1;
	}
	return 0;
}

/* set buffer state, ptr: pointing to beginging of buffer user data */
static inline void fsm_dp_set_buf_state(void *ptr, enum fsm_dp_buf_state state)
{
	struct fsm_dp_buf_cntrl *pf = (ptr - FSM_DP_L1_CACHE_BYTES);

	pf->state = state;
}

/* get true buffer size which includes size for user space and control  */
static inline uint32_t fsm_dp_buf_true_size(struct fsm_dp_mem *mem)
{
	return (mem->buf_sz + mem->buf_overhead_sz);
}

#endif /* __FSM_DP_MEM_H__ */
