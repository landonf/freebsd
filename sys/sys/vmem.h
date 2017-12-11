/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c)2006 YAMAMOTO Takashi,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/* From	$NetBSD: vmem.h,v 1.20 2013/01/29 21:26:24 para Exp $	*/

/* $FreeBSD$ */

#ifndef _SYS_VMEM_H_
#define	_SYS_VMEM_H_

#include <sys/types.h>

#ifdef _KERNEL

#include <sys/proc.h>
#include <sys/condvar.h>

#include <vm/uma.h>

typedef struct vmem vmem_t;

typedef uintptr_t	vmem_addr_t;
typedef size_t		vmem_size_t;

#define	VMEM_ADDR_MIN	0
#define	VMEM_ADDR_MAX	(~(vmem_addr_t)0)

typedef int (vmem_import_t)(void *, vmem_size_t, int, vmem_addr_t *);
typedef void (vmem_release_t)(void *, vmem_addr_t, vmem_size_t);
typedef void (vmem_reclaim_t)(vmem_t *, int);

/*
 * Create a vmem:
 *	name		- Name of the region
 *	base		- Initial span start (optional)
 *	size		- Initial span size
 *	quantum		- Natural unit of allocation (ie PAGE_SIZE, 1, etc)
 *	qcache_max	- Maximum size to quantum cache.  This creates a UMA
 *			  cache for each multiple of quantum up to qcache_max.
 *	flags		- M_* flags
 */
vmem_t *vmem_create(const char *name, vmem_addr_t base,
    vmem_size_t size, vmem_size_t quantum, vmem_size_t qcache_max, int flags);
vmem_t *vmem_init(vmem_t *vm, const char *name, vmem_addr_t base,
    vmem_size_t size, vmem_size_t quantum, vmem_size_t qcache_max, int flags);
void vmem_destroy(vmem_t *);

/*
 * Set callbacks for bringing in dynamic regions:
 *	importfn	- Backing store import routine.
 *	releasefn	- Backing store release routine.
 *	arg		- Backing store argument
 *	import_quantum	- Size to import from backing store
 */

void vmem_set_import(vmem_t *vm, vmem_import_t *importfn,
    vmem_release_t *releasefn, void *arg, vmem_size_t import_quantum);

/*
 * Set a limit on the total size of a vmem.
 */

void vmem_set_limit(vmem_t *vm, vmem_size_t limit);

/*
 * Set a callback for reclaiming memory when space is exhausted:
 */
void vmem_set_reclaim(vmem_t *vm, vmem_reclaim_t *reclaimfn);

/*
 * Allocate and free linear regions from a vmem.  Must specify
 * BESTFIT or FIRSTFIT.  Free is non-blocking.  These routines
 * respect the quantum caches.
 */
int vmem_alloc(vmem_t *vm, vmem_size_t size, int flags, vmem_addr_t *addrp);
void vmem_free(vmem_t *vm, vmem_addr_t addr, vmem_size_t size);

/*
 * Constrained allocate and free routines.  These bypass the quantum cache.
 *	size		- Size in units of 1, not quantum.
 *	align		- Required alignment of the start of region
 *	phase		- Offset from alignment
 *	nocross		- Illegal boundary
 *	minaddr		- Minimum allowed address for last byte
 *	maxaddr		- Maximum allowed address for first byte
 *	flags		- M_* flags
 *	addrp		- result
 */
int vmem_xalloc(vmem_t *vm, vmem_size_t size, vmem_size_t align,
    vmem_size_t phase, vmem_size_t nocross, vmem_addr_t minaddr,
    vmem_addr_t maxaddr, int flags, vmem_addr_t *addrp);
void vmem_xfree(vmem_t *vm, vmem_addr_t addr, vmem_size_t size);

/*
 * Add a static region to a vmem after create.  This won't be freed
 * until the vmem is destroyed.
 */
int vmem_add(vmem_t *vm, vmem_addr_t addr, vmem_size_t size, int flags);

/*
 * Given roundup size to the vmem's native quantum size.
 */
vmem_size_t vmem_roundup_size(vmem_t *vm, vmem_size_t size);

/*
 * Report vmem utilization according to the requested type.
 */
vmem_size_t vmem_size(vmem_t *vm, int typemask);

void vmem_whatis(vmem_addr_t addr, int (*fn)(const char *, ...)
    __printflike(1, 2));
void vmem_print(vmem_addr_t addr, const char *, int (*fn)(const char *, ...)
    __printflike(1, 2));
void vmem_printall(const char *, int (*fn)(const char *, ...)
    __printflike(1, 2));
void vmem_startup(void);

/* vmem_size typemask */
#define VMEM_ALLOC	0x01
#define VMEM_FREE	0x02
#define VMEM_MAXFREE	0x10

// XXX hacked into booting
/* vmem arena */
typedef struct vmem_btag bt_t;

#define	QC_NAME_MAX	16

struct qcache {
	uma_zone_t	qc_cache;
	vmem_t 		*qc_vmem;
	vmem_size_t	qc_size;
	char		qc_name[QC_NAME_MAX];
};
typedef struct qcache qcache_t;
#define	QC_POOL_TO_QCACHE(pool)	((qcache_t *)(pool->pr_qcache))


/* boundary tag */
struct vmem_btag {
	TAILQ_ENTRY(vmem_btag) bt_seglist;
	union {
		LIST_ENTRY(vmem_btag) u_freelist; /* BT_TYPE_FREE */
		LIST_ENTRY(vmem_btag) u_hashlist; /* BT_TYPE_BUSY */
	} bt_u;
#define	bt_hashlist	bt_u.u_hashlist
#define	bt_freelist	bt_u.u_freelist
	vmem_addr_t	bt_start;
	vmem_size_t	bt_size;
	int		bt_type;
};

TAILQ_HEAD(vmem_seglist, vmem_btag);
LIST_HEAD(vmem_freelist, vmem_btag);
LIST_HEAD(vmem_hashlist, vmem_btag);

#define	VMEM_NAME_MAX	16
#define	VMEM_OPTORDER		5
#define	VMEM_OPTVALUE		(1 << VMEM_OPTORDER)
#define	VMEM_MAXORDER						\
    (VMEM_OPTVALUE - 1 + sizeof(vmem_size_t) * NBBY - VMEM_OPTORDER)

#define	VMEM_HASHSIZE_MIN	16
#define	VMEM_HASHSIZE_MAX	131072

#define	VMEM_QCACHE_IDX_MAX	16

#define	VMEM_FITMASK	(M_BESTFIT | M_FIRSTFIT)

#define	VMEM_FLAGS						\
    (M_NOWAIT | M_WAITOK | M_USE_RESERVE | M_NOVM | M_BESTFIT | M_FIRSTFIT)
struct vmem {
	struct mtx_padalign	vm_lock;
	struct cv		vm_cv;
	char			vm_name[VMEM_NAME_MAX+1];
	LIST_ENTRY(vmem)	vm_alllist;
	struct vmem_hashlist	vm_hash0[VMEM_HASHSIZE_MIN];
	struct vmem_freelist	vm_freelist[VMEM_MAXORDER];
	struct vmem_seglist	vm_seglist;
	struct vmem_hashlist	*vm_hashlist;
	vmem_size_t		vm_hashsize;

	/* Constant after init */
	vmem_size_t		vm_qcache_max;
	vmem_size_t		vm_quantum_mask;
	vmem_size_t		vm_import_quantum;
	int			vm_quantum_shift;

	/* Written on alloc/free */
	LIST_HEAD(, vmem_btag)	vm_freetags;
	int			vm_nfreetags;
	int			vm_nbusytag;
	vmem_size_t		vm_inuse;
	vmem_size_t		vm_size;
	vmem_size_t		vm_limit;

	/* Used on import. */
	vmem_import_t		*vm_importfn;
	vmem_release_t		*vm_releasefn;
	void			*vm_arg;

	/* Space exhaustion callback. */
	vmem_reclaim_t		*vm_reclaimfn;

	/* quantum cache */
	qcache_t		vm_qcache[VMEM_QCACHE_IDX_MAX];
};

#endif /* _KERNEL */

#endif /* !_SYS_VMEM_H_ */
