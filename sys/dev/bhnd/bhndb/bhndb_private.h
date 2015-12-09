/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHNDB_PRIVATE_H_
#define _BHND_BHNDB_PRIVATE_H_

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include "bhndbvar.h"

/*
 * Private bhndb(4) driver definitions.
 */

struct bhndb_dw_region;
struct bhndb_region;
struct bhndb_resources;

struct resource			*bhndb_find_regwin_resource(
				     struct bhndb_resources *r,
				     const struct bhndb_regwin *win);

struct bhndb_resources		*bhndb_alloc_resources(device_t dev,
				     device_t parent_dev,
				     const struct bhndb_hwcfg *cfg);

void				 bhndb_free_resources(
				     struct bhndb_resources *res);

int				 bhndb_resources_add_device_region(
				     struct bhndb_resources *r, device_t dev,
				     bhnd_port_type port_type, u_int port,
				     u_int region,
				     const struct bhndb_regwin *static_regwin, 
				     bhndb_priority_t priority);

struct bhndb_region		*bhndb_resources_find_region(
				     struct bhndb_resources *r,
				     bhnd_addr_t addr, bhnd_size_t size);

size_t				 bhndb_regwin_count(
				     const struct bhndb_regwin *table,
				     bhndb_regwin_type_t type);

const struct bhndb_regwin	*bhndb_regwin_find_type(
				     const struct bhndb_regwin *table,
				     bhndb_regwin_type_t type,
				     bus_size_t min_size);

const struct bhndb_regwin	*bhndb_regwin_find_core(
				     const struct bhndb_regwin *table,
				     bhnd_devclass_t class, int unit,
				     bhnd_port_type port_type, u_int port,
				     u_int region);


const struct bhndb_regwin	*bhndb_regwin_find_best(
				     const struct bhndb_regwin *table,
				     bhnd_devclass_t class, int unit,
				     bhnd_port_type port_type, u_int port,
				     u_int region, bus_size_t min_size);

bool				 bhndb_regwin_matches_device(
				     const struct bhndb_regwin *regw,
				     device_t dev);

const struct bhndb_hw_priority	*bhndb_hw_priority_find_device(
				     const struct bhndb_hw_priority *table,
				     device_t device);


/**
 * A dynamic register window allocation record. 
 */
struct bhndb_dw_region {
	const struct bhndb_regwin	*win;		/**< window definition */
	struct resource			*parent_res;	/**< enclosing resource */
	struct resource			*child_res;	/**< associated child resource, or NULL */
	u_int				 rnid;		/**< region identifier */
};

/**
 * A bus address region description.
 */
struct bhndb_region {
	bhnd_addr_t			 addr;		/**< start of mapped range */
	bhnd_size_t			 size;		/**< size of mapped range */
	bhndb_priority_t		 dw_priority;	/**< dynamic window allocation priority */
	const struct bhndb_regwin	*static_regwin;	/**< fixed mapping regwin, if any */

	STAILQ_ENTRY(bhndb_region)	 link;
};

/**
 * BHNDB resource allocation state.
 */
struct bhndb_resources {
	device_t			 dev;		/**< bridge device */
	const struct bhndb_hwcfg	*cfg;		/**< hardware configuration */

	device_t			 parent_dev;	/**< parent device */
	struct resource_spec		*res_spec;	/**< parent bus resource specs */
	struct resource			**res;		/**< parent bus resources */

	STAILQ_HEAD(, bhndb_region) 	 bus_regions;	/**< bus region descriptors */

	struct bhndb_dw_region		*dw_regions;	/**< dynamic window regions */
	size_t				 dw_count;	/**< number of dynamic window regions. */
	uint32_t			 dw_freelist;	/**< dw_regions free list */
	bhndb_priority_t		 dw_min_prio;	/**< minimum resource priority required to
							     allocate a dynamic window region */
};

#define	BHNDB_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), \
	    "bhndb_gen resource allocator lock", MTX_DEF)
#define	BHNDB_LOCK(sc)			mtx_lock(&(sc)->sc_mtx)
#define	BHNDB_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define	BHNDB_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->sc_mtx, what)
#define	BHNDB_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->sc_mtx)

/**
 * Mark a dynamic window region as free.
 */
#define	BHNDB_DW_REGION_RELEASE(r, rnid)	do {		\
	KASSERT((r)->dw_regions[rnid].child_res != NULL &&	\
	    !BHNDB_DW_REGION_IS_FREE((r), (rnid)),		\
	    (("dw_region double free")));			\
								\
	(r)->dw_freelist |= (1 << (rnid));			\
	(r)->dw_regions[rnid].child_res = NULL;		\
} while(0)

/**
 * Evaluates to true if the all dynamic regions have been exhausted.
 */
#define	BHNDB_DW_REGION_EXHAUSTED(r)		((r)->dw_freelist == 0)

/**
 * Find the next free dynamic window region. It is an error to
 * call this macro without first checking if BHNDB_DW_REGION_EXHAUSTED
 * evaluates to true.
 */
#define	BHNDB_DW_REGION_NEXT_FREE(r)		__builtin_ctz((r)->dw_freelist)

/**
 * Mark a dynamic window region as reserved.
 */
#define	BHNDB_DW_REGION_RESERVE(r, rnid, cr)	do {		\
	KASSERT((r)->dw_regions[rnid].child_res == NULL &&	\
	    BHNDB_DW_REGION_IS_FREE((r), (rnid)),		\
	    (("dw_region is busy")));				\
								\
	(r)->dw_freelist &= ~(1 << (rnid));			\
	(r)->dw_regions[rnid].child_res = cr;			\
} while(0)

/**
 * Return non-zero value if a dynamic window region is marked as free.
 */
#define	BHNDB_DW_REGION_IS_FREE(r, rnid) \
	((r)->dw_freelist & (1 << (rnid)))

#endif /* _BHND_BHNDB_PRIVATE_H_ */