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

/**
 * Private per-core flags
 */
enum {
	BHNDB_CF_HW_DISABLED	= 1 << 0,	/**< core hardware is unusable */
	BHNDB_CF_HOSTB		= 1 << 1,	/**< core is host bridge */
};

/** bhndb child instance state */
struct bhndb_devinfo {
        struct resource_list    resources;	/**< child resources. */
};

/**
 * A register window allocation record. 
 */
struct bhndb_regwin_region {
	const struct bhndb_regwin	*win;		/**< window definition */
	struct resource			*parent_res;	/**< enclosing resource */
	struct resource			*child_res;	/**< associated child resource, or NULL */
	u_int				 rnid;		/**< region identifier */
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
#define	BHNDB_DW_REGION_RELEASE(sc, rnid)	do {		\
	KASSERT((sc)->dw_regions[rnid].child_res != NULL &&	\
	    !BHNDB_DW_REGION_IS_FREE((sc), (rnid)),		\
	    (("dw_region double free")));			\
								\
	(sc)->dw_freelist |= (1 << (rnid));			\
	(sc)->dw_regions[rnid].child_res = NULL;		\
} while(0)

/**
 * Mark a dynamic window region as reserved.
 */
#define	BHNDB_DW_REGION_RESERVE(sc, rnid, cr)	do {		\
	KASSERT((sc)->dw_regions[rnid].child_res == NULL &&	\
	    BHNDB_DW_REGION_IS_FREE((sc), (rnid)),		\
	    (("dw_region is busy")));				\
								\
	(sc)->dw_freelist &= ~(1 << (rnid));			\
	(sc)->dw_regions[rnid].child_res = cr;			\
} while(0)

/**
 * Return non-zero value if a dynamic window region is marked as free.
 */
#define	BHNDB_DW_REGION_IS_FREE(sc, rnid) \
	((sc)->dw_freelist & (1 << (rnid)))

#endif /* _BHND_BHNDB_PRIVATE_H_ */