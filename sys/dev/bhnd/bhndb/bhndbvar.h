/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * $FreeBSD$
 */

#ifndef _BHND_BHNDBVAR_H_
#define _BHND_BHNDBVAR_H_

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <dev/bhnd/bhnd.h>
#include "bhndb.h"

#include "bhndb_if.h"

/*
 * Definitions shared by bhndb(4) driver implementations.
 */

DECLARE_CLASS(bhndb_driver);

int	bhndb_gen_probe(device_t dev);
int	bhndb_gen_attach(device_t dev);
int	bhndb_gen_detach(device_t dev);
int	bhndb_gen_suspend(device_t dev);
int	bhndb_gen_resume(device_t dev);
int	bhndb_gen_read_ivar(device_t dev, device_t child, int index,
	    uintptr_t *result);
int	bhndb_gen_write_ivar(device_t dev, device_t child, int index,
	    uintptr_t value);


size_t				 bhndb_regwin_count(
				     const struct bhndb_regwin *table,
				     bhndb_regwin_type_t type);

const struct bhndb_regwin	*bhndb_regwin_find_type(
				     const struct bhndb_regwin *table,
				     bhndb_regwin_type_t type);

const struct bhndb_regwin	*bhndb_regwin_find_core(
				     const struct bhndb_regwin *table,
				     bhnd_devclass_t class, int unit, int port,
				     int region);

/**
 * bhndb driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bhndb_softc {
	device_t			 dev;		/**< bridge device */
	const struct bhndb_hw		*hw;		/**< hardware spec */

	device_t			 parent_dev;	/**< parent device */
	size_t				 res_count;	/**< parent bus resource count */
	struct resource_spec		*res_spec;	/**< parent bus resource specs */
	struct resource			**res;		/**< parent bus resources */

	struct rman			 mem_rman;	/**< bridged bus memory manager */

	struct mtx			 sc_mtx;	/**< softc lock. */
	
	struct bhndb_regwin_region	*dw_regions;	/**< dynamic window regions */
	size_t				 dw_count;	/**< number of dynamic window regions. */
	uint32_t			 dw_freelist;	/**< dw_regions free list */
};

#endif /* _BHND_BHNDBVAR_H_ */
