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

int	bhndb_generic_probe(device_t dev);
int	bhndb_generic_attach(device_t dev);
int	bhndb_generic_detach(device_t dev);
int	bhndb_generic_suspend(device_t dev);
int	bhndb_generic_resume(device_t dev);
int	bhndb_generic_read_ivar(device_t dev, device_t child, int index,
	    uintptr_t *result);
int	bhndb_generic_write_ivar(device_t dev, device_t child, int index,
	    uintptr_t value);

size_t				 bhndb_regwin_count(
				     const struct bhndb_regwin *table,
				     bhndb_regwin_type_t type);

const struct bhndb_regwin	*bhndb_regwin_find_type(
				     const struct bhndb_regwin *table,
				     bhndb_regwin_type_t type,
				     bus_size_t min_size);

const struct bhndb_regwin	*bhndb_regwin_find_core(
				     const struct bhndb_regwin *table,
				     bhnd_devclass_t class, int unit, int port,
				     int region);

const struct bhndb_regwin	*bhndb_regwin_find_core_or_dyn(
				     const struct bhndb_regwin *table,
				     bhnd_devclass_t class, int unit, int port,
				     int region, bus_size_t min_size);

/**
 * bhndb driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bhndb_softc {
	device_t			 dev;		/**< bridge device */
	const struct bhndb_hw		*hw;		/**< hardware spec */
	struct bhnd_chipid		 chipid;	/**< chip identification */

	device_t			 parent_dev;	/**< parent device */
	size_t				 res_count;	/**< parent bus resource count */
	struct resource_spec		*res_spec;	/**< parent bus resource specs */
	struct resource			**res;		/**< parent bus resources */

	device_t			 bus_dev;	/**< child bhnd(4) bus */
	struct rman			 mem_rman;	/**< bridged bus memory manager */

	struct mtx			 sc_mtx;	/**< softc lock. */
	
	struct bhndb_regwin_region	*dw_regions;	/**< dynamic window regions */
	size_t				 dw_count;	/**< number of dynamic window regions. */
	uint32_t			 dw_freelist;	/**< dw_regions free list */
};

#endif /* _BHND_BHNDBVAR_H_ */
