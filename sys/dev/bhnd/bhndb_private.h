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

#ifndef _BHND_BHNDB_PRIVATE_H_
#define _BHND_BHNDB_PRIVATE_H_

/*
 * 
 */


#include <sys/param.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include "bhnd.h"


/**
 * BHNDB register window types.
 */
typedef enum {
        BHNDB_REGWIN_T_CORE,            /**< Fixed mapping of a core register block. */
        BHNDB_REGWIN_T_SPROM,           /**< Fixed mapping an SPROM */
        BHNDB_REGWIN_T_DYN              /**< A dynamically configurable window */
} bhndb_regwin_type_t;


/**
 * BHNDB register window definition.
 */
struct bhndb_regwin {
	int			rid;	/**< resource-id of this register window */
	bus_size_t		offset;	/**< offset of the window within the resource */
	bus_size_t		size;	/**< size of the window */
	bhndb_regwin_type_t	type;	/**< window type */

	union {
		/** Core-specific register window (BHNDB_REGWIN_T_CORE). */
		struct {
			bhnd_devclass_t	class;	/**< mapped core's class */
			u_int		unit;	/**< mapped core's unit */
			u_int		port;	/**< mapped port number */
			u_int		region;	/**< mapped region number */
		} core;

		/** SPROM register window (BHNDB_REGWIN_T_SPROM). */
		struct {} sprom;

                /** Dynamic register window (BHNDB_REGWIN_T_DYN). */
		struct {
			bus_size_t	cfg_offset;	/**< window address config offset. */
		} dyn;
        };
};

#define	BHNDB_REGWIN_TABLE_END	{ -1, 0, 0, 0 }

/**
 * BHNDB hardware configuration.
 */
struct bhndb_hw_cfg {
	struct resource_spec	*resources;
	struct bhndb_regwin	*reg_windows;
};

#endif /* _BHND_BHNDB_PRIVATE_H_ */
