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

#include <machine/bus.h>
#include <sys/rman.h>

#include <dev/bhnd/bhnd.h>

#include "bhndb_if.h"

struct bhndb_regwin_region;

int	bhndb_attach_bridge(device_t parent, devclass_t devclass,
	    device_t *bhndb, int unit);

/**
 * bhndb register window types.
 */
typedef enum {
	BHNDB_REGWIN_T_CORE,		/**< Fixed mapping of a core register block. */
	BHNDB_REGWIN_T_SPROM,		/**< Fixed mapping of device SPROM */
	BHNDB_REGWIN_T_DYN,		/**< A dynamically configurable window */
	BHNDB_REGWIN_T_INVALID		/**< Invalid type */
} bhndb_regwin_type_t;

/**
 * bhndb register window definition.
 */
struct bhndb_regwin {
	bhndb_regwin_type_t	win_type;	/**< window type */
	bus_size_t		win_offset;	/**< offset of the window within the resource */
	bus_size_t		win_size;	/**< size of the window */
	
	/** Resource identification */
	struct {
		int		type;		/**< resource type */
		int		rid;		/**< resource id */
	} res;


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

#define	BHNDB_REGWIN_TABLE_END	{ BHNDB_REGWIN_T_INVALID, 0, 0, { 0, 0 } }

/**
 * bhndb hardware configuration.
 */
struct bhndb_hwcfg {
	const struct resource_spec	*resource_specs;
	const struct bhndb_regwin	*register_windows;
};

/**
 * Hardware configuration table entry.
 * 
 * Defines a set of match criteria that may be used to determine the
 * register map and resource configuration for a bhndb bridge device. 
 */
struct bhndb_hw {
	const char			*name;		/**< configuration name */
	const struct bhnd_core_match	*hw_reqs;	/**< match requirements */
	u_int				 num_hw_reqs;	/**< number of match requirements */
	const struct bhndb_hwcfg	*cfg;		/**< associated hardware configuration */
};

/**
 * bhndb driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bhndb_softc {
	device_t			 dev;		/**< bridge device */
	const struct bhndb_hwcfg	*hwcfg;		/**< hardware config */

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
