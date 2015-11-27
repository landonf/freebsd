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

#ifndef _BHND_BHNDB_H_
#define _BHND_BHNDB_H_

#include <sys/param.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhndb_bus_if.h"

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
 * Evaluate to true if the given register window type defines a static
 * mapping.
 * 
 * @param rtype Window type.
 */
#define	BHNDB_REGWIN_T_IS_STATIC(rtype)	\
    ((rtype) == BHNDB_REGWIN_T_CORE || (rtype) == BHNDB_REGWIN_T_SPROM)

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
 * Bridge hardware configuration.
 * 
 * Provides the bridge's register/address mappings, and the resources
 * via which those mappings may be accessed.
 */
struct bhndb_hwcfg {
	const struct resource_spec	*resource_specs;
	const struct bhndb_regwin	*register_windows;
};

/**
 * Hardware specification entry.
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

#endif /* _BHND_BHNDB_H_ */