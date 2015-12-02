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

#ifndef _BHND_BHNDB_H_
#define _BHND_BHNDB_H_

#include <sys/param.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhndb_bus_if.h"

extern devclass_t bhndb_devclass;

int	bhndb_attach_bridge(device_t parent, device_t *bhndb, int unit);

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