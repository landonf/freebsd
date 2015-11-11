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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd_private.h>

#include "bcma_private.h"


/**
 * Return the name of a slave port type.
 */
const char *
bcma_port_type_name (bcma_sport_type port_type)
{
	switch (port_type) {
	case BCMA_SPORT_TYPE_DEVICE:
		return "device";
	case BCMA_SPORT_TYPE_BRIDGE:
		return "bridge";
	case BCMA_SPORT_TYPE_SWRAP:
		return "swrap";
	case BCMA_SPORT_TYPE_MWRAP:
		return "mwrap";
	}
}

/**
 * Allocate and initialize new device info structure.
 * 
 * @param core_index Core index on the bus.
 * @param core_unit Core unit number.
 * @param vendor Core designer.
 * @param device Core identifier (e.g. part number).
 * @param revision Core revision identifier.
 */
struct bcma_devinfo *
bcma_alloc_dinfo(u_int core_index, int core_unit, uint16_t vendor,
    uint16_t device, uint8_t revid)
{
	struct bcma_devinfo *dinfo;
	
	dinfo = malloc(sizeof(struct bcma_devinfo), M_BHND, M_WAITOK);
	if (dinfo == NULL)
		return NULL;

	dinfo->cfg.vendor = vendor;
	dinfo->cfg.device = device;
	dinfo->cfg.revid = revid;
	dinfo->cfg.core_index = core_index;
	dinfo->cfg.core_unit = core_unit;

	resource_list_init(&dinfo->resources);

	STAILQ_INIT(&dinfo->cfg.mports);
	STAILQ_INIT(&dinfo->cfg.dports);
	STAILQ_INIT(&dinfo->cfg.wports);

	return dinfo;
}

/**
 * Deallocate the given device info structure and any associated resources.
 * 
 * @param dinfo Device info to be deallocated.
 */
void
bcma_free_dinfo(struct bcma_devinfo *dinfo)
{
	struct bcma_mport *mport, *mnext;
	struct bcma_sport *sport, *snext;

	resource_list_free(&dinfo->resources);

	STAILQ_FOREACH_SAFE(mport, &dinfo->cfg.mports, mp_link, mnext) {
		free(mport, M_BHND);
	}
	
	STAILQ_FOREACH_SAFE(sport, &dinfo->cfg.dports, sp_link, snext) {
		bcma_free_sport(sport);
	}
	
	STAILQ_FOREACH_SAFE(sport, &dinfo->cfg.wports, sp_link, snext) {
		bcma_free_sport(sport);
	}

	free(dinfo, M_BHND);
}


/**
 * Allocate and initialize new slave port descriptor.
 * 
 * @param port_num Per-core port number.
 * @param port_type Port type.
 */
struct bcma_sport *
bcma_alloc_sport(bcma_pid_t port_num, bcma_sport_type port_type)
{
	struct bcma_sport *sport;
	
	sport = malloc(sizeof(struct bcma_sport), M_BHND, M_WAITOK);
	if (sport == NULL)
		return NULL;
	
	sport->sp_num = port_num;
	sport->sp_type = port_type;
	sport->sp_num_maps = 0;
	STAILQ_INIT(&sport->sp_maps);

	return sport;
}

/**
 * Deallocate all resources associated with the given port descriptor.
 * 
 * @param sport Port descriptor to be deallocated.
 */
void
bcma_free_sport(struct bcma_sport *sport) {
	struct bcma_map *map, *mapnext;

	STAILQ_FOREACH_SAFE(map, &sport->sp_maps, m_link, mapnext) {
		free(map, M_BHND);
	}

	free(sport, M_BHND);
}

