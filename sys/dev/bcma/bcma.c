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
#include <sys/malloc.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/bhnd/bhnd_device_ids.h>
#include <dev/bhnd/bhndvar.h>

#include "bcmavar.h"

MALLOC_DEFINE(M_BCMA, "bcma", "BCMA bus data structures");

static const struct bcma_nomatch {
	uint16_t	vendor;		/**< core designer */
	uint16_t	device;		/**< core id */
	bool		report;		/**< always/only bootverbose */
} bcma_nomatch_table[] = {
	{ 0,			BHND_COREID_INVALID,		false }
};

int
bcma_generic_print_child(device_t dev, device_t child)
{
	struct resource_list	*rl;
	int			retval = 0;

	rl = BUS_GET_RESOURCE_LIST(dev, child);

	retval += bus_print_child_header(dev, child);

	retval += resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#lx");
	retval += printf(" at core %u", bhnd_get_core_index(child));

	retval += bus_print_child_domain(dev, child);
	retval += bus_print_child_footer(dev, child);

	return (retval);
}

void
bcma_generic_probe_nomatch(device_t dev, device_t child)
{
	struct bcma_corecfg		*cfg;
	struct bcma_devinfo		*dinfo;
	const struct bcma_nomatch	*nm;
	struct resource_list		*rl;
	bool				report;

	dinfo = device_get_ivars(child);
	cfg = &dinfo->cfg;
	rl = &dinfo->resources;
	report = true;

	for (nm = bcma_nomatch_table; nm->device != BHND_COREID_INVALID; nm++) {
		if (nm->vendor == bhnd_get_vendor(child) &&
		    nm->device == bhnd_get_device(child))
		{
			report = nm->report;
		}
	}
	
	if (report || bootverbose) {
		device_printf(dev, "<%s %s>", bhnd_get_vendor_name(child),
		    bhnd_get_device_name(child));

		resource_list_print_type(rl, "mem", SYS_RES_MEMORY, "%#lx");
		printf(" at core %u (no driver attached)\n",
		    bhnd_get_core_index(child));
	}
}

int
bcma_generic_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	const struct bcma_devinfo *dinfo;
	const struct bcma_corecfg *cfg;
	
	dinfo = device_get_ivars(child);
	cfg = &dinfo->cfg;
	
	switch (index) {
	case BHND_IVAR_VENDOR:
		*result = cfg->vendor;
		return (0);
	case BHND_IVAR_DEVICE:
		*result = cfg->device;
		return (0);
	case BHND_IVAR_REVID:
		*result = cfg->revid;
		return (0);
	case BHND_IVAR_DEVICE_CLASS:
		*result = bhnd_core_class(cfg->vendor, cfg->device);
		return (0);
	case BHND_IVAR_VENDOR_NAME:
		*result = (uintptr_t) bhnd_vendor_name(cfg->vendor);
		return (0);
	case BHND_IVAR_DEVICE_NAME:
		*result = (uintptr_t) bhnd_core_name(cfg->vendor, cfg->device);
		return (0);
	case BHND_IVAR_CORE_INDEX:
		*result = cfg->core_index;
		return (0);
	default:
		return (ENOENT);
	}
}

int
bcma_generic_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	switch (index) {
	case BHND_IVAR_VENDOR:
	case BHND_IVAR_DEVICE:
	case BHND_IVAR_REVID:
	case BHND_IVAR_DEVICE_CLASS:
	case BHND_IVAR_VENDOR_NAME:
	case BHND_IVAR_DEVICE_NAME:
	case BHND_IVAR_CORE_INDEX:
		return (EINVAL);
	default:
		return (ENOENT);
	}
}

void
bcma_generic_child_deleted(device_t dev, device_t child)
{
	struct bcma_devinfo *dinfo = device_get_ivars(child);
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);
}

struct resource_list *
bcma_generic_get_resource_list(device_t dev, device_t child)
{
	struct bcma_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

int
bcma_generic_get_port_rid(device_t dev, device_t child, u_int port_num, u_int
    region_num)
{
	struct bcma_devinfo	*dinfo;
	struct bcma_map		*map;
	struct bcma_sport	*port;
	
	dinfo = device_get_ivars(child);
	
	if (port_num > dinfo->cfg.num_dports)
		return -1;

	STAILQ_FOREACH(port, &dinfo->cfg.dports, sp_link) {
		if (port->sp_num != port_num)
			continue;
		
		STAILQ_FOREACH(map, &port->sp_maps, m_link) {
			if (map->m_region_num == region_num) {
				return map->m_rid;
			}
		}
	}

	return -1;
}

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
 * @param vendor Core designer.
 * @param device Core identifier (e.g. part number).
 * @param revision Core revision identifier.
 */
struct bcma_devinfo *
bcma_alloc_dinfo(u_int core_index, uint16_t vendor, uint16_t device, uint8_t revid)
{
	struct bcma_devinfo *dinfo;
	
	dinfo = malloc(sizeof(struct bcma_devinfo), M_BCMA, M_WAITOK);
	if (dinfo == NULL)
		return NULL;

	dinfo->cfg.vendor = vendor;
	dinfo->cfg.device = device;
	dinfo->cfg.revid = revid;
	dinfo->cfg.core_index = core_index;

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
		free(mport, M_BCMA);
	}
	
	STAILQ_FOREACH_SAFE(sport, &dinfo->cfg.dports, sp_link, snext) {
		bcma_free_sport(sport);
	}
	
	STAILQ_FOREACH_SAFE(sport, &dinfo->cfg.wports, sp_link, snext) {
		bcma_free_sport(sport);
	}

	free(dinfo, M_BCMA);
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
	
	sport = malloc(sizeof(struct bcma_sport), M_BCMA, M_WAITOK);
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
		free(map, M_BCMA);
	}

	free(sport, M_BCMA);
}