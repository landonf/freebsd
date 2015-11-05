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
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/bhnd/bhnd.h>

#include "bcmavar.h"

MALLOC_DEFINE(M_BCMA, "bcma", "BCMA bus data structures");

static int
bcma_probe(device_t dev)
{
	device_set_desc(dev, "BCMA bus");
	return (BUS_PROBE_NOWILDCARD);
}

static int
bcma_attach(device_t dev)
{
	return (bus_generic_attach(dev));
}

static int
bcma_detach(device_t dev)
{
	return (bus_generic_detach(dev));
}

static int
bcma_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
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

static int
bcma_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
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

static void
bcma_child_deleted(device_t dev, device_t child)
{
	struct bcma_devinfo *dinfo = device_get_ivars(child);
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);
}

static struct resource_list *
bcma_get_resource_list(device_t dev, device_t child)
{
	struct bcma_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

static int
bcma_get_port_rid(device_t dev, device_t child, u_int port_num, u_int
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

static device_method_t bcma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_probe),
	DEVMETHOD(device_attach,		bcma_attach),
	DEVMETHOD(device_detach,		bcma_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bus_generic_suspend), // TODO
	DEVMETHOD(device_resume,		bus_generic_resume), // TODO
	
	/* Bus interface */
	DEVMETHOD(bus_child_deleted,		bcma_child_deleted),
	DEVMETHOD(bus_print_child,		bhnd_generic_print_child),
	DEVMETHOD(bus_probe_nomatch,		bhnd_generic_probe_nomatch),
	DEVMETHOD(bus_read_ivar,		bcma_read_ivar),
	DEVMETHOD(bus_write_ivar,		bcma_write_ivar),

	DEVMETHOD(bus_get_resource_list,	bcma_get_resource_list),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_alloc_resource,		bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_adjust_resource,		bus_generic_adjust_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),

	/* BHND interface */
	DEVMETHOD(bhndbus_get_port_rid,		bcma_get_port_rid),
	DEVMETHOD(bhndbus_alloc_resource,	bhnd_generic_alloc_bhnd_resource),
	DEVMETHOD(bhndbus_release_resource,	bhnd_generic_release_bhnd_resource),
	DEVMETHOD(bhndbus_activate_resource,	bhnd_generic_activate_bhnd_resource),
	DEVMETHOD(bhndbus_deactivate_resource,	bhnd_generic_deactivate_bhnd_resource),

	DEVMETHOD_END
};

devclass_t bcma_devclass;
devclass_t bcmab_devclass;

driver_t bcma_driver = {
	BCMA_DEVNAME,
	bcma_methods,
	sizeof(struct bcma_softc)
};

MODULE_VERSION(bcma, 1);
MODULE_DEPEND(bcma, bhnd, 1, 1, 1);
DRIVER_MODULE(bcma, bcmab, bcma_driver, bcma_devclass, NULL, NULL);
