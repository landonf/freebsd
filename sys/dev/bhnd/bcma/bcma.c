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

#include "bcmavar.h"

#include "bcma_eromreg.h"
#include "bcma_eromvar.h"

int
bcma_probe(device_t dev)
{
	device_set_desc(dev, "BCMA bus");
	return (BUS_PROBE_NOWILDCARD);
}

int
bcma_attach(device_t dev)
{
	/* Bus' generic attach will probe and attach any enumerated children */
	return (bus_generic_attach(dev));
}

int
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
	cfg = dinfo->corecfg;
	
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
	case BHND_IVAR_CORE_UNIT:
		*result = cfg->core_unit;
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
	case BHND_IVAR_CORE_UNIT:
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
	
	if (port_num > dinfo->corecfg->num_dev_ports)
		return -1;

	STAILQ_FOREACH(port, &dinfo->corecfg->dev_ports, sp_link) {
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

static int
bcma_decode_port_rid(device_t dev, device_t child, int type, int rid,
    u_int *port_num, u_int *region_num)
{
	struct bcma_devinfo	*dinfo;
	struct bcma_map		*map;
	struct bcma_sport	*port;

	dinfo = device_get_ivars(child);

	/* Ports are always memory mapped */
	if (type != SYS_RES_MEMORY)
		return (EINVAL);

	/* Search the port list */
	STAILQ_FOREACH(port, &dinfo->corecfg->dev_ports, sp_link) {
		STAILQ_FOREACH(map, &port->sp_maps, m_link) {
			if (map->m_rid != rid)
				continue;

			*port_num = port->sp_num;
			*region_num = map->m_region_num;
			return (0);
		}
	}

	return (ENOENT);
}

static int
bcma_get_port_addr(device_t dev, device_t child, u_int port_num,
	u_int region_num, u_long *addr, u_long *size)
{
	struct bcma_devinfo	*dinfo;
	struct bcma_map		*map;
	struct bcma_sport	*port;

	dinfo = device_get_ivars(child);

	/* Search the port list */
	STAILQ_FOREACH(port, &dinfo->corecfg->dev_ports, sp_link) {
		if (port->sp_num != port_num)
			continue;

		STAILQ_FOREACH(map, &port->sp_maps, m_link) {
			if (map->m_region_num != region_num)
				continue;
			
			/* Not representable */
			if (map->m_base > ULONG_MAX || map->m_size > ULONG_MAX)
				return (ENODEV);

			/* Found! */
			*addr = map->m_base;
			*size = map->m_size;
			return (0);
		}
	}

	return (ENOENT);
}

/**
 * Scan a device enumeration ROM table, adding all discovered cores to the bus.
 * 
 * @param bus The bcma bus.
 * @param erom_res An active resource mapping the EROM core.
 * @param erom_offset Base offset of the EROM core's register mapping.
 */
int
bcma_add_children(device_t bus, struct resource *erom_res, bus_size_t erom_offset)
{
	struct bcma_erom	 erom;
	struct bcma_corecfg	*corecfg;
	struct bcma_devinfo	*dinfo;
	device_t		 child;
	int			 error;
	
	dinfo = NULL;
	corecfg = NULL;

	/* Initialize our reader */
	if ((error = bcma_erom_open(erom_res, erom_offset, &erom)))
		return (error);

	/* Add all cores. */
	while (!error) {
		/* Parse next core */
		error = bcma_erom_parse_corecfg(&erom, &corecfg);
		if (error && error == ENOENT) {
			return (0);
		} else if (error) {
			goto failed;
		}

		/* Allocate per-device bus info */
		dinfo = bcma_alloc_dinfo(bus, corecfg);
		if (dinfo == NULL) {
			error = ENXIO;
			goto failed;
		}

		/* The dinfo instance now owns the corecfg value */
		corecfg = NULL;

		/* Add the child device */
		child = device_add_child_ordered(bus, BHND_PROBE_ORDER_DEFAULT,
		    NULL, -1);
		if (child == NULL) {
			error = ENXIO;
			goto failed;
		}

		/* The child device now owns the dinfo pointer */
		device_set_ivars(child, dinfo);
		dinfo = NULL;
	}

	/* Hit EOF parsing cores? */
	if (error == ENOENT)
		return (0);
	
failed:
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);

	if (corecfg != NULL)
		bcma_free_corecfg(corecfg);

	return (error);
}


static device_method_t bcma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_probe),
	DEVMETHOD(device_attach,		bcma_attach),
	DEVMETHOD(device_detach,		bcma_detach),
	
	/* Bus interface */
	DEVMETHOD(bus_child_deleted,		bcma_child_deleted),
	DEVMETHOD(bus_read_ivar,		bcma_read_ivar),
	DEVMETHOD(bus_write_ivar,		bcma_write_ivar),
	DEVMETHOD(bus_get_resource_list,	bcma_get_resource_list),

	/* BHND interface */
	DEVMETHOD(bhnd_get_port_rid,		bcma_get_port_rid),
	DEVMETHOD(bhnd_decode_port_rid,		bcma_decode_port_rid),
	DEVMETHOD(bhnd_get_port_addr,		bcma_get_port_addr),

	DEVMETHOD_END
};

devclass_t bcma_devclass;
devclass_t bcmab_devclass;

DEFINE_CLASS_1(bcma, bcma_driver, bcma_methods, sizeof(struct bcma_softc), bhnd_driver);
MODULE_VERSION(bcma, 1);
MODULE_DEPEND(bcma, bhnd, 1, 1, 1);
