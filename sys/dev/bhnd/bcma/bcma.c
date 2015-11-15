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

#include <dev/bhnd/bhnd_private.h>

#include "bcmavar.h"
#include "bcma_private.h"

#include "bcma_eromreg.h"
#include "bcma_eromvar.h"

/** BMCA per-instance state */
struct bcma_softc {
};

static int		 bcma_devinfo_fill_port_regions(device_t bus,
			     struct bcma_erom *erom, struct bcma_devinfo *dinfo,
			     bcma_pid_t port_num, bcma_sport_type port_type);

int			 bcma_next_core_unit(struct bcma_erom_core *cores,
			     u_int core_index);

static int		 bcma_next_child_devinfo(device_t bus, 
			     struct bcma_erom *erom, u_int core_index,
			     int core_unit, struct bcma_devinfo **result);

int
bcma_generic_probe(device_t dev)
{
	device_set_desc(dev, "BCMA backplane");
	return (BUS_PROBE_NOWILDCARD);
}

int
bcma_generic_attach(device_t dev)
{
	/* Add our child devices */
	// TODO
	// BHND_ENUMERATE_CHILDREN(dev, dev);

	/* Bus' generic attach will probe and attach the enumerated children */
	return (bus_generic_attach(dev));
}

int
bcma_generic_detach(device_t dev)
{
	return (bus_generic_detach(dev));
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
	case BHND_IVAR_CORE_UNIT:
		*result = cfg->core_unit;
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
	case BHND_IVAR_CORE_UNIT:
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
 * Register all MMIO region descriptors for the given slave port.
 * 
 * @param bus The BCMA bus.
 * @param erom EROM reader.
 * @param dinfo Device info to be populated with the scanned port regions.
 * @param port_num Port index for which regions will be parsed.
 * @param port_type The port region type to be parsed.
 * @param[out] offset The offset at which to perform parsing. On success, this
 * will be updated to point to the next EROM table entry.
 */
static int 
bcma_devinfo_fill_port_regions(device_t bus,
    struct bcma_erom *erom, struct bcma_devinfo *dinfo,
    bcma_pid_t port_num, bcma_sport_type port_type)
{
	struct bcma_sport	*sport;
	struct bcma_sport_list	*sports;
	bus_size_t		 entry_offset;
	int			 error;
	uint8_t			 req_region_type;
	bool			 skip_registration;

	error = 0;
	skip_registration = false;

	/* Allocate a new port descriptor */
	sport = bcma_alloc_sport(port_num, port_type);
	if (sport == NULL)
		return (ENOMEM);
	
	/* Determine the required region type for our region descriptors */
	switch (port_type) {
	case BCMA_SPORT_TYPE_DEVICE:
		req_region_type = BCMA_EROM_REGION_TYPE_DEVICE;
		sports = &dinfo->cfg.dports;
		break;
	case BCMA_SPORT_TYPE_BRIDGE:
		req_region_type = BCMA_EROM_REGION_TYPE_BRIDGE;
		sports = &dinfo->cfg.dports;
		break;
	case BCMA_SPORT_TYPE_SWRAP:
		req_region_type = BCMA_EROM_REGION_TYPE_SWRAP;
		sports = &dinfo->cfg.wports;
		break;
	case BCMA_SPORT_TYPE_MWRAP:
		req_region_type = BCMA_EROM_REGION_TYPE_MWRAP;
		sports = &dinfo->cfg.wports;
		break;
	}

	/* Read all address regions defined for this port */
	for (bcma_rmid_t region_num = 0;; region_num++) {
		struct bcma_map			*map;
		struct bcma_erom_sport_region	 spr;
		bcma_addr_t			 end;

		/* No valid port definition should come anywhere near
		 * BCMA_RMID_MAX. */
		if (region_num == BCMA_RMID_MAX) {
			device_printf(bus,
			    "core%u %s%u: region count reached upper limit of %u\n",
			    dinfo->cfg.core_index,
			    bcma_port_type_name(port_type),
			    port_num, BCMA_RMID_MAX);

			error = EINVAL;
			goto cleanup;
		}

		/* Parse the next region entry. */
		entry_offset = bcma_erom_tell(erom);
		error = bcma_erom_parse_sport_region(erom, &spr);
		if (error && error != ENOENT) {
			device_printf(bus,
			    "core%u %s%u.%u: invalid slave port address region\n",
			    dinfo->cfg.core_index,
			    bcma_port_type_name(port_type),
			    port_num, region_num);
			goto cleanup;
		}

		/* ENOENT signals no further region entries */
		if (error == ENOENT) {
			/* No further entries */
			error = 0;
			break;
		} 
		
		/* A region or type mismatch also signals no further region
		 * entries */
		if (spr.port_num != port_num ||
		    spr.port_type != req_region_type)
		{
			/* We don't want to consume this entry */
			bcma_erom_seek(erom, entry_offset);

			error = 0;
			goto cleanup;
		}

		/*
		 * Create the map entry. 
		 */
		map = malloc(sizeof(struct bcma_map), M_BHND, M_WAITOK);
		if (map == NULL) {
			error = ENOMEM;
			goto cleanup;
		}

		map->m_region_num = region_num;
		map->m_base = spr.base_addr;
		map->m_size = spr.size;
		map->m_rid = -1;

		/*
		 * Create the corresponding device resource list entry.
		 * 
		 * We necessarily skip registration of the region in the 
		 * per-device resource_list if the memory range is not
		 * representable using rman/resource API's u_long address
		 * type.
		 */
		end = map->m_base + spr.size;
		if (map->m_base <= ULONG_MAX && end <= ULONG_MAX) {
			map->m_rid = resource_list_add_next(&dinfo->resources,
			    SYS_RES_MEMORY, spr.base_addr, end, spr.size);
		} else if (bootverbose) {
			device_printf(bus,
				"core%u %s%u.%u: region %llx-%llx extends "
				"beyond supported addressable range\n",
				dinfo->cfg.core_index,
				bcma_port_type_name(port_type),
				port_num, region_num,
				(unsigned long long) map->m_base,
				(unsigned long long) end);
		}

		/* Add the region map to the port */
		STAILQ_INSERT_TAIL(&sport->sp_maps, map, m_link);
		sport->sp_num_maps++;
	}

cleanup:
	/* Append the new port descriptor on success, or deallocate the
	 * partially parsed descriptor on failure. */
	if (error == 0) {
		STAILQ_INSERT_TAIL(sports, sport, sp_link);
	} else if (sport != NULL) {
		bcma_free_sport(sport);
	}

	return error;
}


/**
 * Determine the next available unit number for the given core.
 * 
 * @param cores All cores on in the EROM table, in their original order.
 * @param core_index Core for which the unit is to be determined.
 * 
 * @retval 0 success
 * @retval non-zero an error occured fetching the device list
 */
int
bcma_next_core_unit(struct bcma_erom_core *cores, u_int core_index)
{
	struct bcma_erom_core	*core;
	int			 unit;
	
	core = &cores[core_index];

	/* Determine the next unit number */
	unit = 0;
	for (u_int i = 0; i < core_index; i++) {
		if (cores[i].vendor == core->vendor &&
		    cores[i].device == core->device)
			unit += 1;
	}

	return (unit);
}

/**
 * Parse the next child devinfo entry from the EROM table.
 * 
 * @param bus The BCMA bus.
 * @param erom EROM reader.
 * @param core_index The index of the core being parsed.
 * @param[out] result On success, the core's device info. The caller inherits
 * ownership of this allocation.
 * 
 * @return If successful, returns 0. If the end of the EROM table is hit,
 * ENOENT will be returned. On error, returns a non-zero error value.
 */
static int
bcma_next_child_devinfo(device_t bus, struct bcma_erom *erom, u_int core_index,
    int core_unit, struct bcma_devinfo **result)
{
	struct bcma_devinfo	*dinfo;
	struct bcma_erom_core	 core;
	bcma_sport_type		 first_port_type;
	int			 error;

	dinfo = NULL;

	/* Parse the next core entry */
	if ((error = bcma_erom_parse_core(erom, &core)))
		return (error);

	/* Allocate our device info */
	dinfo = bcma_alloc_dinfo(core_index, core_unit, core.vendor,
	    core.device, core.rev);
	if (dinfo == NULL)
		return (ENOMEM);
	
	/* These are 5-bit values in the EROM table, and should never be able
	 * to overflow BCMA_PID_MAX. */
	KASSERT(core.num_mport <= BCMA_PID_MAX, ("unsupported mport count"));
	KASSERT(core.num_dport <= BCMA_PID_MAX, ("unsupported dport count"));
	KASSERT(core.num_mwrap + core.num_swrap <= BCMA_PID_MAX,
	    ("unsupported wport count"));

	dinfo->cfg.num_dports = core.num_dport;
	dinfo->cfg.num_mports = core.num_mport;
	dinfo->cfg.num_wports = core.num_mwrap + core.num_swrap;

	if (bootverbose) {
		device_printf(bus, 
			    "core%u: %s %s (cid=%hx, rev=%hhu, unit=%d)\n",
			    core_index,
			    bhnd_vendor_name(dinfo->cfg.vendor),
			    bhnd_core_name(dinfo->cfg.vendor, dinfo->cfg.device), 
			    dinfo->cfg.device, dinfo->cfg.revid,
			    dinfo->cfg.core_unit);
	}

	/* Parse Master Port Descriptors */
	for (uint8_t i = 0; i < core.num_mport; i++) {
		struct bcma_mport	*mport;
		struct bcma_erom_mport	 mpd;
	
		/* Parse the master port descriptor */
		error = bcma_erom_parse_mport(erom, &mpd);
		if (error)
			goto failed;

		/* Initialize a new bus mport structure */
		mport = malloc(sizeof(struct bcma_mport), M_BHND, M_WAITOK);
		if (mport == NULL) {
			error = ENOMEM;
			goto failed;
		}
		
		mport->mp_vid = mpd.port_vid;
		mport->mp_num = mpd.port_num;

		/* Update dinfo */
		STAILQ_INSERT_TAIL(&dinfo->cfg.mports, mport, mp_link);
	}
	

	/*
	 * Determine whether this is a bridge device; if so, we can
	 * expect the first sequence of address region descriptors to
	 * be of EROM_REGION_TYPE_BRIDGE instead of
	 * BCMA_EROM_REGION_TYPE_DEVICE.
	 * 
	 * It's unclear whether this is the correct mechanism by which we
	 * should detect/handle bridge devices, but this approach matches
	 * that of (some of) Broadcom's published drivers.
	 */
	if (core.num_dport > 0) {
		uint32_t entry;

		if ((error = bcma_erom_peek32(erom, &entry)))
			goto failed;

		if (BCMA_EROM_ENTRY_IS(entry, REGION) && 
		    BCMA_EROM_GET_ATTR(entry, REGION_TYPE) == BCMA_EROM_REGION_TYPE_BRIDGE)
		{
			first_port_type = BCMA_SPORT_TYPE_BRIDGE;
		} else {
			first_port_type = BCMA_SPORT_TYPE_DEVICE;
		}
	}
	
	/* Device/bridge port descriptors */
	for (uint8_t sp_num = 0; sp_num < core.num_dport; sp_num++) {
		error = bcma_devinfo_fill_port_regions(bus, erom, dinfo, sp_num,
		    first_port_type);

		if (error)
			goto failed;
	}

	/* Wrapper (aka device management) descriptors (for master ports). */
	for (uint8_t sp_num = 0; sp_num < core.num_mwrap; sp_num++) {
		error = bcma_devinfo_fill_port_regions(bus, erom, dinfo, sp_num,
		    BCMA_SPORT_TYPE_MWRAP);

		if (error)
			goto failed;
	}

	
	/* Wrapper (aka device management) descriptors (for slave ports). */	
	for (uint8_t i = 0; i < core.num_swrap; i++) {
		/* Slave wrapper ports are not numbered distinctly from master
		 * wrapper ports. */
		uint8_t sp_num = core.num_mwrap + i;
		error = bcma_devinfo_fill_port_regions(bus, erom, dinfo, sp_num,
		    BCMA_SPORT_TYPE_SWRAP);

		if (error)
			goto failed;
	}

	*result = dinfo;
	return (0);
	
failed:
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);

	return error;
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
	struct bcma_erom_core	*cores;
	struct bcma_devinfo	*dinfo;
	device_t		 child;
	u_int			 num_cores;
	int			 error;
	
	dinfo = NULL;
	cores = NULL;

	/* Initialize our reader */
	if ((error = bcma_erom_open(erom_res, erom_offset, &erom)))
		return (error);

	/* Fetch the full table of core descriptors; used to generate unit
	 * numbers for each core. */
	if ((error = bcma_erom_get_cores(&erom, &cores, &num_cores))) {
		device_printf(bus, "failed to read core table: %d\n", error);
		return (error);
	}

	/* Parse per-core descriptors */
	for (u_int core_index = 0; core_index < num_cores; core_index++) 
	{
		int core_unit;

		/* Determine the core's unit number */
		core_unit = bcma_next_core_unit(cores, core_index);
		
		/* Generate the devinfo structure */
		error = bcma_next_child_devinfo(bus, &erom, core_index,
		    core_unit, &dinfo);
		if (error)
			goto failed;

		/* Add the child device */
		child = device_add_child_ordered(bus, BHND_PROBE_ORDER_DEFAULT, NULL, -1);
		if (child == NULL) {
			error = ENXIO;
			goto failed;
		}

		/* The child device now owns the dinfo pointer */
		device_set_ivars(child, dinfo);
		dinfo = NULL;
	}

	return (0);
	
failed:
	if (dinfo != NULL)
		bcma_free_dinfo(dinfo);
	
	if (cores != NULL)
		free(cores, M_BHND);

	return (error);
}


static device_method_t bcma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_generic_probe),
	DEVMETHOD(device_attach,		bcma_generic_attach),
	DEVMETHOD(device_detach,		bcma_generic_detach),
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),
	DEVMETHOD(device_suspend,		bus_generic_suspend), // TODO
	DEVMETHOD(device_resume,		bus_generic_resume), // TODO
	
	/* Bus interface */
	DEVMETHOD(bus_child_deleted,		bcma_generic_child_deleted),
	DEVMETHOD(bus_print_child,		bhnd_generic_print_child),
	DEVMETHOD(bus_probe_nomatch,		bhnd_generic_probe_nomatch),
	DEVMETHOD(bus_read_ivar,		bcma_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,		bcma_generic_write_ivar),

	DEVMETHOD(bus_get_resource_list,	bcma_generic_get_resource_list),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_delete_resource,		bus_generic_rl_delete_resource),
	DEVMETHOD(bus_alloc_resource,		bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_adjust_resource,		bus_generic_adjust_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),

	/* BHND interface */
	DEVMETHOD(bhnd_get_port_rid,		bcma_generic_get_port_rid),
	DEVMETHOD(bhnd_alloc_resource,		bhnd_generic_alloc_bhnd_resource),
	DEVMETHOD(bhnd_release_resource,	bhnd_generic_release_bhnd_resource),
	DEVMETHOD(bhnd_activate_resource,	bhnd_generic_activate_bhnd_resource),
	DEVMETHOD(bhnd_deactivate_resource,	bhnd_generic_deactivate_bhnd_resource),

	DEVMETHOD_END
};

devclass_t bcma_devclass;
devclass_t bcmab_devclass;

DEFINE_CLASS_0(bcma, bcma_driver, bcma_methods, sizeof(struct bcma_softc));

MODULE_VERSION(bcma, 1);
MODULE_DEPEND(bcma, bhnd, 1, 1, 1);
DRIVER_MODULE(bcma, bcmab, bcma_driver, bcma_devclass, NULL, NULL);
