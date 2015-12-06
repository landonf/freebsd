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

#include <dev/bhnd/cores/bhnd_chipcreg.h>

#include "sibareg.h"
#include "sibavar.h"

/* The port/region/rid triplet used for all siba(4) cores. */
#define	SIBA_CORE_PORT		0	/**< fixed register block port. */
#define	SIBA_CORE_REGION	0	/**< fixed register block region. */
#define SIBA_CORE_RID		1	/**< fixed register resource-ID */

int
siba_probe(device_t dev)
{
	device_set_desc(dev, "SIBA BHND bus");
	return (BUS_PROBE_DEFAULT);
}

int
siba_attach(device_t dev)
{
	/* Bus' generic attach will probe and attach any enumerated children */
	return (bus_generic_attach(dev));
}

int
siba_detach(device_t dev)
{
	return (bus_generic_detach(dev));
}

static int
siba_read_core_table(kobj_class_t driver, const struct bhnd_chipid *chipid, 
    const struct bhnd_bus_ctx *bus, struct bhnd_core_info **core_table,
    u_int *num_cores)
{
	struct bhnd_core_info	*cores;
	uint8_t			 ncores;

	/* We can only handle siba(4) */
	if (chipid->chip_type != BHND_CHIPTYPE_SIBA)
		return (ENXIO);

	/* Determine the core count */
	ncores = siba_get_ncores(chipid);
	if (ncores == 0) {
		device_printf(bus->dev,
		    "core count unknown for chip ID 0x%hx\n", chipid->chip_id);
		return (ENXIO);
	}

	/* Allocate output table */
	cores = malloc(sizeof(*cores) * ncores, M_BHND, M_WAITOK);
	if (cores == NULL)
		return (ENOMEM);

	/* Iterate the bus */
	for (uint8_t i = 0; i < ncores; i++) {
		struct bhnd_core_info	*ci;
		bhnd_addr_t		 addr;
		uint32_t		 idH;

		/* Fetch ID register */
		ci = &cores[i];
		addr = SIBA_CORE_ADDR(i);
		idH = bus->ops->read4(bus->context, addr + SIBA_IDHIGH);

		/* Extract basic core info */
		*ci = siba_parse_core_info(idH, i, 0);

		/* Determine unit number */
		for (uint8_t j = 0; j < i; j++) {
			if (cores[j].vendor == ci->vendor &&
			    cores[j].device == ci->device)
			{
				ci->unit++;
			}
		}
	}

	*core_table = cores;
	*num_cores = ncores;

	return (0);
}

static int
siba_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	const struct siba_devinfo *dinfo;
	const struct bhnd_core_info *cfg;
	
	dinfo = device_get_ivars(child);
	cfg = &dinfo->core_info;
	
	switch (index) {
	case BHND_IVAR_VENDOR:
		*result = cfg->vendor;
		return (0);
	case BHND_IVAR_DEVICE:
		*result = cfg->device;
		return (0);
	case BHND_IVAR_HWREV:
		*result = cfg->hwrev;
		return (0);
	case BHND_IVAR_DEVICE_CLASS:
		*result = bhnd_core_class(cfg);
		return (0);
	case BHND_IVAR_VENDOR_NAME:
		*result = (uintptr_t) bhnd_vendor_name(cfg->vendor);
		return (0);
	case BHND_IVAR_DEVICE_NAME:
		*result = (uintptr_t) bhnd_core_name(cfg);
		return (0);
	case BHND_IVAR_CORE_INDEX:
		*result = cfg->core_id;
		return (0);
	case BHND_IVAR_CORE_UNIT:
		*result = cfg->unit;
		return (0);
	default:
		return (ENOENT);
	}
}

static int
siba_write_ivar(device_t dev, device_t child, int index, uintptr_t value)
{
	switch (index) {
	case BHND_IVAR_VENDOR:
	case BHND_IVAR_DEVICE:
	case BHND_IVAR_HWREV:
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
siba_child_deleted(device_t dev, device_t child)
{
	struct siba_devinfo *dinfo = device_get_ivars(child);
	if (dinfo != NULL)
		siba_free_dinfo(dinfo);
}

static struct resource_list *
siba_get_resource_list(device_t dev, device_t child)
{
	struct siba_devinfo *dinfo = device_get_ivars(child);
	return (&dinfo->resources);
}

static int
siba_get_port_rid(device_t dev, device_t child, u_int port_num, u_int
    region_num)
{
	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev) {
		return (BHND_GET_PORT_RID(device_get_parent(dev), child,
		    port_num, region_num));
	}

	/* siba(4) cores only support a single port and region */
	if (port_num != SIBA_CORE_PORT || region_num != SIBA_CORE_REGION)
		return (-1);

	/* found */
	return (SIBA_CORE_RID);
}

static int
siba_decode_port_rid(device_t dev, device_t child, int type, int rid,
    u_int *port_num, u_int *region_num)
{
	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev) {
		return (BHND_DECODE_PORT_RID(device_get_parent(dev), child,
		    type, rid, port_num, region_num));
	}
	
	if (type != SYS_RES_MEMORY)
		return (EINVAL);

	/* siba(4) cores only support a single memory RID */
	if (rid != SIBA_CORE_RID)
		return (ENOENT);


	*port_num = SIBA_CORE_PORT;
	*region_num = SIBA_CORE_REGION;
	return (0);
}

static int
siba_get_port_addr(device_t dev, device_t child, u_int port_num,
	u_int region_num, bhnd_addr_t *addr, bhnd_size_t *size)
{
	struct siba_devinfo		*dinfo;
	struct resource_list_entry	*rle;

	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev) {
		return (BHND_GET_PORT_ADDR(device_get_parent(dev), child,
		    port_num, region_num, addr, size));
	}

	dinfo = device_get_ivars(child);

	/* siba(4) cores only support a single port and region */
	if (port_num != SIBA_CORE_PORT || region_num != SIBA_CORE_REGION)
		return (ENOENT);
	
	/* fetch the port addr/size from the resource list */
	rle = resource_list_find(&dinfo->resources, SYS_RES_MEMORY,
	    SIBA_CORE_RID);
	if (rle == NULL)
		return (ENOENT);

	*addr = rle->start;
	*size = rle->count;
	return (0);
}

/*
 * Resource-based bus i/o implementation for siba_add_children().
 * Automatically maps/unmaps core-sized resources to meet bus I/O
 * requests.
 * 
 * We don't bother caching the allocated core resource across requests; our
 * enumeration code only performs a single read from each core.
 */
static uint32_t
siba_enum_read4(void *handle, bhnd_addr_t addr)
{
	struct resource	*r;
	device_t	 dev;
	uint32_t	 value;
	u_long		 start, end, count, offset;
	int		 rid;
	
	dev = (device_t) handle;

	/* Determine core-aligned resource range */
	start = addr - (addr % SIBA_CORE_SIZE);
	count = SIBA_CORE_SIZE;
	end = start + count - 1;

	/* The 4 byte request may be impossible to fill */
	offset = addr - start;
	if ((SIBA_CORE_SIZE - offset) < sizeof(uint32_t))
		return (UINT32_MAX);

	/* Allocate resource and perform read */
	rid = 0;
	r = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid, start, end, count,
	    RF_ACTIVE);
	if (r == NULL) {
		device_printf(dev, "failed allocation of siba enum resource\n");
		return (UINT32_MAX);
	}

	value = bus_read_4(r, offset);

	/* Clean up */
	bus_release_resource(dev, SYS_RES_MEMORY, rid, r);

	return (value);
}

/**
 * Scan the core table and add all valid discovered cores to
 * the bus.
 * 
 * @param bus The siba bus.
 * @param chipid The chip identifier, if known or if the device
 * does not provide a ChipCommon core. May be NULL otherwise.
 */
int
siba_add_children(device_t bus, const struct bhnd_chipid *chipid)
{
	struct bhnd_bus_ctx	 bus_ctx;
	struct bhnd_chipid	 ccid;
	struct bhnd_core_info	*cores;
	struct siba_devinfo	*dinfo;
	device_t		 child;
	u_int			 num_cores;
	int			 error;
	
	dinfo = NULL;
	cores = NULL;

	/* Initialize bus direct I/O context */
	bus_ctx = (struct bhnd_bus_ctx) {
		.dev = bus,
		.context = bus,
		.ops = &(struct bhnd_bus_ops) {
			.read4 = &siba_enum_read4,
			.write4 = NULL
		}
	};

	/* If not provided by our caller, read the chip ID now. */
	if (chipid == NULL) {
		struct resource_spec rs = {
			.rid = 0,
			.type = SYS_RES_MEMORY,
			.flags = RF_ACTIVE,
		};

		error = bhnd_read_chipid(bus, &rs, SIBA_ENUM_ADDR, &ccid);
		if (error) {
			device_printf(bus, "failed to read bus chipid\n");
			return (error);
		}

		chipid = &ccid;
	}

	/* Parse the core table */
	error = BHND_READ_CORE_TABLE(device_get_driver(bus), chipid, &bus_ctx,
	    &cores, &num_cores);
	if (error)
		goto cleanup;

	/* Attach our null_chipc device if no ChipCommon core is available. */
	if (bhnd_find_core(cores, num_cores, BHND_DEVCLASS_CC) == NULL) {
		// TODO
		device_printf(bus, "null_chipc\n");
	}

	/* Add all cores. */
	for (u_int i = 0; i < num_cores; i++) {
		/* Allocate per-device bus info */
		dinfo = siba_alloc_dinfo(bus, &cores[i]);
		if (dinfo == NULL) {
			error = ENXIO;
			goto cleanup;
		}

		/* Populate the resource list */
		resource_list_add(&dinfo->resources, SYS_RES_MEMORY,
		    SIBA_CORE_RID, SIBA_CORE_ADDR(i),
		    SIBA_CORE_ADDR(i) + SIBA_CORE_SIZE - 1, SIBA_CORE_SIZE);

		/* Add the child device */
		child = device_add_child(bus, NULL, -1);
		if (child == NULL) {
			error = ENXIO;
			goto cleanup;
		}

		/* The child device now owns the dinfo pointer */
		device_set_ivars(child, dinfo);
		dinfo = NULL;

		/* If pins are floating or the hardware is otherwise
		 * unpopulated, the device shouldn't be used. */
		if (bhnd_is_hw_disabled(child))
			device_disable(child);
	}
	
cleanup:
	if (dinfo != NULL)
		siba_free_dinfo(dinfo);

	if (cores != NULL)
		free(cores, M_BHND);

	return (error);
}

static device_method_t siba_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			siba_probe),
	DEVMETHOD(device_attach,		siba_attach),
	DEVMETHOD(device_detach,		siba_detach),
	
	/* Bus interface */
	DEVMETHOD(bus_child_deleted,		siba_child_deleted),
	DEVMETHOD(bus_read_ivar,		siba_read_ivar),
	DEVMETHOD(bus_write_ivar,		siba_write_ivar),
	DEVMETHOD(bus_get_resource_list,	siba_get_resource_list),

	/* BHND interface */
	DEVMETHOD(bhnd_read_core_table,		siba_read_core_table),
	DEVMETHOD(bhnd_get_port_rid,		siba_get_port_rid),
	DEVMETHOD(bhnd_decode_port_rid,		siba_decode_port_rid),
	DEVMETHOD(bhnd_get_port_addr,		siba_get_port_addr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhnd, siba_driver, siba_methods, sizeof(struct siba_softc), bhnd_driver);

MODULE_VERSION(siba, 1);
MODULE_DEPEND(siba, bhnd, 1, 1, 1);
