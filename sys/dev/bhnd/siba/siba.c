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

#define SIBA_IS_PORT_VALID(type, port, region)	\
	((type) == BHND_PORT_DEVICE &&		\
	 (port_num) == SIBA_CORE_PORT &&	\
	 region_num == SIBA_CORE_REGION)

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
siba_read_ivar(device_t dev, device_t child, int index, uintptr_t *result)
{
	const struct siba_devinfo *dinfo;
	const struct bhnd_core_info *cfg;
	
	dinfo = device_get_ivars(child);
	cfg = &dinfo->core_id.core_info;
	
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

static u_int
siba_get_port_count(device_t dev, device_t child, bhnd_port_type type)
{
	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev)
		return (BHND_GET_PORT_COUNT(device_get_parent(dev), child, type));

	/* We advertise exactly one device port */
	if (type == BHND_PORT_DEVICE)
		return (1);

	return (0);
}

static u_int
siba_get_region_count(device_t dev, device_t child, bhnd_port_type type,
    u_int port)
{
	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev)
		return (BHND_GET_REGION_COUNT(device_get_parent(dev), child,
		    type, port));

	/* We advertise exactly one port region */
	if (type == BHND_PORT_DEVICE && port == SIBA_CORE_PORT)
		return (1);

	return (0);
}

static int
siba_get_port_rid(device_t dev, device_t child, bhnd_port_type port_type,
    u_int port_num, u_int region_num)
{
	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev)
		return (BHND_GET_PORT_RID(device_get_parent(dev), child,
		    port_type, port_num, region_num));

	if (SIBA_IS_PORT_VALID(port_type, port_num, region_num))
		return (SIBA_CORE_RID);

	/* not found */
	return (-1);
}

static int
siba_decode_port_rid(device_t dev, device_t child, int type, int rid,
    bhnd_port_type *port_type, u_int *port_num, u_int *region_num)
{
	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev) {
		return (BHND_DECODE_PORT_RID(device_get_parent(dev), child,
		    type, rid, port_type, port_num, region_num));
	}
	
	if (type != SYS_RES_MEMORY)
		return (EINVAL);

	/* siba(4) cores only support a single memory RID */
	if (rid != SIBA_CORE_RID)
		return (ENOENT);

	*port_type = BHND_PORT_DEVICE;
	*port_num = SIBA_CORE_PORT;
	*region_num = SIBA_CORE_REGION;
	return (0);
}

static int
siba_get_region_addr(device_t dev, device_t child, bhnd_port_type port_type,
    u_int port_num, u_int region_num, bhnd_addr_t *addr, bhnd_size_t *size)
{
	struct siba_devinfo		*dinfo;
	struct resource_list_entry	*rle;

	/* delegate non-bus-attached devices to our parent */
	if (device_get_parent(child) != dev) {
		return (BHND_GET_REGION_ADDR(device_get_parent(dev), child,
		    port_type, port_num, region_num, addr, size));
	}

	dinfo = device_get_ivars(child);

	/* siba(4) cores only support a single device port region */
	if (!SIBA_IS_PORT_VALID(port_type, port_num, region_num))
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


/**
 * Register all address space mappings for @p di.
 *
 * @param dev The siba bus device.
 * @param di The device info instance on which to register all address
 * space entries.
 * @param r A resource mapping the enumeration table block for @p di.
 */
static int
siba_register_addrspaces(device_t dev, struct siba_devinfo *di,
    struct resource *r)
{
	struct siba_addrspace	*enum_spc;
	uint32_t		 addr;
	uint32_t		 size;
	int			 error;

	/* Register the device address space entries */
	for (uint8_t i = 0; i < di->core_id.num_addrspace; i++) {
		uint32_t	adm;
		u_int		adm_offset;

		/* Determine the register offset */
		adm_offset = siba_admatch_offset(i);
		if (adm_offset == 0) {
		    device_printf(dev, "address region %hhu is unsupported", i);
		    return (ENODEV);
		}

		/* Fetch the address match register value */
		adm = bus_read_4(r, adm_offset);

		/* Skip disabled entries */
		if (adm & SIBA_AM_ADEN)
			continue;
			
		/* Parse the value */
		if ((error = siba_parse_admatch(adm, &addr, &size))) {
			device_printf(dev, "failed to decode address "
			    " match register value 0x%x\n", adm);
			return (error);
		}

		/* Append the region info */
		error = siba_append_dinfo_region(di, BHND_PORT_DEVICE, 0, i,
		    addr, size);
		if (error)
			return (error);
	}

	/* Register agent regions mapped within the 4kb enumeration space
	 * register block */
	enum_spc = STAILQ_FIRST(&di->device_port.sp_maps);

	/* If no enum space mapping, nothing to register */
	if (enum_spc == NULL || enum_spc->sa_space_id != 0) {
		device_printf(dev, "core missing enumeration space\n");
		return (EINVAL);
	}

	/* Sanity check the address region */
	if (enum_spc->sa_base < SIBA_ENUM_ADDR ||
	    enum_spc->sa_base >= SIBA_ENUM_ADDR + SIBA_ENUM_SIZE ||
	    enum_spc->sa_size < SIBA_AGENT_0_OFFSET + SIBA_AGENT_REGION_SIZE)
	{
		device_printf(dev, "core has invalid enumeration space "
		    "(0x%x 0x%x)\n", enum_spc->sa_base, enum_spc->sa_size);
		return (EINVAL);
	}

	/* Register SIBA_AGENT_0 */
	addr = enum_spc->sa_base + SIBA_AGENT_0_OFFSET;
	size = SIBA_AGENT_REGION_SIZE;
	error = siba_append_dinfo_region(di, BHND_PORT_AGENT, 0,
	    enum_spc->sa_space_id, addr, size);
	if (error) {
		device_printf(dev, "failed to register agent 0 region\n");
		return (error);
	}

	/* Register SIBA_AGENT_1 (sonics >= 2.3) */
	if (di->core_id.sonics_rev >= SIBA_IDL_SBREV_2_3) {
		addr = enum_spc->sa_base + SIBA_AGENT_1_OFFSET;
		size = SIBA_AGENT_REGION_SIZE;
		error = siba_append_dinfo_region(di, BHND_PORT_AGENT, 0,
		    enum_spc->sa_space_id, addr, size);
		if (error) {
		device_printf(dev, "failed to register agent 1 region\n");
			return (error);
		}
	}

	return (0);
}

/**
 * Scan the core table and add all valid discovered cores to
 * the bus.
 * 
 * @param dev The siba bus device.
 * @param chipid The chip identifier, if known or if the device
 * does not provide a ChipCommon core. May be NULL otherwise.
 */
int
siba_add_children(device_t dev, const struct bhnd_chipid *chipid)
{
	struct bhnd_chipid	 ccid;
	struct bhnd_core_info	*cores;
	struct siba_devinfo	*dinfo;
	struct resource		*r;
	u_int			 ncores;
	int			 rid;
	int			 error;

	dinfo = NULL;
	cores = NULL;
	r = NULL;

	/* If not provided by our caller, read the chip ID now. */
	if (chipid == NULL) {
		struct resource_spec rs = {
			.rid = 0,
			.type = SYS_RES_MEMORY,
			.flags = RF_ACTIVE,
		};

		error = bhnd_read_chipid(dev, &rs, SIBA_ENUM_ADDR, &ccid);
		if (error) {
			device_printf(dev, "failed to read bus chipid\n");
			return (error);
		}

		chipid = &ccid;
	}
	
	/* Determine the core count */
	ncores = siba_get_ncores(chipid);
	if (ncores == 0) {
		device_printf(dev, "core count unknown for chip ID 0x%hx\n",
		    chipid->chip_id);
		return (ENXIO);
	}

	/* Allocate our temporary core table and enumerate all cores */
	cores = malloc(sizeof(*cores) * ncores, M_BHND, M_WAITOK);
	if (cores == NULL)
		return (ENOMEM);

	/* Add all cores. */
	for (u_int i = 0; i < ncores; i++) {
		struct siba_core_id	 cid;
		device_t		 child;
		uint32_t		 idhigh, idlow;
		u_long			 r_count, r_end, r_start;

		/* Map the core's register block */
		rid = 0;
		r_start = SIBA_CORE_ADDR(i);
		r_count = SIBA_CORE_SIZE;
		r_end = r_start + SIBA_CORE_SIZE - 1;
		r = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid, r_start,
		    r_end, r_count, RF_ACTIVE);
		if (r == NULL) {
			error = ENXIO;
			goto cleanup;
		}

		/* Read the core info */
		idhigh = bus_read_4(r, SB0_REG_ABS(SIBA_R0_IDHIGH));
		idlow = bus_read_4(r, SB0_REG_ABS(SIBA_R0_IDLOW));

		cid = siba_parse_core_id(idhigh, idlow, i, 0);
		cores[i] = cid.core_info;

		/* Determine unit number */
		for (u_int j = 0; j < i; j++) {
			if (cores[j].vendor == cores[i].vendor &&
			    cores[j].device == cores[i].device)
				cores[i].unit++;
		}

		/* Allocate per-device bus info */
		dinfo = siba_alloc_dinfo(dev, &cid);
		if (dinfo == NULL) {
			error = ENXIO;
			goto cleanup;
		}

		/* Register the core's address space(s). */
		if ((error = siba_register_addrspaces(dev, dinfo, r)))
			goto cleanup;

		/* Add the child device */
		child = device_add_child(dev, NULL, -1);
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
				
		/* Release our resource */
		bus_release_resource(dev, SYS_RES_MEMORY, rid, r);
		r = NULL;
	}
	
cleanup:
	if (cores != NULL)
		free(cores, M_BHND);

	if (dinfo != NULL)
		siba_free_dinfo(dinfo);

	if (r != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, rid, r);

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
	DEVMETHOD(bhnd_get_port_count,		siba_get_port_count),
	DEVMETHOD(bhnd_get_region_count,	siba_get_region_count),
	DEVMETHOD(bhnd_get_port_rid,		siba_get_port_rid),
	DEVMETHOD(bhnd_decode_port_rid,		siba_decode_port_rid),
	DEVMETHOD(bhnd_get_region_addr,		siba_get_region_addr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhnd, siba_driver, siba_methods, sizeof(struct siba_softc), bhnd_driver);

MODULE_VERSION(siba, 1);
MODULE_DEPEND(siba, bhnd, 1, 1, 1);
