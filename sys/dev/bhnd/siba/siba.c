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

int
siba_probe(device_t dev)
{
	device_set_desc(dev, "SIBA BHND bus");
	return (BUS_PROBE_NOWILDCARD);
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

/**
 * Determine the number of cores available on the bus.
 * 
 * Some devices require a hardcoded core count:
 * - Earlier ChipCommon revisions (chip_rev <= 4) did not include a core count.
 * - Earlier siba(4) devices did not include a ChipCommon core at all.
 */
static uint8_t
siba_get_ncores(const struct bhnd_chipid *chipid) {
	/* Use the real count if available. */
	if (chipid->ncores > 0)
		return (chipid->ncores);

	/*
	 * The magic constants below were copied from the previous
	 * siba driver implementation; their correctness has
	 * not been verified.
	 */
	switch (chipid->chip_id) {
		case 0x4401:	/* BCM4401 PCI ID? */
		case BHND_CHIPID_BCM4402:
			return (3);

		case 0x4301:	/* BCM4031 PCI ID? */
		case 0x4307:	/* BCM4307 PCI ID? */
			return (5);
			
		case BHND_CHIPID_BCM4306:
			return (6);
			
		case BHND_CHIPID_BCM5365:
			return (7);

		case 0x4310:	/* ??? */
			return (8);

		case 0x4610:	/* BCM4610 Sentry5 PCI Card? */
		case BHND_CHIPID_BCM4704:
		case BHND_CHIPID_BCM4710:
			return (9);

		default:
			return (0);
	}
}

// TODO lift out into siba_subr
// TODO additional vendor codes
static uint16_t
siba_bhnd_mfgid(uint16_t vendor)
{
	switch (vendor) {
	case SIBA_VEND_BCM:
		return (BHND_MFGID_BCM);
	default:
		return (BHND_MFGID_INVALID);
	}
}

static int
siba_read_core_table(kobj_class_t driver, device_t dev,
    const struct bhnd_chipid *chipid, 
    void *ioh,
    const struct bhnd_iosw *iosw,
    struct bhnd_core_info **core_table,
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
		device_printf(dev, "core count unknown for chip ID 0x%hx\n",
		    chipid->chip_id);
		return (ENXIO);
	}

	/* Allocate output table */
	cores = malloc(sizeof(*cores) * ncores, M_BHND, M_WAITOK);
	if (cores == NULL)
		return (ENOMEM);

	/* Iterate the bus */
	// TODO lift out into siba_subr
	for (uint8_t i = 0; i < ncores; i++) {
		struct bhnd_core_info	*ci;
		bhnd_addr_t		 addr;
		uint32_t		 idH;

		/* Fetch ID register */
		ci = &cores[i];
		addr = SIBA_CORE_ADDR(i);
		idH = iosw->read4(ioh, addr + SIBA_IDHIGH);

		/* Extract core info */
		*ci = (struct bhnd_core_info) {
			.vendor	= SIBA_REG_GET(idH, IDH_VENDOR),
			.device	= SIBA_REG_GET(idH, IDH_DEVICE),
			.hwrev	= SIBA_CORE_REV(idH),
			.core_id = i,
			.unit	= 0
		};
		
		/* Map vendor to bhnd mfgid */
		ci->vendor = siba_bhnd_mfgid(ci->vendor);

		/* Determine unit number */
		for (uint8_t j = 0; j < i; j++) {
			if (cores[j].vendor == ci->vendor &&
			    cores[j].device == ci->device)
			{
				ci->unit++;
			}
		}

		// TODO
		device_printf(dev,
		    "found 0x%hx/0x%hx (rev %hu) unit %d %s %s\n",
		    ci->vendor, ci->device, ci->hwrev, ci->unit,
		    bhnd_vendor_name(ci->vendor),
		    bhnd_core_name(ci->vendor, ci->device));
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
	case BHND_IVAR_REVID:
		*result = cfg->hwrev;
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
	// TODO
	return (ENXIO);
}

static int
siba_decode_port_rid(device_t dev, device_t child, int type, int rid,
    u_int *port_num, u_int *region_num)
{
	// TODO
	return (ENXIO);
}

static int
siba_get_port_addr(device_t dev, device_t child, u_int port_num,
	u_int region_num, bhnd_addr_t *addr, bhnd_size_t *size)
{
	// TODO
	return (ENXIO);
}

/**
 * Scan the core table and add all valid discovered cores to
 * the bus.
 * 
 * @param bus The siba bus.
 */
int
siba_add_children(device_t bus)
{
	// TODO
	return (ENXIO);
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
