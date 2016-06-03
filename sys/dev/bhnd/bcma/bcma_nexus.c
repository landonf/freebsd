/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <dev/bhnd/bhnd_ids.h>
#include <dev/bhnd/cores/chipc/chipcreg.h>

#include "bcmavar.h"

/*
 * Supports bcma(4) attachment to a nexus bus.
 */

static int	bcma_nexus_attach(device_t);
static int	bcma_nexus_probe(device_t);

static struct resource_spec bcma_nexus_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1,			0,	0 }
};

struct bcma_nexus_softc {
	struct bcma_softc		parent_sc;
	struct bhnd_chipid		bcma_cid;
};

static int
bcma_nexus_probe(device_t dev)
{
	struct bcma_nexus_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	/* Read the ChipCommon info using the hints the kernel
	 * was compiled with. */
	error = bhnd_read_chipid(dev, bcma_nexus_res_spec, 0, &sc->bcma_cid);
	if (error) {
		device_printf(dev, "error %d reading chip ID\n", error);
		return (error);
	}

	if (sc->bcma_cid.chip_type != BHND_CHIPTYPE_BCMA)
		return (ENXIO);

	if ((error = bcma_probe(dev)) > 0) {
		device_printf(dev, "error %d in probe\n", error);
		return (error);
	}

	return (0);
}

static int
bcma_nexus_attach(device_t dev)
{
	struct bcma_nexus_softc	*sc;
	struct resource		*erom_res;
	int error;

	sc = device_get_softc(dev);

	/* Map the EROM resource and enumerate the bus. */
	rid = 0;
	erom_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	    sc->bcma_cid.enum_addr, cid->enum_addr + BCMA_EROM_TABLE_SIZE,
	    BCMA_EROM_TABLE_SIZE, RF_ACTIVE);
	if (erom_res == NULL) {
		device_printf(dev, "failed to allocate EROM resource\n");
		return (ENXIO);
	}

	error = bcma_add_children(dev, erom_res, BCMA_EROM_TABLE_START);
	bus_release_resource(dev, SYS_RES_MEMORY, rid, erom_res);

	if (error)
		return (error);

	return (bcma_attach(dev));
}

static bool
bcma_nexus_is_hw_disabled(device_t dev, device_t child)
{
	return false;
}

static const struct bhnd_chipid *
bcma_nexus_get_chipid(device_t dev, device_t child) {
	struct bcma_nexus_softc	*sc = device_get_softc(dev);
	return (&sc->bcma_cid);
}

static bhnd_attach_type
bcma_nexus_get_attach_type(device_t dev, device_t child)
{
	return (BHND_ATTACH_NATIVE);
}

static int
bcma_nexus_activate_resource(device_t dev, device_t child, int type, int rid,
    struct bhnd_resource *r)
{
	int error;

	/* Always direct */
	if ((error = bus_activate_resource(child, type, rid, r->res)))
		return (error);

	r->direct = true;
	return (0);
}


static device_method_t bcma_nexus_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bcma_nexus_probe),
	DEVMETHOD(device_attach,		bcma_nexus_attach),

	/* bhnd interface */
	DEVMETHOD(bhnd_bus_get_chipid,		bcma_nexus_get_chipid),
	DEVMETHOD(bhnd_bus_activate_resource,	bcma_nexus_activate_resource),
	DEVMETHOD(bhnd_bus_is_hw_disabled,	bcma_nexus_is_hw_disabled),
	DEVMETHOD(bhnd_bus_get_attach_type,	bcma_nexus_get_attach_type),

	DEVMETHOD_END
};

DEFINE_CLASS_1(bhnd, bcma_nexus_driver, bcma_nexus_methods,
    sizeof(struct bcma_nexus_softc), bcma_driver);

DRIVER_MODULE(bcma_nexus, nexus, bcma_nexus_driver, bhnd_devclass, 0, 0);
