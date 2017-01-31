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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * BHND SPROM driver.
 * 
 * Abstract driver for memory-mapped SPROM devices.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_nvram_if.h"

#include "bhnd_nvram_io.h"

#include "bhnd_spromvar.h"

static int	bhnd_sprom_new_store(device_t dev,
		    struct bhnd_nvram_store **store, bus_size_t offset);

/**
 * Default bhnd sprom driver implementation of DEVICE_PROBE().
 */
int
bhnd_sprom_probe(device_t dev)
{
	device_set_desc(dev, "SPROM/OTP");

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}

/* Default DEVICE_ATTACH() implementation; assumes a zero offset to the
 * SPROM data */
static int
bhnd_sprom_generic_attach(device_t dev)
{
	return (bhnd_sprom_attach(dev, 0));
}

/**
 * BHND SPROM device attach.
 * 
 * This should be called from DEVICE_ATTACH() with the @p offset to the
 * SPROM data.
 * 
 * Assumes SPROM is mapped via SYS_RES_MEMORY resource with RID 0.
 * 
 * @param dev BHND SPROM device.
 * @param offset Offset to the SPROM data.
 */
int
bhnd_sprom_attach(device_t dev, bus_size_t offset)
{
	struct bhnd_sprom_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Fetch NVRAM plane from the bus */
	if ((sc->plane = bhnd_get_nvram_plane(dev)) == NULL) {
		device_printf(dev, "missing NVRAM plane\n");
		return (ENXIO);
	}

	/* Parse SPROM data and allocate new NVRAM data store */
	if ((error = bhnd_sprom_new_store(dev, &sc->store, offset)))
		return (error);

	// XXX TODO: initialize sc->prov

	/* Map our device into the NVRAM plane */
	error = bhnd_nvram_plane_map_provider(sc->plane, sc->prov, M_WAITOK);
	if (error) {
		device_printf(dev, "failed to map NVRAM store: %d\n", error);
		bhnd_nvram_store_free(sc->store);
		return (error);
	}

	return (0);
}


/**
 * Allocate a new NVRAM store instance.
 * 
 * Assumes SPROM is mapped via SYS_RES_MEMORY resource with RID 0.
 *
 * @param	dev	BHND SPROM device.
 * @param[out]	store	On success, the new NVRAM store instance.
 * @param	offset	Offset to the SPROM data in RID 0.
 */
static int
bhnd_sprom_new_store(device_t dev, struct bhnd_nvram_store **store,
    bus_size_t offset)
{
	struct bhnd_nvram_io	*io;
	struct bhnd_resource	*r;
	bus_size_t		 r_size, sprom_size;
	int			 rid;
	int			 error;

	io = NULL;
	r = NULL;

	/* Allocate SPROM resource */
	rid = 0;
	r = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (r == NULL) {
		device_printf(dev, "failed to allocate resources\n");
		return (ENXIO);
	}

	/* Determine SPROM size */
	r_size = rman_get_size(r->res);
	if (r_size <= offset || (r_size - offset) > BUS_SPACE_MAXSIZE) {
		device_printf(dev, "invalid sprom offset %#jx\n",
		    (uintmax_t)offset);
		bhnd_release_resource(dev, SYS_RES_MEMORY, rid, r);

		return (ENXIO);
	}

	sprom_size = r_size - offset;

	/* Allocate an I/O context for the SPROM parser. All SPROM reads
	 * must be 16-bit aligned */
	io = bhnd_nvram_iores_new(r, offset, sprom_size, sizeof(uint16_t));
	if (io == NULL) {
		bhnd_release_resource(dev, SYS_RES_MEMORY, rid, r);
		return (ENXIO);
	}

	/* Attempt to initialize NVRAM data store */
	error = bhnd_nvram_store_parse_new(store, io, &bhnd_nvram_sprom_class);

	/* Clean up I/O context before releasing the backing resource */
	bhnd_nvram_io_free(io);
	bhnd_release_resource(dev, SYS_RES_MEMORY, rid, r);

	return (error);
}

/**
 * Default bhnd_sprom implementation of DEVICE_RESUME().
 */
int
bhnd_sprom_resume(device_t dev)
{
	return (0);
}

/**
 * Default bhnd sprom driver implementation of DEVICE_SUSPEND().
 */
int
bhnd_sprom_suspend(device_t dev)
{
	return (0);
}

/**
 * Default bhnd sprom driver implementation of DEVICE_DETACH().
 */
int
bhnd_sprom_detach(device_t dev)
{
	struct bhnd_sprom_softc	*sc;

	sc = device_get_softc(dev);

	/* Disconnect our device from the NVRAM plane */
	bhnd_nvram_plane_unmap_provider(sc->plane, sc->prov);

	/* Clean up backing NVRAM store */
	bhnd_nvram_store_free(sc->store);

	return (0);
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_GETVAR().
 */
static int
bhnd_sprom_getvar(device_t dev, const char *name, void *buf, size_t *len,
    bhnd_nvram_type type)
{
	struct bhnd_sprom_softc	*sc = device_get_softc(dev);

	return (bhnd_nvram_store_getvar(sc->store, name, buf, len, type));
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_SETVAR().
 */
static int
bhnd_sprom_setvar(device_t dev, const char *name, const void *buf,
    size_t len, bhnd_nvram_type type)
{
	struct bhnd_sprom_softc	*sc = device_get_softc(dev);

	return (bhnd_nvram_store_setvar(sc->store, name, buf, len, type));
}

static device_method_t bhnd_sprom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_sprom_probe),
	DEVMETHOD(device_attach,		bhnd_sprom_generic_attach),
	DEVMETHOD(device_resume,		bhnd_sprom_resume),
	DEVMETHOD(device_suspend,		bhnd_sprom_suspend),
	DEVMETHOD(device_detach,		bhnd_sprom_detach),

	/* NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,		bhnd_sprom_getvar),
	DEVMETHOD(bhnd_nvram_setvar,		bhnd_sprom_setvar),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram_store, bhnd_sprom_driver, bhnd_sprom_methods, sizeof(struct bhnd_sprom_softc));
MODULE_VERSION(bhnd_sprom, 1);
