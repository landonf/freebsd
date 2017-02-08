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

static int	bhnd_sprom_new_provider(device_t dev,
		    struct bhnd_nvram_provider **prov, bus_size_t offset);

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
	struct bhnd_nvram_plane	*plane;
	int			 error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Fetch NVRAM plane from the bus */
	if ((plane = bhnd_get_nvram_plane(dev)) == NULL) {
		device_printf(dev, "missing NVRAM plane\n");
		return (ENXIO);
	}

	sc->plane = bhnd_nvram_plane_retain(plane);

	/* Parse SPROM data and allocate NVRAM provider */
	if ((error = bhnd_sprom_new_provider(dev, &sc->prov, offset)))
		return (error);

	/* Register provider with the NVRAM plane */
	if ((error = bhnd_nvram_plane_set_provider(sc->plane, sc->prov))) {
		device_printf(dev, "failed to map SPROM into NVRAM plane: "
		    "%d\n", error);

		bhnd_nvram_provider_release(sc->prov);
		return (error);
	}

	return (0);
}


/**
 * Allocate a new NVRAM provider instance.
 * 
 * Assumes SPROM is mapped via SYS_RES_MEMORY resource with RID 0.
 *
 * @param	dev	BHND SPROM device.
 * @param[out]	prov	On success, the new NVRAM provider instance.
 * @param	offset	Offset to the SPROM data in RID 0.
 */
static int
bhnd_sprom_new_provider(device_t dev, struct bhnd_nvram_provider **prov,
    bus_size_t offset)
{
	struct bhnd_nvram_store_init_params	 params;
	struct bhnd_nvram_data			*data;
	struct bhnd_nvram_io			*io;
	struct bhnd_resource			*r;
	bus_size_t				 r_size, sprom_size;
	int					 rid;
	int					 error;

	io = NULL;
	r = NULL;
	data = NULL;

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

		error = ENXIO;
		goto cleanup;
	}

	sprom_size = r_size - offset;

	/* Allocate an I/O context for the SPROM parser. All SPROM reads
	 * must be 16-bit aligned */
	io = bhnd_nvram_iores_new(r, offset, sprom_size, sizeof(uint16_t));
	if (io == NULL) {
		error = ENXIO;
		goto cleanup;
	}

	/* Try to parse the data */
	error = bhnd_nvram_data_new(&bhnd_nvram_sprom_class, &data, io);
	if (error) {
		data = NULL;
		goto cleanup;
	}

	/* Attempt to initialize the NVRAM provider instance */
	params = (struct bhnd_nvram_store_init_params) {
		.dev = dev,
		.data = data
	};

	error = bhnd_nvram_provider_new(prov, &bhnd_nvram_store_provider,
	    &params);
	if (error)
		goto cleanup;

cleanup:
	if (io != NULL) {
		/* Clean up I/O context before releasing its backing resource */
		bhnd_nvram_io_free(io);
	}

	if (r != NULL)
		bhnd_release_resource(dev, SYS_RES_MEMORY, rid, r);

	if (data != NULL)
		bhnd_nvram_data_release(data);

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

	// XXX TODO: Disconnect our device from the NVRAM provider
	bhnd_nvram_provider_release(sc->prov);

	return (0);
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_GETVAR().
 */
static int
bhnd_sprom_getvar(device_t dev, const char *name, void *buf, size_t *len,
    bhnd_nvram_type type)
{
//	struct bhnd_sprom_softc	*sc = device_get_softc(dev);

	// XXX TODO
	return (ENOENT);
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_SETVAR().
 */
static int
bhnd_sprom_setvar(device_t dev, const char *name, const void *buf,
    size_t len, bhnd_nvram_type type)
{
//	struct bhnd_sprom_softc	*sc = device_get_softc(dev);

	// XXX TODO
	return (ENOENT);
}

static device_method_t bhnd_sprom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_sprom_probe),
	DEVMETHOD(device_attach,		bhnd_sprom_generic_attach),
	DEVMETHOD(device_resume,		bhnd_sprom_resume),
	DEVMETHOD(device_suspend,		bhnd_sprom_suspend),
	DEVMETHOD(device_detach,		bhnd_sprom_detach),

	/* Legacy NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,		bhnd_sprom_getvar),
	DEVMETHOD(bhnd_nvram_setvar,		bhnd_sprom_setvar),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram_store, bhnd_sprom_driver, bhnd_sprom_methods, sizeof(struct bhnd_sprom_softc));
MODULE_VERSION(bhnd_sprom, 1);
