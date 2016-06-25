/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
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
 * BHND CFE NVRAM driver.
 * 
 * Provides access to device NVRAM via CFE.
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

#define	NVCFE_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "bhnd_cfe nvram lock", MTX_DEF)
#define	NVCFE_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	NVCFE_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	NVCFE_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->mtx, what)
#define	NVCFE_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx)

struct bhnd_nvcfe_softc {
	device_t		 dev;
	struct mtx		 mtx;		/**< nvram mutex */
};

static int
bhnd_nvram_cfe_probe(device_t dev)
{
	device_set_desc(dev, "CFE NVRAM");

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}

static int
bhnd_nvram_cfe_attach(device_t dev)
{
	struct bhnd_nvcfe_softc	*sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Initialize mutex */
	NVCFE_LOCK_INIT(sc);

	return (0);
}

static int
bhnd_nvram_cfe_resume(device_t dev)
{
	return (0);
}

static int
bhnd_nvram_cfe_suspend(device_t dev)
{
	return (0);
}

static int
bhnd_nvram_cfe_detach(device_t dev)
{
	struct bhnd_nvcfe_softc	*sc;

	sc = device_get_softc(dev);

	NVCFE_LOCK_DESTROY(sc);

	return (0);
}

static int
bhnd_nvram_cfe_getvar(device_t dev, const char *name, void *buf, size_t *len)
{
	struct bhnd_nvcfe_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	NVCFE_LOCK(sc);
	// TODO
	error = ENXIO;
	NVCFE_UNLOCK(sc);

	return (error);
}

/**
 * Default bhnd sprom driver implementation of BHND_NVRAM_SETVAR().
 */
static int
bhnd_nvram_cfe_setvar(device_t dev, const char *name, const void *buf,
    size_t len)
{
	struct bhnd_nvcfe_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	NVCFE_LOCK(sc);
	// TODO
	error = ENXIO;
	NVCFE_UNLOCK(sc);

	return (error);
}

static device_method_t bhnd_nvcfe_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_nvram_cfe_probe),
	DEVMETHOD(device_attach,		bhnd_nvram_cfe_attach),
	DEVMETHOD(device_resume,		bhnd_nvram_cfe_resume),
	DEVMETHOD(device_suspend,		bhnd_nvram_cfe_suspend),
	DEVMETHOD(device_detach,		bhnd_nvram_cfe_detach),

	/* NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,		bhnd_nvram_cfe_getvar),
	DEVMETHOD(bhnd_nvram_setvar,		bhnd_nvram_cfe_setvar),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram, bhnd_nvcfe_driver, bhnd_nvcfe_methods, 
    sizeof(struct bhnd_nvcfe_softc));
EARLY_DRIVER_MODULE(bhnd_nvram_cfe, nexus, bhnd_nvcfe_driver,
    bhnd_nvram_devclass, NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_EARLY);

