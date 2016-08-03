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

#include <dev/cfe/cfe_api.h>
#include <dev/cfe/cfe_error.h>
#include <dev/cfe/cfe_ioctl.h>

#include "bhnd_nvram_if.h"

#include "bhnd_nvramreg.h"
#include "bhnd_nvramvar.h"

/**
 * Default bhnd_nvram driver implementation of DEVICE_PROBE().
 */
int
bhnd_nvram_probe(device_t dev)
{
	device_set_desc(dev, "Broadcom NVRAM");

	/* Refuse wildcard attachments */
	return (BUS_PROBE_NOWILDCARD);
}

/**
 * Default bhnd_nvram driver implementation of DEVICE_ATTACH().
 */
int
bhnd_nvram_attach(device_t dev)
{
	return (0);
}

/**
 * Default bhnd_nvram driver implementation of DEVICE_RESUME().
 */
int
bhnd_nvram_resume(device_t dev)
{
	return (0);
}

/**
 * Default bhnd_nvram driver implementation of DEVICE_SUSPEND().
 */
int
bhnd_nvram_suspend(device_t dev)
{
	return (0);
}

/**
 * Default bhnd_nvram driver implementation of DEVICE_DETACH().
 */
int
bhnd_nvram_detach(device_t dev)
{
	struct bhnd_nvram_softc	*sc;

	sc = device_get_softc(dev);

	bhnd_nvram_fini(&sc->nvram);
	BHND_NVRAM_LOCK_DESTROY(sc);

	return (0);
}

/**
 * Default bhnd_nvram driver implementation of BHND_NVRAM_GETVAR().
 */
static int
bhnd_nvram_getvar_method(device_t dev, const char *name, void *buf, size_t *len)
{
	struct bhnd_nvram_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	BHND_NVRAM_LOCK(sc);
	// TODO
	error = ENODEV;
	BHND_NVRAM_UNLOCK(sc);

	return (error);
}

/**
 * Default bhnd_nvram driver implementation of BHND_NVRAM_SETVAR().
 */
static int
bhnd_nvram_setvar_method(device_t dev, const char *name, const void *buf,
    size_t len)
{
	struct bhnd_nvram_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	BHND_NVRAM_LOCK(sc);
	// TODO
	error = ENODEV;
	BHND_NVRAM_UNLOCK(sc);

	return (error);
}

static device_method_t bhnd_nvram_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_nvram_probe),
	DEVMETHOD(device_attach,	bhnd_nvram_attach),
	DEVMETHOD(device_resume,	bhnd_nvram_resume),
	DEVMETHOD(device_suspend,	bhnd_nvram_suspend),
	DEVMETHOD(device_detach,	bhnd_nvram_detach),

	/* NVRAM interface */
	DEVMETHOD(bhnd_nvram_getvar,	bhnd_nvram_getvar_method),
	DEVMETHOD(bhnd_nvram_setvar,	bhnd_nvram_setvar_method),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram, bhnd_nvram_driver, bhnd_nvram_methods, sizeof(struct bhnd_nvram_softc));
