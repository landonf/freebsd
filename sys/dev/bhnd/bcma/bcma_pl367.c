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

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bcma_pl367reg.h"
#include "bcma_pl367var.h"

/*
 * BCMA ARM PL367 OOB Router driver.
 */

/* Supported device identifiers */
static const struct bhnd_device bcma_pl367_devices[] = {
	BHND_DEVICE(ARM, OOB_ROUTER, NULL, NULL),

	BHND_DEVICE_END
};

static int
bcma_pl367_probe(device_t dev)
{
	const struct bhnd_device	*id;
	int				 error;

	id = bhnd_device_lookup(dev, bcma_pl367_devices,
	     sizeof(bcma_pl367_devices[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bcma_pl367_attach(device_t dev)
{
	struct bcma_pl367_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	/* Allocate register block */
	sc->mem_rid = 0;
	sc->mem_res = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "failed to allocate resources\n");
		return (ENXIO);
	}

	return (0);
}

static int
bcma_pl367_detach(device_t dev)
{
	struct bcma_pl367_softc	*sc;

	sc = device_get_softc(dev);

	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	return (0);
}

static device_method_t bcma_pl367_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcma_pl367_probe),
	DEVMETHOD(device_attach,	bcma_pl367_attach),
	DEVMETHOD(device_detach,	bcma_pl367_detach),

	DEVMETHOD_END
};

DEFINE_CLASS(bcma_pl367, bcma_pl367_driver, bcma_pl367_methods,
    sizeof(struct bcma_pl367_softc));

MODULE_DEPEND(bcma_pl367, bcma, 1, 1, 1);
MODULE_VERSION(bcma_pl367, 1);
