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

#include "bhnd_pmureg.h"
#include "bhnd_pmuvar.h"

/*
 * Broadcom PMU driver.
 * 
 * Abstract driver for Broadcom PMU devices.
 */

devclass_t bhnd_pmu_devclass;	/**< bhnd(4) PMU device class */

/**
 * Default bhnd_pmu driver implementation of DEVICE_PROBE().
 */
int
bhnd_pmu_probe(device_t dev)
{
	return (BUS_PROBE_DEFAULT);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_ATTACH().
 * 
 * Assumes the PMU register block is mapped via SYS_RES_MEMORY resource
 * with RID 0.
 */
int
bhnd_pmu_attach(device_t dev)
{
	struct bhnd_pmu_softc		*sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = 0;
	
	/* Allocate register block resource */
	sc->pmu_rid = 0;
	sc->pmu = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->pmu_rid,
	    RF_ACTIVE);
	if (sc->pmu == NULL) {
		device_printf(dev, "failed to allocate resources\n");
		return (ENXIO);
	}

	BPMU_LOCK_INIT(sc);

	return (0);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_DETACH().
 */
int
bhnd_pmu_detach(device_t dev)
{
	struct bhnd_pmu_softc	*sc;

	sc = device_get_softc(dev);

	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->pmu_rid,
	    sc->pmu);
	BPMU_LOCK_DESTROY(sc);

	return (0);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_SUSPEND().
 */
int
bhnd_pmu_suspend(device_t dev)
{
	return (0);
}

/**
 * Default bhnd_pmu driver implementation of DEVICE_RESUME().
 */
int
bhnd_pmu_resume(device_t dev)
{
	return (0);
}

static device_method_t bhnd_pmu_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_pmu_probe),
	DEVMETHOD(device_attach,	bhnd_pmu_attach),
	DEVMETHOD(device_detach,	bhnd_pmu_detach),
	DEVMETHOD(device_suspend,	bhnd_pmu_suspend),
	DEVMETHOD(device_resume,	bhnd_pmu_resume),

	/* BHND PMU interface */
	// TODO

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pmu, bhnd_pmu_driver, bhnd_pmu_methods, sizeof(struct bhnd_pmu_softc));
MODULE_VERSION(bhnd_pmu, 1);
