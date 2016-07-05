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
#include <sys/malloc.h>
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
 * On modern BHND chipsets, the PMU, GCI, and SRENG (Save/Restore Engine?)
 * register blocks are found within a dedicated PMU core (attached via
 * the AHB 'always on bus').
 * 
 * On earlier chipsets, these register blocks are found at the same
 * offsets within the ChipCommon core.
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
 * @param dev PMU device.
 * @param res The PMU device registers. The driver will maintain a borrowed
 * reference to this resource for the lifetime of the device.
 */
int
bhnd_pmu_attach(device_t dev, struct bhnd_resource *res)
{
	struct bhnd_pmu_softc		*sc;
	devclass_t			 bhnd_class;
	device_t			 core, bus;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = 0;
	sc->res = res;

	/* Find the bus-attached core */
	bhnd_class = devclass_find("bhnd");
	core = sc->dev;
	while ((bus = device_get_parent(core)) != NULL) {
		if (device_get_devclass(bus) == bhnd_class)
			break;

		core = bus;
	}

	if (core == NULL) {
		device_printf(sc->dev, "bhnd bus not found\n");
		return (ENXIO);
	}

	/* Initialize bus-dependent state */
	sc->cid = *bhnd_get_chipid(core);
	sc->chipc_dev = bhnd_find_child(bus, BHND_DEVCLASS_CC, 0);

	if (sc->chipc_dev == NULL) {
		device_printf(sc->dev, "chipcommon device not found\n");
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
	DEVMETHOD(device_detach,	bhnd_pmu_detach),
	DEVMETHOD(device_suspend,	bhnd_pmu_suspend),
	DEVMETHOD(device_resume,	bhnd_pmu_resume),

	/* BHND PMU interface */
	// TODO

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pmu, bhnd_pmu_driver, bhnd_pmu_methods, sizeof(struct bhnd_pmu_softc));
MODULE_VERSION(bhnd_pmu, 1);
