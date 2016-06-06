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
 * Broadcom PMU driver.
 */

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

devclass_t bhnd_pmu_devclass;	/**< bhnd(4) PMU device class */

static struct bhnd_device_quirk bhnd_pmu_quirks[];

/* Supported device identifiers */
static const struct bhnd_device bhnd_pmu_devices[] = {
	BHND_DEVICE(BCM, PMU, NULL, bhnd_pmu_quirks),
	BHND_DEVICE_END
};


/* Device quirks table */
static struct bhnd_device_quirk bhnd_pmu_quirks[] = {
	BHND_DEVICE_QUIRK_END
};


static int
bhnd_pmu_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bhnd_pmu_devices,
	     sizeof(bhnd_pmu_devices[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bhnd_pmu_attach(device_t dev)
{
	struct bhnd_pmu_softc		*sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = bhnd_device_quirks(dev, bhnd_pmu_devices,
	    sizeof(bhnd_pmu_devices[0]));

	BPMU_LOCK_INIT(sc);

	return (0);
}

static int
bhnd_pmu_detach(device_t dev)
{
	struct bhnd_pmu_softc	*sc;

	sc = device_get_softc(dev);
	BPMU_LOCK_DESTROY(sc);

	return (0);
}

static int
bhnd_pmu_suspend(device_t dev)
{
	return (0);
}

static int
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
DRIVER_MODULE(bhnd_pmu, bhnd, bhnd_pmu_driver, bhnd_pmu_devclass, 0, 0);
MODULE_DEPEND(bhnd_pmu, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_pmu, 1);
