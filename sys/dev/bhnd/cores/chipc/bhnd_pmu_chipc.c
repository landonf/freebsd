/*-
 * Copyright (c) 2016 Landon Fuller <landon@landonf.org>
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
 * ChipCommon PMU driver.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/cores/chipc/chipc.h>

#include "bhnd_chipc_if.h"
#include "bhnd_pmu_if.h"

#include "bhnd_pmuvar.h"

static void
bhnd_pmu_chipc_identify(driver_t *driver, device_t parent)
{
	struct chipc_caps *caps;

	/* PMU must be supported by the chipset, and this must not be an
	 * Always-on-Bus device that provides the PMU as a distinct core */
	caps = BHND_CHIPC_GET_CAPS(parent);
	if (!caps->pmu || caps->aob)
		return;

	if (device_find_child(parent, "bhnd_pmu", -1) != NULL)
		return;

	if (BUS_ADD_CHILD(parent, 0, "bhnd_pmu", -1) == NULL)
		device_printf(parent, "add bhnd_pmu failed\n");
}

static int
bhnd_pmu_chipc_probe(device_t dev)
{
	struct chipc_caps	*caps;
	device_t		 chipc;
	int			 error;

	/* Look for chipc parent */
	chipc = device_get_parent(dev);
	if (device_get_devclass(chipc) != devclass_find("bhnd_chipc"))
		return (ENXIO);

	/* Verify chipc capability flags */
	caps = BHND_CHIPC_GET_CAPS(chipc);
	if (!caps->pmu || caps->aob)
		return (ENXIO);

	/* Defer to default driver implementation */
	if ((error = bhnd_pmu_probe(dev)) > 0)
		return (error);

	// TODO: include PMU revision?
	device_set_desc(dev, "Broadcom ChipCommon PMU");

	return (BUS_PROBE_NOWILDCARD);
}

static device_method_t bhnd_pmu_chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,		bhnd_pmu_chipc_identify),
	DEVMETHOD(device_probe,			bhnd_pmu_chipc_probe),
	DEVMETHOD_END
};

DEFINE_CLASS_1(bhnd_pmu, bhnd_pmu_chipc_driver, bhnd_pmu_chipc_methods,
    sizeof(struct bhnd_pmu_softc), bhnd_pmu_driver);
EARLY_DRIVER_MODULE(bhnd_pmu_chipc, bhnd_chipc, bhnd_pmu_chipc_driver,
    bhnd_pmu_devclass, NULL, NULL, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);

MODULE_DEPEND(bhnd_pmu_chipc, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_pmu_chipc, 1);
