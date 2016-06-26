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
 * ChipCommon attachment support for the bhnd(4) PMU driver.
 * 
 * Supports non-AOB ("Always-on Bus") devices that map the PMU register block
 * via the ChipCommon core, rather than vending a distinct PMU core on the
 * bhnd bus.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/cores/pmu/bhnd_pmuvar.h>

#include "bhnd_chipc_if.h"
#include "bhnd_pmu_if.h"

#include "chipc.h"

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

	/*
	 * Verify chipc capability flags:
	 * - PMU must be supported by the chipset,
	 * - This must not be an Always-on-Bus device that provides the PMU
	 *   as a distinct core 
	 */
	caps = BHND_CHIPC_GET_CAPS(chipc);
	if (!caps->pmu || caps->aob)
		return (ENXIO);

	/* Defer to default driver implementation */
	if ((error = bhnd_pmu_probe(dev)) > 0)
		return (error);

	return (BUS_PROBE_NOWILDCARD);
}

static device_method_t bhnd_pmu_chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_pmu_chipc_probe),
	DEVMETHOD_END
};

DEFINE_CLASS_1(bhnd_pmu, bhnd_pmu_chipc_driver, bhnd_pmu_chipc_methods,
    sizeof(struct bhnd_pmu_softc), bhnd_pmu_driver);
EARLY_DRIVER_MODULE(bhnd_pmu_chipc, bhnd_chipc, bhnd_pmu_chipc_driver,
    bhnd_pmu_devclass, NULL, NULL, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);

MODULE_DEPEND(bhnd_pmu_chipc, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_pmu_chipc, 1);
