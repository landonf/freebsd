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
 * ChipCommon Power/Clock Control driver.
 * 
 * Provides a bhnd_pmu_if-compatible interface to device clocking and
 * power management on non-PMU chipsets.
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
#include <dev/bhnd/cores/pmu/bhnd_pmuvar.h>

#include "bhnd_chipc_if.h"
#include "bhnd_pmu_if.h"

struct chipc_pwrctrl_softc {
	device_t	dev;
};

static int
chipc_pwrctrl_probe(device_t dev)
{
	struct chipc_caps	*ccaps;
	device_t		 chipc;

	/* Look for chipc parent */
	chipc = device_get_parent(dev);
	if (device_get_devclass(chipc) != devclass_find("bhnd_chipc"))
		return (ENXIO);

	/* Verify chipc capability flags */
	ccaps = BHND_CHIPC_GET_CAPS(chipc);
	if (ccaps->pmu || !ccaps->pwr_ctrl)
		return (ENXIO);

	device_set_desc(dev, "Broadcom ChipCommon Power Control");
	return (BUS_PROBE_NOWILDCARD);
}

static int
chipc_pwrctrl_attach(device_t dev)
{
	// TODO
	return (ENXIO);
}

static int
chipc_pwrctrl_detach(device_t dev)
{
	// TODO
	return (ENXIO);
}

static int
chipc_pwrctrl_suspend(device_t dev)
{
	// TODO
	return (ENXIO);
}

static int
chipc_pwrctrl_resume(device_t dev)
{
	// TODO
	return (ENXIO);
}

static device_method_t chipc_pwrctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		chipc_pwrctrl_probe),
	DEVMETHOD(device_attach,	chipc_pwrctrl_attach),
	DEVMETHOD(device_detach,	chipc_pwrctrl_detach),
	DEVMETHOD(device_suspend,	chipc_pwrctrl_suspend),
	DEVMETHOD(device_resume,	chipc_pwrctrl_resume),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pmu, chipc_pwrctrl_driver, chipc_pwrctrl_methods,
    sizeof(struct chipc_pwrctrl_softc));
EARLY_DRIVER_MODULE(chipc_pwrctrl, bhnd_chipc, chipc_pwrctrl_driver,
    bhnd_pmu_devclass, NULL, NULL, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);

MODULE_DEPEND(chipc_pwrctrl, bhnd, 1, 1, 1);
MODULE_VERSION(chipc_pwrctrl, 1);
