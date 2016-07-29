/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2010, Broadcom Corporation.
 * All rights reserved.
 * 
 * This file is derived from the siutils.c source distributed with the
 * Asus RT-N16 firmware source code release.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * $Id: siutils.c,v 1.821.2.48 2011-02-11 20:59:28 Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/bhnd/bhnd.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>
#include <dev/bhnd/cores/chipc/chipcvar.h>

#include <dev/bhnd/cores/pmu/bhnd_pmuvar.h>
#include <dev/bhnd/cores/pmu/bhnd_pmureg.h>

#include "bhnd_chipc_if.h"

#include "bhnd_pwrctl_private.h"

/*
 * ChipCommon Power Control.
 * 
 * Provides a bhnd_pmu_if-compatible interface to device clocking and
 * power management on non-PMU chipsets.
 */

static struct bhnd_device_quirk pwrctl_quirks[];

/* Supported parent core device identifiers */
static const struct bhnd_device pwrctl_devices[] = {
	BHND_DEVICE(BCM, CC, "ChipCommon Power Control", pwrctl_quirks),
	BHND_DEVICE_END
};

/* Device quirks table */
static struct bhnd_device_quirk pwrctl_quirks[] = {
	BHND_CORE_QUIRK	(HWREV_LTE(5),		PWRCTL_QUIRK_PCICLK_CTL),
	BHND_CORE_QUIRK	(HWREV_RANGE(6, 9),	PWRCTL_QUIRK_SLOWCLK_CTL),
	BHND_CORE_QUIRK	(HWREV_RANGE(10, 19),	PWRCTL_QUIRK_INSTACLK_CTL),

	BHND_DEVICE_QUIRK_END
};

static int
bhnd_pwrctl_probe(device_t dev)
{
	const struct bhnd_device	*id;
	struct chipc_caps		*ccaps;
	device_t			 chipc;

	/* Look for compatible chipc parent */
	chipc = device_get_parent(dev);
	if (device_get_devclass(chipc) != devclass_find("bhnd_chipc"))
		return (ENXIO);

	if (device_get_driver(chipc) != &bhnd_chipc_driver)
		return (ENXIO);

	/* Verify chipc capability flags */
	ccaps = BHND_CHIPC_GET_CAPS(chipc);
	if (ccaps->pmu || !ccaps->pwr_ctrl)
		return (ENXIO);

	/* Check for device match */
	id = bhnd_device_lookup(dev, pwrctl_devices, sizeof(pwrctl_devices[0]));
	if (id == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
	return (BUS_PROBE_NOWILDCARD);
}

static int
bhnd_pwrctl_attach(device_t dev)
{
	struct bhnd_pwrctl_softc	*sc;
	struct chipc_softc		*chipc_sc;

	sc = device_get_softc(dev);

	sc->dev = dev;
	sc->chipc_dev = device_get_parent(dev);
	sc->quirks = bhnd_device_quirks(sc->chipc_dev, pwrctl_devices,
	    sizeof(pwrctl_devices[0]));

	/* Fetch core register block from ChipCommon parent */
	chipc_sc = device_get_softc(sc->chipc_dev);
	sc->res = chipc_sc->core;

	PWRCTL_LOCK_INIT(sc);
	return (0);
}

static int
bhnd_pwrctl_detach(device_t dev)
{
	struct bhnd_pwrctl_softc *sc = device_get_softc(dev);

	PWRCTL_LOCK_DESTROY(sc);
	return (0);
}

static int
bhnd_pwrctl_suspend(device_t dev)
{
	// TODO
	return (0);
}

static int
bhnd_pwrctl_resume(device_t dev)
{
	// TODO
	return (0);
}

static int
bhnd_pwrctl_core_req_clock(device_t dev, struct bhnd_core_pmu_info *pinfo,
    bhnd_clock clock)
{
	// TODO
	return (ENODEV);
}

static int
bhnd_pwrctl_core_en_clocks(device_t dev, struct bhnd_core_pmu_info *pinfo,
    uint32_t clocks)
{
	// TODO
	return (ENODEV);
}

static int
bhnd_pwrctl_core_release(device_t dev, struct bhnd_core_pmu_info *pinfo)
{
	// TODO
	return (ENODEV);
}

static device_method_t bhnd_pwrctl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_pwrctl_probe),
	DEVMETHOD(device_attach,		bhnd_pwrctl_attach),
	DEVMETHOD(device_detach,		bhnd_pwrctl_detach),
	DEVMETHOD(device_suspend,		bhnd_pwrctl_suspend),
	DEVMETHOD(device_resume,		bhnd_pwrctl_resume),

	/* BHND PMU interface */
	DEVMETHOD(bhnd_pmu_core_req_clock,	bhnd_pwrctl_core_req_clock),
	DEVMETHOD(bhnd_pmu_core_en_clocks,	bhnd_pwrctl_core_en_clocks),
	DEVMETHOD(bhnd_pmu_core_release,	bhnd_pwrctl_core_release),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_pmu, bhnd_pwrctl_driver, bhnd_pwrctl_methods,
    sizeof(struct bhnd_pwrctl_softc));
EARLY_DRIVER_MODULE(bhnd_pwrctl, bhnd_chipc, bhnd_pwrctl_driver,
    bhnd_pmu_devclass, NULL, NULL, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);

MODULE_DEPEND(bhnd_pwrctl, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_pwrctl, 1);
