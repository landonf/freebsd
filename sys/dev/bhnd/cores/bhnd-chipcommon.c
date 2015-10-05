/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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
 * Broadcom ChipCommon driver.
 * 
 * With the exception of some very early chipsets, the ChipCommon core
 * has been included in all HND SoCs and chipsets based on the siba(4) and baxi(4)
 * interconnects, providing a common interface to chipset identification,
 * bus enumeration, UARTs, clocks, watchdog interrupts, GPIO, flash, etc.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

struct bhnd_chipcommon_softc {};

static int
bhnd_cc_probe(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_cc_attach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_cc_detach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_cc_suspend(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_cc_resume(device_t dev)
{
	return (ENXIO);
}

static device_method_t bhnd_cc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_cc_probe),
	DEVMETHOD(device_attach,	bhnd_cc_attach),
	DEVMETHOD(device_detach,	bhnd_cc_detach),
	DEVMETHOD(device_suspend,	bhnd_cc_suspend),
	DEVMETHOD(device_resume,	bhnd_cc_resume),
	DEVMETHOD_END
};
static driver_t bhnd_cc_driver = {
	"bhnd_chipcommon",
	bhnd_cc_methods,
	sizeof(struct bhnd_chipcommon_softc)
};
static devclass_t bhnd_cc_devclass;
DRIVER_MODULE(bhnd_chipcommon, bhnd, bhnd_cc_driver, bhnd_cc_devclass, 0, 0);