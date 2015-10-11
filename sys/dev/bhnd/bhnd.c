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
 * Broadcom Home Networking Division (HND) Bus Driver.
 * 
 * The Broadcom HND family of devices consists of both SoCs and host-connected
 * networking chipsets containing a common family of Broadcom IP cores, including
 * an integrated MIPS and/or ARM processors.
 * 
 * HND devices expose a nearly identical interface whether accessible over a native
 * SoC interconnect, or when connected via a host interface such as PCIe. As a result,
 * the majority of hardware support code should be re-usable across host drivers for HND
 * networking chipsets, as well as FreeBSD support for Broadcom MIPS/ARM HND SoCs.
 * 
 * Earlier HND models used the siba(4) on-chip interconnect, while later models
 * use bcma(4); the programming model is almost entirely independent
 * of the actual underlying interconect.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include "bhndvar.h"

static int
bhnd_probe(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_attach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_detach(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_suspend(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_resume(device_t dev)
{
	return (ENXIO);
}

static device_method_t bhnd_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_probe),
	DEVMETHOD(device_attach,	bhnd_attach),
	DEVMETHOD(device_detach,	bhnd_detach),
	DEVMETHOD(device_suspend,	bhnd_suspend),
	DEVMETHOD(device_resume,	bhnd_resume),
	DEVMETHOD_END
};
static driver_t bhnd_driver = {
	"bhnd",
	bhnd_methods,
	sizeof(struct bhnd_softc)
};
static devclass_t bhnd_devclass;
DRIVER_MODULE(bhnd, bhnd, bhnd_driver, bhnd_devclass, 0, 0);