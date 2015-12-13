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
 * has been included in all HND SoCs and chipsets based on the siba(4) 
 * and bcma(4) interconnects, providing a common interface to chipset 
 * identification, bus enumeration, UARTs, clocks, watchdog interrupts, GPIO, 
 * flash, etc.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_chipcreg.h"
#include "bhnd_chipcvar.h"

static const struct resource_spec bhnd_chipc_rspec[BHND_CHIPC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

static const struct bhnd_chipc_device {
	uint16_t	 device;
	const char	*desc;
} bhnd_chipc_devices[] = {
	{ BHND_COREID_CC,	NULL },
	{ BHND_COREID_INVALID,	NULL }
};

static int
bhnd_chipc_probe(device_t dev)
{
	const struct bhnd_chipc_device	*id;
	const char 			*desc;

	for (id = bhnd_chipc_devices; id->device != BHND_COREID_INVALID; id++)
	{
		if (bhnd_get_vendor(dev) == BHND_MFGID_BCM &&
		    bhnd_get_device(dev) == id->device)
		{
			if (id->desc == NULL)
				desc = bhnd_get_device_name(dev);
			else
				desc = id->desc;
		
			device_set_desc(dev, desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
bhnd_chipc_attach(device_t dev)
{
	struct bhnd_chipc_softc	*sc;
	struct bhnd_resource	*r;
	int			 error;

	sc = device_get_softc(dev);
	memcpy(sc->rspec, bhnd_chipc_rspec, sizeof(sc->rspec));

	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	// TODO
	r = sc->res[0];
	device_printf(dev, "got rid=%d res=%p\n", sc->rspec[0].rid, r);
	
	uint32_t chipc	= bhnd_bus_read_4(dev, r, CHIPC_ID);
	uint16_t chip	= CHIPC_GET_ATTR(chipc, ID_CHIP);
	uint8_t rev	= CHIPC_GET_ATTR(chipc, ID_REV);
	uint8_t pkg	= CHIPC_GET_ATTR(chipc, ID_PKG);
	uint8_t ncore	= CHIPC_GET_ATTR(chipc, ID_NUMCORE);
	uint8_t bus	= CHIPC_GET_ATTR(chipc, ID_BUS);
	device_printf(dev, "chip=0x%hx rev=0x%hhx pkg=0x%hhx ncore=0x%hhu bus=0x%hhx\n", chip, rev, pkg, ncore, bus);

	return (0);
}

static int
bhnd_chipc_detach(device_t dev)
{
	struct bhnd_chipc_softc	*sc;

	sc = device_get_softc(dev);
	bhnd_release_resources(dev, sc->rspec, sc->res);

	return (0);
}

static int
bhnd_chipc_suspend(device_t dev)
{
	return (ENXIO);
}

static int
bhnd_chipc_resume(device_t dev)
{
	return (ENXIO);
}

static device_method_t bhnd_chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bhnd_chipc_probe),
	DEVMETHOD(device_attach,	bhnd_chipc_attach),
	DEVMETHOD(device_detach,	bhnd_chipc_detach),
	DEVMETHOD(device_suspend,	bhnd_chipc_suspend),
	DEVMETHOD(device_resume,	bhnd_chipc_resume),
	DEVMETHOD_END
};

static devclass_t bhnd_chipc_devclass;

DEFINE_CLASS_0(bhnd_cc, bhnd_chipc_driver, bhnd_chipc_methods, sizeof(struct bhnd_chipc_softc));
DRIVER_MODULE(bhnd_cc, bhnd, bhnd_chipc_driver, bhnd_chipc_devclass, 0, 0);
