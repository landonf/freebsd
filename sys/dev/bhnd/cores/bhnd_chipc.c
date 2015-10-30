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

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd_device_ids.h>
#include <dev/bhnd/bhndvar.h>

#include "bhnd_chipc.h"

struct bhnd_chipc_softc {};

static const struct chipc_bhnd_device {
	uint16_t	 vendor;
	uint16_t	 device;
	uint8_t		 revision;
	const char	*desc;
} chipc_bhnd_devices[] = {
	{ JEDEC_MFGID_BCM,	BHND_COREID_CC,		BHND_HWREV_ANY,	NULL },
	{ 0,			BHND_COREID_NODEV,	BHND_HWREV_ANY,	NULL }
};

static int
bhnd_chipc_probe(device_t dev)
{
	const struct chipc_bhnd_device	*id;
	const char 			*desc;

	for (id = chipc_bhnd_devices; id->device != BHND_COREID_NODEV; id++)
	{
		if (bhnd_get_vendor(dev) == id->vendor &&
		    bhnd_get_device(dev) == id->device &&
		    (id->revision == BHND_HWREV_ANY ||
			bhnd_get_revid(dev) != id->revision))
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
	int rid = bhnd_get_port_rid(dev, 0, 0);
	struct bhnd_resource *res = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 0 /* RF_ACTIVE | RF_SHAREABLE */);

	// TODO
	device_printf(dev, "got rid=%d res=%p\n", rid, res);

	return (0);
}

static int
bhnd_chipc_detach(device_t dev)
{
	return (ENXIO);
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

static driver_t bhnd_chipc_driver = {
	BHND_CHIPC_DEVNAME,
	bhnd_chipc_methods,
	sizeof(struct bhnd_chipc_softc)
};

static devclass_t bhnd_chipc_devclass;

DRIVER_MODULE(bhnd_chipc, bcma, bhnd_chipc_driver, bhnd_chipc_devclass, 0, 0);
DRIVER_MODULE(bhnd_chipc, siba, bhnd_chipc_driver, bhnd_chipc_devclass, 0, 0);