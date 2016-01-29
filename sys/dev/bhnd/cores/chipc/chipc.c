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

#include "chipcreg.h"
#include "chipcvar.h"

devclass_t bhnd_chipc_devclass;	/**< bhnd(4) chipcommon device class */

static const struct resource_spec chipc_rspec[CHIPC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

static const struct chipc_device {
	uint16_t	 device;
} chipc_devices[] = {
	{ BHND_COREID_CC },
	{ BHND_COREID_INVALID }
};

static int
chipc_probe(device_t dev)
{
	const struct chipc_device	*id;

	for (id = chipc_devices; id->device != BHND_COREID_INVALID; id++)
	{
		if (bhnd_get_vendor(dev) == BHND_MFGID_BCM &&
		    bhnd_get_device(dev) == id->device)
		{
			bhnd_set_generic_core_desc(dev);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
chipc_attach(device_t dev)
{
	struct chipc_softc	*sc;
	bhnd_addr_t		 enum_addr;
	uint32_t		 ccid_reg;
	uint8_t			 chip_type;
	int			 error;

	sc = device_get_softc(dev);
	memcpy(sc->rspec, chipc_rspec, sizeof(sc->rspec));

	if ((error = bhnd_alloc_resources(dev, sc->rspec, sc->res)))
		return (error);

	sc->core = sc->res[0];
	
	/* Fetch our chipset identification data */
	ccid_reg = bhnd_bus_read_4(sc->core, CHIPC_ID);
	chip_type = CHIPC_GET_ATTR(ccid_reg, ID_BUS);

	switch (chip_type) {
	case BHND_CHIPTYPE_SIBA:
		/* enumeration space starts at the ChipCommon register base. */
		enum_addr = rman_get_start(sc->core->res);
		break;
	case BHND_CHIPTYPE_BCMA:
	case BHND_CHIPTYPE_BCMA_1:
		enum_addr = bhnd_bus_read_4(sc->core, CHIPC_EROMPTR);
		break;
	default:
		device_printf(dev, "unsupported chip type %hhu\n", chip_type);
		error = ENODEV;
		goto cleanup;
	}

	sc->ccid = bhnd_parse_chipid(ccid_reg, enum_addr);

	return (0);
	
cleanup:
	bhnd_release_resources(dev, sc->rspec, sc->res);
	return (error);
}

static int
chipc_detach(device_t dev)
{
	struct chipc_softc	*sc;

	sc = device_get_softc(dev);
	bhnd_release_resources(dev, sc->rspec, sc->res);

	return (0);
}

static int
chipc_suspend(device_t dev)
{
	return (0);
}

static int
chipc_resume(device_t dev)
{
	return (0);
}

static bhnd_nvram_hw_t
chipc_avail_nvram_hw(device_t dev)
{
	// TODO
	return (BHND_NVRAM_HW_NONE);
}

static device_method_t chipc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			chipc_probe),
	DEVMETHOD(device_attach,		chipc_attach),
	DEVMETHOD(device_detach,		chipc_detach),
	DEVMETHOD(device_suspend,		chipc_suspend),
	DEVMETHOD(device_resume,		chipc_resume),
	
	/* ChipCommon interface */
	DEVMETHOD(bhnd_chipc_avail_nvram_hw,	chipc_avail_nvram_hw),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_chipc, chipc_driver, chipc_methods, sizeof(struct chipc_softc));
DRIVER_MODULE(bhnd_chipc, bhnd, chipc_driver, bhnd_chipc_devclass, 0, 0);
MODULE_VERSION(bhnd_chipc, 1);
