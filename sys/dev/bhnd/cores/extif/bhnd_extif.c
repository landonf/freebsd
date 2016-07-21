/*-
 * Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
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
 * Broadcom EXTIF driver.
 * 
 * Provides a bhnd_chipc_if-compatible interface to the EXTIF core
 * used in some early siba(4)-based chipsets.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhndvar.h>

#include <dev/bhnd/cores/chipc/chipcreg.h>
#include <dev/bhnd/cores/chipc/chipcvar.h>

#include "bhnd_extifreg.h"
#include "bhnd_extifvar.h"

static struct bhnd_device_quirk bhnd_extif_quirks[];

/* Supported device identifiers */
static const struct bhnd_device bhnd_extif_devices[] = {
	BHND_DEVICE(BCM, EXTIF, NULL, bhnd_extif_quirks),
	BHND_DEVICE_END
};


/* Device quirks table */
static struct bhnd_device_quirk bhnd_extif_quirks[] = {
	BHND_DEVICE_QUIRK_END
};

static int		 bhnd_extif_add_children(struct bhnd_extif_softc *sc);
static int		 bhnd_extif_probe_caps(struct bhnd_extif_softc *sc,
			     struct chipc_caps *caps);

static int
bhnd_extif_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bhnd_extif_devices,
	     sizeof(bhnd_extif_devices[0]));
	if (id == NULL)
		return (ENXIO);

	bhnd_set_default_core_desc(dev);
	return (BUS_PROBE_DEFAULT);
}

static int
bhnd_extif_attach(device_t dev)
{
	struct bhnd_extif_softc		*sc;
	int				 error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->quirks = bhnd_device_quirks(dev, bhnd_extif_devices,
	    sizeof(bhnd_extif_devices[0]));

	/* Allocate the extif register block */
	sc->rid = 0;
	sc->res = bhnd_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "couldn't map memory resources\n");
		return (ENXIO);
	}

	/* Populate chipc-compatible capability structure */
	if ((error = bhnd_extif_probe_caps(sc, &sc->caps)))
		goto failed;

	/* Attach all supported child devices */
	if ((error = bhnd_extif_add_children(sc)))
		goto failed;

	if ((error = bus_generic_attach(dev)))
		goto failed;

	return (0);

failed:
	device_delete_children(sc->dev);
	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
	return (error);
}

static int
bhnd_extif_detach(device_t dev)
{
	struct bhnd_extif_softc	*sc;
	int			 error;

	sc = device_get_softc(dev);

	if ((error = bus_generic_detach(dev)))
		return (error);

	bhnd_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
	return (0);
}

static int
bhnd_extif_add_children(struct bhnd_extif_softc *sc)
{
	device_t	 child;

	/* PWR_CTRL is always supported */
	child = device_add_child(sc->dev, "bhnd_pmu", -1);
	if (child == NULL) {
		device_printf(sc->dev, "failed to add pmu\n");
		return (ENXIO);
	}

	/* All remaining devices are SoC-only */
	if (bhnd_get_attach_type(sc->dev) != BHND_ATTACH_NATIVE)
		return (0);

	/* TODO: UART, etc ... */

	return (0);
}

/* Probe EXTIF capabilities and initialize a chipc-compatibile
 * capability structure. */
static int
bhnd_extif_probe_caps(struct bhnd_extif_softc *sc, struct chipc_caps *caps)
{
	/* Probe Capabilities */

	/* TODO: probe UART register blocks */
	caps->num_uarts		= 0;
	caps->uart_gpio		= false;
	caps->uart_clock	= 0;

	/* TODO: supported by EXTIF? */
	caps->extbus_type	= 0x0;
	caps->jtag_master	= false;
	caps->mipseb		= false;
	caps->boot_rom		= false;

	caps->pwr_ctrl		= true;		/* always supported */
	caps->pll_type		= CHIPC_PLL_TYPE1;
	caps->backplane_64	= false;
	caps->pmu		= false;
	caps->eci		= false;


	caps->seci		= false;
	caps->gsio		= false;
	caps->aob		= false;

	/* TODO: Flash */
	caps->flash_type	= CHIPC_FLASH_NONE;	/* TODO */
	caps->cfi_width		= 0;

	/* TODO: SPROM/OTP */
	caps->sprom		= false;		/* TODO */
	caps->otp_size		= 0;			/* TODO */
	caps->nvram_src		= BHND_NVRAM_SRC_UNKNOWN;
	caps->sprom_offset	= 0;

	return (0);
}

static int
bhnd_extif_suspend(device_t dev)
{
	return (bus_generic_suspend(dev));
}

static int
bhnd_extif_resume(device_t dev)
{
	return (bus_generic_resume(dev));
}

static int
bhnd_extif_enable_sprom(device_t dev)
{
	/* Unsupported */
	return (ENODEV);
}

static void
bhnd_extif_disable_sprom(device_t dev)
{
	/* Unsupported */
	return;
}

static uint32_t
bhnd_extif_read_chipst(device_t dev)
{
	/* Unsupported */
	return (0x0);
}

static void
bhnd_extif_write_chipctrl(device_t dev, uint32_t value, uint32_t mask)
{
	/* Unsupported */
	return;
}

static struct chipc_caps *
bhnd_extif_get_caps(device_t dev)
{
	struct bhnd_extif_softc	*sc;

	sc = device_get_softc(dev);
	return (&sc->caps);
}

static device_method_t bhnd_extif_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			bhnd_extif_probe),
	DEVMETHOD(device_attach,		bhnd_extif_attach),
	DEVMETHOD(device_detach,		bhnd_extif_detach),
	DEVMETHOD(device_suspend,		bhnd_extif_suspend),
	DEVMETHOD(device_resume,		bhnd_extif_resume),

	/* ChipCommon interface */
	DEVMETHOD(bhnd_chipc_read_chipst,	bhnd_extif_read_chipst),
	DEVMETHOD(bhnd_chipc_write_chipctrl,	bhnd_extif_write_chipctrl),
	DEVMETHOD(bhnd_chipc_enable_sprom,	bhnd_extif_enable_sprom),
	DEVMETHOD(bhnd_chipc_disable_sprom,	bhnd_extif_disable_sprom),
	DEVMETHOD(bhnd_chipc_get_caps,		bhnd_extif_get_caps),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bhnd_chipc, bhnd_extif_driver, bhnd_extif_methods, sizeof(struct bhnd_extif_softc));
EARLY_DRIVER_MODULE(bhnd_chipc, bhnd, bhnd_extif_driver, bhnd_chipc_devclass, 0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
MODULE_DEPEND(bhnd_extif, bhnd, 1, 1, 1);
MODULE_VERSION(bhnd_extif, 1);
